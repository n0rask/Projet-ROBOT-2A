
#include <msp430.h>
#include <stdint.h>

#include "navigation.h"
#include "temporisation.h"
#include "ligne.h"
#include "obstacle.h"
#include "moteurs.h"

/* Durees du parcours */
#define NAV_DUREE_LIVRAISON_MS    3000UL
#define NAV_DUREE_MAX_MS         60000UL

/* Periode de mise a jour des capteurs */
#define NAV_PERIODE_MS           LIGNE_PERIODE_MS

/* Nombre de livraisons a effectuer */
#define NAV_NOMBRE_LIVRAISONS    3

/* Variables internes */
static navigation_etat_t etat_navigation = NAV_ARRET;

static unsigned long debut_parcours = 0;
static unsigned long debut_livraison = 0;
static unsigned long dernier_tick = 0;

static uint8_t nombre_livraisons = 0;


/*
 * Initialisation generale des modules.
 *
 * ATTENTION :
 * L'horloge SMCLK doit etre configuree a 1 MHz
 * avant cet appel.
 *
 * Les interruptions globales seront activees
 * dans main.c apres l'initialisation.
 */
void navigation_init(void)
{
    temporisation_init();

    robot_init_moteurs();

    ligne_init();

    obstacle_init();

    robot_arreter();

    etat_navigation = NAV_ARRET;
    nombre_livraisons = 0;
}


/*
 * Demarrage du parcours.
 * Cette fonction sera appelee apres l'appui sur S2.
 */
void navigation_demarrer(void)
{
    if (etat_navigation != NAV_ARRET)
    {
        return;
    }

    debut_parcours = temporisation_get_ms();
    dernier_tick = debut_parcours;

    nombre_livraisons = 0;

    ligne_start();

    etat_navigation = NAV_SUIVI_LIGNE;
}


/*
 * Arret definitif du robot.
 */
static void navigation_arreter(void)
{
    robot_arreter();
    etat_navigation = NAV_TERMINE;
}


/*
 * Mise a jour principale de la navigation.
 *
 * Cette fonction doit etre appelee regulierement
 * dans la boucle while(1) de main.c.
 */
void navigation_update(void)
{
    unsigned long maintenant;

    maintenant = temporisation_get_ms();

    /* Aucun traitement si le parcours n'est pas actif */
    if (etat_navigation == NAV_ARRET ||
        etat_navigation == NAV_TERMINE ||
        etat_navigation == NAV_TIMEOUT ||
        etat_navigation == NAV_ERREUR)
    {
        return;
    }

    /* Limite totale du parcours : 60 secondes */
    if (temporisation_ecoulee(debut_parcours,
                              NAV_DUREE_MAX_MS))
    {
        robot_arreter();
        etat_navigation = NAV_TIMEOUT;
        return;
    }

    /* Arret temporaire sur une zone de livraison */
    if (etat_navigation == NAV_ARRET_LIVRAISON)
    {
        robot_arreter();

        if (temporisation_ecoulee(debut_livraison,
                                  NAV_DUREE_LIVRAISON_MS))
        {
            ligne_start();
            dernier_tick = maintenant;
            etat_navigation = NAV_SUIVI_LIGNE;
        }

        return;
    }

    /*
     * Les capteurs sont lus toutes les 5 ms.
     * La lecture de l'ADC est sequentielle :
     * obstacle_update() puis ligne_update().
     */
    if (!temporisation_ecoulee(dernier_tick,
                               NAV_PERIODE_MS))
    {
        return;
    }

    dernier_tick = maintenant;

    /* Surveillance de l'obstacle */
    obstacle_update();

    if (obstacle_detecte())
    {
        robot_arreter();
        etat_navigation = NAV_ATTENTE_OBSTACLE;
        return;
    }

    /* Attente de disparition de l'obstacle */
    if (etat_navigation == NAV_ATTENTE_OBSTACLE)
    {
        ligne_start();
        etat_navigation = NAV_SUIVI_LIGNE;
        return;
    }

    /* Suivi normal de la ligne */
    ligne_update();

    /* Verification d'une perte definitive de la ligne */
    if (ligne_echec())
    {
        robot_arreter();
        etat_navigation = NAV_ERREUR;
        return;
    }

    /* Detection d'une zone blanche double */
    if (ligne_blanc_double_detecte())
    {
        ligne_acquitter_blanc_double();

        /*
         * Les trois premieres zones sont les livraisons.
         * La zone suivante correspond a la base finale.
         */
        if (nombre_livraisons < NAV_NOMBRE_LIVRAISONS)
        {
            nombre_livraisons++;

            robot_arreter();

            debut_livraison = temporisation_get_ms();

            etat_navigation = NAV_ARRET_LIVRAISON;
        }
        else
        {
            /* Quatrieme zone : arrivee a la base */
            navigation_arreter();
        }
    }
}


/*
 * Renvoie l'etat de la navigation.
 */
navigation_etat_t navigation_get_etat(void)
{
    return etat_navigation;
}


/*
 * Renvoie le nombre de livraisons effectuees.
 */
uint8_t navigation_get_livraisons(void)
{
    return nombre_livraisons;
}
