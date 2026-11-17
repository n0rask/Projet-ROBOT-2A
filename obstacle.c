
#include <msp430.h>
#include <stdint.h>
#include "obstacle.h"

/*
 * CONFIGURATION A VERIFIER SUR LA SAMBOARD
 *
 * Exemple provisoire :
 * P1.3 = entree analogique A3
 *
 * Le GP2D120 doit etre branche sur une entree ADC10.
 */
#define OBSTACLE_CANAL_ADC     3

/*
 * Seuil provisoire de detection.
 * A determiner experimentalement.
 */
#define OBSTACLE_SEUIL_DEFAUT  500

/*
 * Nombre de lectures consecutives necessaires
 * pour confirmer un changement d'etat.
 */
#define OBSTACLE_CONFIRMATION  2

/* Variables internes */
static uint16_t seuil_obstacle = OBSTACLE_SEUIL_DEFAUT;

static uint8_t obstacle_present = 0;
static uint8_t compteur_confirmation = 0;
static uint8_t dernier_etat_candidat = 0;


/*
 * Initialisation du module obstacle.
 *
 * L'ADC10 est configure dans ligne_init().
 * On ne reconfigure donc pas l'ADC ici.
 */
void obstacle_init(void)
{
    obstacle_present = 0;
    compteur_confirmation = 0;
    dernier_etat_candidat = 0;
}


/*
 * Lecture du capteur GP2D120.
 *
 * ATTENTION :
 * Cette fonction utilise l'ADC10 partage avec le module ligne.
 * Les lectures doivent etre effectuees sans concurrence.
 */
static uint16_t obstacle_lire_adc(void)
{
    uint16_t valeur;

    /* Selection du canal analogique */
    ADC10CTL0 &= ~ENC;
    ADC10CTL1 = (OBSTACLE_CANAL_ADC << 12);

    /* Demarrage de la conversion */
    ADC10CTL0 |= ENC | ADC10SC;

    /* Attente de fin de conversion */
    while (ADC10CTL1 & ADC10BUSY)
    {
    }

    valeur = ADC10MEM;

    return valeur;
}


/*
 * Mise a jour de la detection.
 *
 * Cette fonction doit etre appelee regulierement
 * depuis navigation.c.
 */
void obstacle_update(void)
{
    uint16_t valeur;
    uint8_t candidat;

    valeur = obstacle_lire_adc();

    /*
     * Le seuil devra etre ajuste apres les essais.
     * Ici, une valeur ADC superieure au seuil
     * signifie provisoirement qu'un obstacle est proche.
     */
    if (valeur >= seuil_obstacle)
    {
        candidat = 1;
    }
    else
    {
        candidat = 0;
    }

    /* Confirmation du changement d'etat */
    if (candidat == dernier_etat_candidat)
    {
        if (compteur_confirmation < OBSTACLE_CONFIRMATION)
        {
            compteur_confirmation++;
        }
    }
    else
    {
        dernier_etat_candidat = candidat;
        compteur_confirmation = 1;
    }

    /* Validation de l'etat */
    if (compteur_confirmation >= OBSTACLE_CONFIRMATION)
    {
        obstacle_present = candidat;
    }
}


/*
 * Indique si un obstacle est present.
 */
uint8_t obstacle_detecte(void)
{
    return obstacle_present;
}


/*
 * Permet de modifier le seuil depuis le programme principal.
 */
void obstacle_regler_seuil(uint16_t seuil)
{
    if (seuil <= 1023)
    {
        seuil_obstacle = seuil;
    }
}
