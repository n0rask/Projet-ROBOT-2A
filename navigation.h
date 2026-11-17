
#ifndef NAVIGATION_H
#define NAVIGATION_H

#include <stdint.h>

/* Etats possibles de la navigation */
typedef enum
{
    NAV_ARRET,
    NAV_SUIVI_LIGNE,
    NAV_ARRET_LIVRAISON,
    NAV_ATTENTE_OBSTACLE,
    NAV_TERMINE,
    NAV_TIMEOUT,
    NAV_ERREUR
} navigation_etat_t;

/* Initialise les differents modules du robot */
void navigation_init(void);

/* Demarre le parcours apres l'appui sur S2 */
void navigation_demarrer(void);

/* Met a jour la navigation, sans attente bloquante */
void navigation_update(void);

/* Renvoie l'etat actuel de la navigation */
navigation_etat_t navigation_get_etat(void);

/* Renvoie le nombre de livraisons effectuees */
uint8_t navigation_get_livraisons(void);

#endif
