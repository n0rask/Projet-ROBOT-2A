/*
 * moteurs.h - Commande des deux moteurs DC (SAM Board v3 + Launchpad MSP430G2553)
 *
 * Vitesse : toujours en pourcentage, de 0 a 100.
 * Pour robot_set_moteurs() : de -100 (marche arriere a fond) a +100 (avant a fond).
 *
 * Ressources utilisees par ce module (a ne pas reutiliser ailleurs) :
 *   - Timer1_A (PWM sur TA1.1 et TA1.2)
 *   - P2.1 / P2.2 : sens et PWM du moteur A
 *   - P2.4 / P2.5 : PWM et sens du moteur B
 */

#ifndef MOTEURS_H_
#define MOTEURS_H_

#include <stdint.h>

/* A appeler une seule fois au demarrage (apres arret du watchdog et reglage de l'horloge). */
void robot_init_moteurs(void);

/* Commande independante des deux moteurs : indispensable pour le suivi de ligne
 * (le module ligne corrige la trajectoire en donnant des vitesses differentes). */
void moteurs_set(int8_t gauche, int8_t droite);

/* Deplacements simples. */
void robot_avancer(uint8_t vitesse);
void robot_reculer(uint8_t vitesse);
void robot_tourner_gauche(uint8_t vitesse);   /* pivot sur place */
void robot_tourner_droite(uint8_t vitesse);   /* pivot sur place */
void moteurs_arreter(void);

#endif /* MOTEURS_H_ */
