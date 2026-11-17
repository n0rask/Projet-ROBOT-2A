/*
 * main_test_moteurs.c - Programme de TEST du module moteurs, a utiliser seul.
 *
 * Mode d'emploi :
 *   1. Mettre le robot sur cales (roues en l'air) ou sur un grand espace libre.
 *   2. Dans le projet CCS, exclure temporairement main.c du build
 *      (clic droit > Exclude from Build) et inclure ce fichier a la place.
 *   3. Verifier : avancer = les 2 roues vers l'avant, gauche/droite = bon sens.
 *      Sinon, corriger MOTEUR_A_EST_GAUCHE / MOTEUR_x_AVANT_NIVEAU dans moteurs.c.
 *   4. Remettre main.c dans le build une fois les tests termines.
 */

#include <msp430.h>
#include "moteurs.h"

#define PAUSE_1S()   __delay_cycles(1000000)   /* 1 s a 1 MHz */

int main(void)
{
    WDTCTL = WDTPW | WDTHOLD;          /* arret du watchdog */

    BCSCTL1 = CALBC1_1MHZ;             /* horloge calibree a 1 MHz */
    DCOCTL  = CALDCO_1MHZ;

    robot_init_moteurs();

    while (1)
    {
        robot_avancer(50);          PAUSE_1S();
        robot_arreter();            PAUSE_1S();

        robot_reculer(50);          PAUSE_1S();
        robot_arreter();            PAUSE_1S();

        robot_tourner_gauche(40);   PAUSE_1S();
        robot_arreter();            PAUSE_1S();

        robot_tourner_droite(40);   PAUSE_1S();
        robot_arreter();            PAUSE_1S();

        /* Virage en courbe : roue gauche plus lente que la droite */
        robot_set_moteurs(30, 60);  PAUSE_1S();
        robot_arreter();            PAUSE_1S();
    }
}
