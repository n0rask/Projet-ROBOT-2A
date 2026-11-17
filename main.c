#include <msp430.h>
#include <stdint.h>

#include "navigation.h"

/* État du bouton de démarrage */
static uint8_t bouton_deja_appuye = 0;

int main(void)
{
    /* Arrêt du watchdog */
    WDTCTL = WDTPW | WDTHOLD;

    /* Configuration de l'horloge à 1 MHz */
    if (CALBC1_1MHZ != 0xFF)
    {
        DCOCTL = 0;
        BCSCTL1 = CALBC1_1MHZ;
        DCOCTL = CALDCO_1MHZ;
    }

    /* Configuration du bouton S2 : P1.3 */
    P1DIR &= ~BIT3;       /* Entrée */
    P1REN |= BIT3;        /* Résistance activée */
    P1OUT |= BIT3;        /* Pull-up */

    /* Initialisation de la navigation et des autres modules */
    navigation_init();

    /* Activation des interruptions */
    __enable_interrupt();

    /* Boucle principale */
    while (1)
    {
        /* S2 est actif à l'état bas */
        if ((P1IN & BIT3) == 0)
        {
            if (bouton_deja_appuye == 0)
            {
                bouton_deja_appuye = 1;

                /* Démarrage du robot */
                navigation_demarrer();
            }
        }
        else
        {
            bouton_deja_appuye = 0;
        }

        /* Mise à jour de la navigation */
        navigation_update();
    }
}