/*
 * moteurs.c - Commande des deux moteurs DC
 *
 * Affectation des broches d'apres le manuel de la SAM Board v3 :
 *   P2.1 : sens moteur A     P2.2 : PWM moteur A (TA1.1)
 *   P2.4 : PWM moteur B (TA1.2)   P2.5 : sens moteur B
 *
 * A VERIFIER SUR LE ROBOT (voir les defines ci-dessous) :
 *   - quel moteur (A ou B) est le gauche
 *   - a quel niveau de la broche de sens le moteur tourne vers l'avant
 */

#include <msp430.h>
#include "moteurs.h"

/* ---------- Parametres modifiables ---------- */

/* 1 : le moteur A est le moteur gauche (B = droit). 0 : l'inverse. */
#ifndef MOTEUR_A_EST_GAUCHE
#define MOTEUR_A_EST_GAUCHE   1
#endif

/* Niveau de la broche de sens qui fait avancer le robot (1 = haut, 0 = bas).
 * Si un moteur tourne a l'envers pendant robot_avancer(), inverser la valeur. */
#ifndef MOTEUR_A_AVANT_NIVEAU
#define MOTEUR_A_AVANT_NIVEAU 1
#endif
#ifndef MOTEUR_B_AVANT_NIVEAU
#define MOTEUR_B_AVANT_NIVEAU 1
#endif

/* Periode PWM en nombre de ticks de SMCLK.
 * 100 ticks => la vitesse en % est directement la valeur du comparateur.
 * Frequence PWM = SMCLK / 100 (10 kHz a 1 MHz, limite de la carte : 250 kHz). */
#define PERIODE_PWM           100

/* ---------- Broches ---------- */
#define MA_SENS   BIT1   /* P2.1 */
#define MA_PWM    BIT2   /* P2.2 -> TA1.1 */
#define MB_PWM    BIT4   /* P2.4 -> TA1.2 */
#define MB_SENS   BIT5   /* P2.5 */

/* ---------- Fonctions internes ---------- */

/* Convertit une consigne (-100..100) en rapport cyclique (0..100) et en sens. */
static uint8_t valeur_absolue_limitee(int16_t v)
{
    if (v < 0)   v = -v;
    if (v > 100) v = 100;
    return (uint8_t)v;
}

static void commande_moteur_a(int16_t v)
{
    uint8_t avant = (v >= 0);

    if (avant == (MOTEUR_A_AVANT_NIVEAU != 0)) P2OUT |= MA_SENS;
    else                                       P2OUT &= ~MA_SENS;

    TA1CCR1 = valeur_absolue_limitee(v);
}

static void commande_moteur_b(int16_t v)
{
    uint8_t avant = (v >= 0);

    if (avant == (MOTEUR_B_AVANT_NIVEAU != 0)) P2OUT |= MB_SENS;
    else                                       P2OUT &= ~MB_SENS;

    TA1CCR2 = valeur_absolue_limitee(v);
}

/* ---------- Fonctions publiques ---------- */

void robot_init_moteurs(void)
{
    /* Sens : sorties numeriques, niveau bas au depart */
    P2SEL  &= ~(MA_SENS | MB_SENS);
    P2SEL2 &= ~(MA_SENS | MB_SENS);
    P2DIR  |=  (MA_SENS | MB_SENS);
    P2OUT  &= ~(MA_SENS | MB_SENS);

    /* PWM : sorties du Timer1_A (fonction alternative). P2SEL est modifie avec |=
     * pour ne pas toucher aux bits 6 et 7 (quartz). */
    P2DIR   |=  (MA_PWM | MB_PWM);
    P2SEL   |=  (MA_PWM | MB_PWM);
    P2SEL2  &= ~(MA_PWM | MB_PWM);

    /* Timer1_A : horloge SMCLK, mode up, periode = PERIODE_PWM ticks */
    TA1CCR0  = PERIODE_PWM - 1;
    TA1CCTL1 = OUTMOD_7;          /* reset/set : rapport cyclique = TA1CCR1 / PERIODE_PWM */
    TA1CCTL2 = OUTMOD_7;
    TA1CCR1  = 0;                 /* moteurs a l'arret au demarrage */
    TA1CCR2  = 0;
    TA1CTL   = TASSEL_2 | MC_1 | TACLR;
}

void moteurs_set(int8_t gauche, int8_t droite)
{
#if MOTEUR_A_EST_GAUCHE
    commande_moteur_a(gauche);
    commande_moteur_b(droite);
#else
    commande_moteur_a(droite);
    commande_moteur_b(gauche);
#endif
}

void robot_avancer(uint8_t vitesse)
{
    int8_t v = (vitesse > 100) ? 100 : (int8_t)vitesse;
    moteurs_set(v, v);
}

void robot_reculer(uint8_t vitesse)
{
    int8_t v = (vitesse > 100) ? 100 : (int8_t)vitesse;
    moteurs_set(-v, -v);
}

void robot_tourner_gauche(uint8_t vitesse)
{
    int8_t v = (vitesse > 100) ? 100 : (int8_t)vitesse;
    moteurs_set(-v, v);     /* roue gauche en arriere, droite en avant */
}

void robot_tourner_droite(uint8_t vitesse)
{
    int8_t v = (vitesse > 100) ? 100 : (int8_t)vitesse;
    moteurs_set(v, -v);     /* roue gauche en avant, droite en arriere */
}

void moteurs_arreter(void)
{
    moteurs_set(0, 0);
}
