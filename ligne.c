/*
 * ligne.c - Suivi de ligne par machine d'etat (membre 2)
 *
 * Convention : "blanc" = 1, "noir" = 0.
 * Capteurs : un de chaque cote de la ligne blanche. Centre -> (0,0).
 *
 * DEPENDANCES (a confirmer avec le membre 1) :
 *   void moteurs_set(int16_t vitesse_g, int16_t vitesse_d);  // -100..+100
 *   void moteurs_arreter(void);
 *
 * ATTENTION ADC : il n'y a qu'un ADC10. Si le membre 3 lit aussi le
 * GP2D120 en analogique, il doit utiliser la meme fonction de lecture
 * (selection de canal avant chaque conversion, comme ici) et ne jamais
 * etre appele depuis une interruption qui preempte ligne_update().
 */

#include <msp430.h>
#include "ligne.h"
#include "moteurs.h"

/* ------------------------------------------------------------------ */
/* Constantes derivees                                                 */
/* ------------------------------------------------------------------ */

#define MS_EN_TICKS(ms)   ((uint16_t)(((ms) + LIGNE_PERIODE_MS - 1) / LIGNE_PERIODE_MS))

#define T_CORRECTION_MAX  MS_EN_TICKS(LIGNE_T_CORRECTION_MAX_MS)
#define T_PERDU_MAX       MS_EN_TICKS(LIGNE_T_PERDU_MAX_MS)
#define T_SANS_BLANC_MAX  MS_EN_TICKS(LIGNE_T_SANS_BLANC_MAX_MS)
#define T_BLOCAGE         MS_EN_TICKS(LIGNE_T_BLOCAGE_MS)

#define COTE_GAUCHE  0
#define COTE_DROITE  1

/* ------------------------------------------------------------------ */
/* Variables internes                                                  */
/* ------------------------------------------------------------------ */

static volatile ligne_etat_t etat = LIGNE_ARRET;

/* Seuils (hysteresis) et references de calibration, par capteur */
static uint16_t seuil_haut[2] = { LIGNE_SEUIL_HAUT_DEFAUT, LIGNE_SEUIL_HAUT_DEFAUT };
static uint16_t seuil_bas[2]  = { LIGNE_SEUIL_BAS_DEFAUT,  LIGNE_SEUIL_BAS_DEFAUT  };
static uint16_t ref_noir[2]   = { 0, 0 };
static uint16_t ref_blanc[2]  = { 1023, 1023 };

/* Etat des capteurs apres hysteresis puis anti-rebond */
static uint8_t blanc_g = 0, blanc_d = 0;       /* etat valide (stable)        */
static uint8_t cand_g  = 0, cand_d  = 0;       /* etat candidat               */
static uint8_t cand_compteur = 0;

static uint8_t  dernier_cote = COTE_GAUCHE;    /* dernier cote ou du blanc a ete vu */
static uint16_t ticks_etat = 0;                /* duree dans l'etat courant   */
static uint16_t ticks_sans_blanc = 0;          /* duree sans aucun blanc      */
static uint16_t inhibition = 0;                /* temps mort blanc double     */

static volatile uint8_t flag_blanc_double = 0;
static volatile uint8_t flag_echec = 0;

/* ------------------------------------------------------------------ */
/* Acquisition ADC                                                     */
/* ------------------------------------------------------------------ */

static uint16_t adc_lire_canal(uint8_t canal)
{
    ADC10CTL0 &= ~ENC;                         /* ENC=0 pour changer de canal */
    ADC10CTL1 = ((uint16_t)canal) << 12;       /* INCHx                       */
    ADC10CTL0 |= ENC | ADC10SC;                /* demarre la conversion       */
    while (ADC10CTL1 & ADC10BUSY) { }
    return ADC10MEM;
}

/* Lecture moyennee, ramenee a l'echelle "1023 = blanc" */
static uint16_t lire_capteur(uint8_t canal)
{
    uint32_t somme = 0;
    uint8_t i;
    for (i = 0; i < LIGNE_NB_ECHANTILLONS; i++) {
        somme += adc_lire_canal(canal);
    }
    uint16_t v = (uint16_t)(somme / LIGNE_NB_ECHANTILLONS);
#if LIGNE_BLANC_TENSION_BASSE
    v = 1023 - v;
#endif
    return v;
}

uint16_t ligne_lire_gauche(void) { return lire_capteur(LIGNE_CANAL_GAUCHE); }
uint16_t ligne_lire_droite(void) { return lire_capteur(LIGNE_CANAL_DROIT);  }

/* ------------------------------------------------------------------ */
/* Initialisation                                                      */
/* ------------------------------------------------------------------ */

void ligne_init(void)
{
    /* Broches analogiques */
    ADC10AE0 |= (1 << LIGNE_CANAL_GAUCHE) | (1 << LIGNE_CANAL_DROIT);
    P1DIR &= ~((1 << LIGNE_CANAL_GAUCHE) | (1 << LIGNE_CANAL_DROIT));

    /* ADC10 : reference Vcc/Vss (3.3 V), temps d'echantillonnage 16 clk,
     * horloge ADC10OSC, ADC allume.                                      */
    ADC10CTL0 = SREF_0 | ADC10SHT_2 | ADC10ON;
    ADC10CTL1 = ADC10DIV_0 | ADC10SSEL_0;

#ifdef LIGNE_DEBUG_LEDS
    P1DIR |= BIT0 | BIT6;
    P1OUT &= ~(BIT0 | BIT6);
#endif

    etat = LIGNE_ARRET;
    flag_blanc_double = 0;
    flag_echec = 0;
    inhibition = 0;
}

/* ------------------------------------------------------------------ */
/* Calibration                                                         */
/* ------------------------------------------------------------------ */

static uint16_t moyenne_longue(uint8_t canal)
{
    uint32_t somme = 0;
    uint8_t i;
    for (i = 0; i < 32; i++) {
        somme += lire_capteur(canal);
        __delay_cycles(2000);                  /* ~2 ms a 1 MHz */
    }
    return (uint16_t)(somme / 32);
}

void ligne_calibrer_noir(void)
{
    ref_noir[COTE_GAUCHE] = moyenne_longue(LIGNE_CANAL_GAUCHE);
    ref_noir[COTE_DROITE] = moyenne_longue(LIGNE_CANAL_DROIT);
}

void ligne_calibrer_blanc(void)
{
    ref_blanc[COTE_GAUCHE] = moyenne_longue(LIGNE_CANAL_GAUCHE);
    ref_blanc[COTE_DROITE] = moyenne_longue(LIGNE_CANAL_DROIT);
}

void ligne_calculer_seuils(void)
{
    uint8_t c;
    for (c = 0; c < 2; c++) {
        if (ref_blanc[c] > ref_noir[c] + 100) {          /* calibration plausible */
            uint16_t delta = ref_blanc[c] - ref_noir[c];
            seuil_bas[c]  = ref_noir[c] + (delta * 4) / 10;   /* 40 % */
            seuil_haut[c] = ref_noir[c] + (delta * 6) / 10;   /* 60 % */
        } else {                                          /* sinon valeurs par defaut */
            seuil_bas[c]  = LIGNE_SEUIL_BAS_DEFAUT;
            seuil_haut[c] = LIGNE_SEUIL_HAUT_DEFAUT;
        }
    }
}

/* ------------------------------------------------------------------ */
/* Interpretation des capteurs : hysteresis + anti-rebond              */
/* ------------------------------------------------------------------ */

static uint8_t appliquer_hysteresis(uint16_t valeur, uint8_t c, uint8_t precedent)
{
    if (valeur > seuil_haut[c]) return 1;
    if (valeur < seuil_bas[c])  return 0;
    return precedent;                          /* zone intermediaire : on garde */
}

static void acquerir_capteurs(void)
{
    uint8_t g = appliquer_hysteresis(ligne_lire_gauche(), COTE_GAUCHE, cand_g);
    uint8_t d = appliquer_hysteresis(ligne_lire_droite(), COTE_DROITE, cand_d);

    if (g == cand_g && d == cand_d) {
        if (cand_compteur < 255) cand_compteur++;
    } else {
        cand_g = g;
        cand_d = d;
        cand_compteur = 1;
    }

    if (cand_compteur >= LIGNE_NB_CONFIRMATION) {
        blanc_g = cand_g;
        blanc_d = cand_d;
    }
}

/* ------------------------------------------------------------------ */
/* Actions moteurs liees a chaque etat                                 */
/* ------------------------------------------------------------------ */

static void appliquer_moteurs(ligne_etat_t e)
{
    switch (e) {
    case LIGNE_AVANCE:
    case LIGNE_BLANC_DOUBLE:                   /* on continue tout droit */
        moteurs_set(LIGNE_V_NOMINALE, LIGNE_V_NOMINALE);
        break;

    case LIGNE_CORRIGE_GAUCHE:                 /* ligne cote gauche : on tourne a gauche */
        moteurs_set(LIGNE_V_ROUE_LENTE, LIGNE_V_NOMINALE);
        break;

    case LIGNE_CORRIGE_DROITE:
        moteurs_set(LIGNE_V_NOMINALE, LIGNE_V_ROUE_LENTE);
        break;

    case LIGNE_PERDU:                          /* pivot vers le dernier cote vu */
        if (dernier_cote == COTE_GAUCHE) {
            moteurs_set(-LIGNE_V_PIVOT, LIGNE_V_PIVOT);
        } else {
            moteurs_set(LIGNE_V_PIVOT, -LIGNE_V_PIVOT);
        }
        break;

    case LIGNE_ARRET:
    default:
        moteurs_arreter();
        break;
    }
}

/* Changement d'etat : remet le chrono a zero et agit sur les moteurs */
static void entrer_etat(ligne_etat_t nouveau)
{
    etat = nouveau;
    ticks_etat = 0;

    if (nouveau == LIGNE_BLANC_DOUBLE && inhibition == 0) {
        flag_blanc_double = 1;                 /* un seul signalement par croisement/cercle */
    }
    appliquer_moteurs(nouveau);
}

/* ------------------------------------------------------------------ */
/* Machine d'etat                                                      */
/* ------------------------------------------------------------------ */

void ligne_start(void)
{
    flag_echec = 0;
    cand_compteur = 0;
    ticks_sans_blanc = 0;
    entrer_etat(LIGNE_AVANCE);
}

void ligne_stop(void)
{
    entrer_etat(LIGNE_ARRET);
}

ligne_etat_t ligne_get_etat(void)
{
    return etat;
}

void ligne_update(void)
{
    if (etat == LIGNE_ARRET) return;

    acquerir_capteurs();

    uint8_t g = blanc_g;
    uint8_t d = blanc_d;

    /* Memoire du dernier cote (seulement si un seul capteur voit du blanc) */
    if (g && !d) dernier_cote = COTE_GAUCHE;
    if (d && !g) dernier_cote = COTE_DROITE;

    /* Chronos */
    if (ticks_etat < 0xFFFF) ticks_etat++;
    if (g || d) ticks_sans_blanc = 0;
    else if (ticks_sans_blanc < 0xFFFF) ticks_sans_blanc++;
    if (inhibition > 0) inhibition--;

    switch (etat) {

    case LIGNE_AVANCE:
        if (g && d)        entrer_etat(LIGNE_BLANC_DOUBLE);
        else if (g)        entrer_etat(LIGNE_CORRIGE_GAUCHE);
        else if (d)        entrer_etat(LIGNE_CORRIGE_DROITE);
        else if (T_SANS_BLANC_MAX > 0 && ticks_sans_blanc > T_SANS_BLANC_MAX)
                           entrer_etat(LIGNE_PERDU);
        break;

    case LIGNE_CORRIGE_GAUCHE:
        if (g && d)                      entrer_etat(LIGNE_BLANC_DOUBLE);
        else if (!g && !d)               entrer_etat(LIGNE_AVANCE);      /* recentre */
        else if (!g && d)                entrer_etat(LIGNE_CORRIGE_DROITE);
        else if (ticks_etat > T_CORRECTION_MAX) entrer_etat(LIGNE_PERDU);
        break;

    case LIGNE_CORRIGE_DROITE:
        if (g && d)                      entrer_etat(LIGNE_BLANC_DOUBLE);
        else if (!g && !d)               entrer_etat(LIGNE_AVANCE);
        else if (g && !d)                entrer_etat(LIGNE_CORRIGE_GAUCHE);
        else if (ticks_etat > T_CORRECTION_MAX) entrer_etat(LIGNE_PERDU);
        break;

    case LIGNE_BLANC_DOUBLE:
        if (!(g && d)) {
            /* sortie du blanc : AVANCE reevaluera les capteurs au tick suivant */
            entrer_etat(LIGNE_AVANCE);
        }
        break;

    case LIGNE_PERDU:
        if (g && d) {
            entrer_etat(LIGNE_BLANC_DOUBLE);
        } else if (g) {
            entrer_etat(LIGNE_CORRIGE_GAUCHE);
        } else if (d) {
            entrer_etat(LIGNE_CORRIGE_DROITE);
        } else if (ticks_etat > T_PERDU_MAX) {
            flag_echec = 1;                /* abandon : on s'arrete */
            entrer_etat(LIGNE_ARRET);
        }
        break;

    default:
        entrer_etat(LIGNE_ARRET);
        break;
    }
}

/* ------------------------------------------------------------------ */
/* Interface navigation                                                */
/* ------------------------------------------------------------------ */

uint8_t ligne_blanc_double_detecte(void)
{
    return flag_blanc_double;
}

void ligne_acquitter_blanc_double(void)
{
    flag_blanc_double = 0;
    inhibition = T_BLOCAGE;     /* ignore les re-detections pendant qu'on quitte la zone */
}

uint8_t ligne_echec(void)
{
    return flag_echec;
}

/* ------------------------------------------------------------------ */
/* Debug                                                               */
/* ------------------------------------------------------------------ */

void ligne_debug_leds(void)
{
    /* P1.0 (LED rouge Launchpad) = capteur gauche blanc
     * P1.6 (LED verte Launchpad) = capteur droit blanc
     * A utiliser seulement si ces broches ne sont pas prises par ailleurs. */
    if (blanc_g) P1OUT |= BIT0; else P1OUT &= ~BIT0;
    if (blanc_d) P1OUT |= BIT6; else P1OUT &= ~BIT6;
}
