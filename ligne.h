/*
 * ligne.h - Suivi de ligne par machine d'etat (membre 2)
 * Cible : MSP430G2553 (Launchpad sur SAMboard v3)
 *
 * Utilisation :
 *   ligne_init();
 *   ligne_calibrer_noir();  ... puis ligne_calibrer_blanc(); ligne_calculer_seuils();
 *   ligne_start();
 *   // puis ligne_update() appelee regulierement (tick de LIGNE_PERIODE_MS)
 */
#ifndef LIGNE_H
#define LIGNE_H

#include <stdint.h>

/* ------------------------------------------------------------------ */
/* Parametres a valider avec l'equipe / a regler sur le terrain        */
/* ------------------------------------------------------------------ */

/* Periode d'appel de ligne_update() (fournie par le tick du membre 4) */
#define LIGNE_PERIODE_MS        5

/* Canaux ADC10 des capteurs (P1.4 = A4, P1.5 = A5) - a valider */
#define LIGNE_CANAL_GAUCHE      4
#define LIGNE_CANAL_DROIT       5

/* 1 : montage collecteur + pull-up -> blanc = tension BASSE (cas probable)
 * 0 : blanc = tension haute                                            */
#define LIGNE_BLANC_TENSION_BASSE   1

/* Nombre de mesures ADC moyennees a chaque lecture */
#define LIGNE_NB_ECHANTILLONS   4

/* Anti-rebond : nb de lectures identiques consecutives pour valider */
#define LIGNE_NB_CONFIRMATION   2

/* Seuils par defaut (avant calibration), echelle 0..1023, 1023 = tres blanc */
#define LIGNE_SEUIL_HAUT_DEFAUT 600
#define LIGNE_SEUIL_BAS_DEFAUT  400

/* Vitesses (pourcentage -100..+100) */
#define LIGNE_V_NOMINALE        60
#define LIGNE_V_ROUE_LENTE      20   /* roue interieure pendant une correction */
#define LIGNE_V_PIVOT           40   /* pivot en recherche de ligne */

/* Temporisations (ms) */
#define LIGNE_T_CORRECTION_MAX_MS   1000 /* correction trop longue -> PERDU    */
#define LIGNE_T_PERDU_MAX_MS        1500 /* recherche trop longue -> ARRET     */
#define LIGNE_T_SANS_BLANC_MAX_MS      0 /* 0 = desactive (voir note ligne.c)  */
#define LIGNE_T_BLOCAGE_MS           500 /* temps mort apres acquittement
                                            d'un blanc double                  */

/* ------------------------------------------------------------------ */
/* Types                                                               */
/* ------------------------------------------------------------------ */

typedef enum {
    LIGNE_ARRET,
    LIGNE_AVANCE,
    LIGNE_CORRIGE_GAUCHE,
    LIGNE_CORRIGE_DROITE,
    LIGNE_BLANC_DOUBLE,
    LIGNE_PERDU
} ligne_etat_t;

/* ------------------------------------------------------------------ */
/* Initialisation et acquisition                                       */
/* ------------------------------------------------------------------ */

void     ligne_init(void);                 /* broches + ADC10 + etat ARRET   */
uint16_t ligne_lire_gauche(void);          /* 0..1023, 1023 = blanc          */
uint16_t ligne_lire_droite(void);          /* 0..1023, 1023 = blanc          */

/* ------------------------------------------------------------------ */
/* Calibration (robot immobile, capteurs poses sur la surface voulue)  */
/* ------------------------------------------------------------------ */

void ligne_calibrer_noir(void);            /* capteurs sur le fond noir      */
void ligne_calibrer_blanc(void);           /* capteurs sur le ruban blanc    */
void ligne_calculer_seuils(void);          /* calcule seuil bas / haut       */

/* ------------------------------------------------------------------ */
/* Machine d'etat                                                      */
/* ------------------------------------------------------------------ */

void         ligne_start(void);            /* -> AVANCE                      */
void         ligne_stop(void);             /* -> ARRET, moteurs coupes       */
void         ligne_update(void);           /* 1 pas, NON bloquant            */
ligne_etat_t ligne_get_etat(void);

/* ------------------------------------------------------------------ */
/* Interface pour navigation (membre 5)                                */
/* ------------------------------------------------------------------ */

uint8_t ligne_blanc_double_detecte(void);  /* 1 si croisement/cercle vu      */
void    ligne_acquitter_blanc_double(void);/* efface le flag + temps mort    */
uint8_t ligne_echec(void);                 /* 1 si ligne definitivement perdue */

/* ------------------------------------------------------------------ */
/* Debug                                                               */
/* ------------------------------------------------------------------ */

void ligne_debug_leds(void);               /* LED P1.0 = gauche, P1.6 = droit */

#endif /* LIGNE_H */
