
#ifndef OBSTACLE_H
#define OBSTACLE_H

#include <stdint.h>

/* Initialisation du capteur */
void obstacle_init(void);

/* Lecture et mise a jour de la detection */
void obstacle_update(void);

/* Renvoie 1 si un obstacle est detecte, sinon 0 */
uint8_t obstacle_detecte(void);

/* Modification du seuil de detection */
void obstacle_regler_seuil(uint16_t seuil);

#endif
