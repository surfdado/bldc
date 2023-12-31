#ifndef BUZZER_H_
#define BUZZER_H_

#include "ch.h" // ChibiOS
#include "hal.h" // ChibiOS HAL

void buzzer_init(void);
void beep_off(void);
void beep_on(void);

#endif /* BUZZER_H_ */
