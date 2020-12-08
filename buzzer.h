#ifndef BUZZER_H_
#define BUZZER_H_

#include "conf_general.h"
#include "ch.h" // ChibiOS
#include "hal.h" // ChibiOS HAL
#include "mc_interface.h" // Motor control functions
#include "hw.h" // Pin mapping on this hardware

#ifdef HAS_EXT_BUZZER
void update_beep_alert(void);
void beep_alert(int num_beeps, bool longbeep);
void beep_off(bool force);
void beep_on(bool force);
#else
#define update_beep_alert(void) {}
#define beep_alert(int, bool) {}
#endif

#endif /* BUZZER_H_ */
