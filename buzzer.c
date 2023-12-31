#include "buzzer.h"

#include "conf_general.h"
#include "mc_interface.h" // Motor control functions

// Default to using servo pin
#ifdef EXT_BUZZER_ON
#define CUSTOM_BUZZER
#else
#define EXT_BUZZER_ON()			palSetPad(HW_ICU_GPIO, HW_ICU_PIN)
#define EXT_BUZZER_OFF()		palClearPad(HW_ICU_GPIO, HW_ICU_PIN)
#endif

void buzzer_init(void) {
#ifndef CUSTOM_BUZZER
	// External Buzzer (using servo pin!)
	palSetPadMode(HW_ICU_GPIO, HW_ICU_PIN,
			PAL_MODE_OUTPUT_PUSHPULL |
			PAL_STM32_OSPEED_HIGHEST);
	EXT_BUZZER_OFF();
#endif
}

void beep_off()
{
	EXT_BUZZER_OFF();
}

void beep_on()
{
	EXT_BUZZER_ON();
}
