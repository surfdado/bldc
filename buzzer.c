#include "buzzer.h"

#ifdef HAS_EXT_BUZZER

#ifndef EXT_BUZZER_ON
#error Missing definition of EXT_BUZZER_ON despite HAS_EXT_BUZZER being defined
#endif
#ifndef EXT_BUZZER_OFF
#error Missing definition of EXT_BUZZER_OFF despite HAS_EXT_BUZZER being defined
#endif

// TODO: Make this configurable from the app
#define ALERT_MIN_BEEP_MS 1200

#define BEEP_SHORT 0
#define BEEP_LONG 1

static int alert_beep_num_left = 0;
static systime_t alert_beep_time;
static unsigned int alert_beep_duration = BEEP_SHORT;

void update_beep_alert(void)
{
	if (alert_beep_num_left > 0) {
		if (chVTGetSystemTimeX() - alert_beep_time > alert_beep_duration) {
			alert_beep_time = chVTGetSystemTimeX();
			alert_beep_num_left--;

			if (alert_beep_num_left & 0x1)
				EXT_BUZZER_ON();
			else
				EXT_BUZZER_OFF();
		}
	}
}

void beep_alert(int num_beeps, bool longbeep)
{
	if (alert_beep_num_left == 0) {
		alert_beep_num_left = num_beeps * 2;
		alert_beep_time = chVTGetSystemTimeX();
		alert_beep_duration = longbeep ? 4 * ALERT_MIN_BEEP_MS : ALERT_MIN_BEEP_MS;
		EXT_BUZZER_ON();
	}
}

void beep_off(bool force)
{
	// don't mess with the buzzer if we're in the process of doing a multi-beep
	if (force || (alert_beep_num_left == 0))
		EXT_BUZZER_OFF();
}

void beep_on(bool force)
{
	// don't mess with the buzzer if we're in the process of doing a multi-beep
	if (force || (alert_beep_num_left == 0))
		EXT_BUZZER_ON();
}

#else
#define update_beep_alert(void) {}
#define beep_alert(int, bool) {}
#endif
