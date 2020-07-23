#ifndef Y_MOTION_TASK_H
#define Y_MOTION_TASK_H

#include "main.h"
#include "remote_control.h"


#define AMMO17_OUT 1200
#define AMMO17_OFF 1900
#define AMMO42_OUT 1800
#define AMMO42_OFF 2800

extern void AMMO_42mm_OUT_task(void);
extern void AMMO_OUT_PWM_configuration(void);
extern void AMMO17_out(void);
extern void AMMO17_off(void);
extern void AMMO42_out(void);
extern void AMMO42_off(void);

#endif
