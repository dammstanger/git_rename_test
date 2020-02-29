/*
 * main.c CPU-02
 */

#include "fc_cpu1.h"

#include "sram/DualPortedRAM.h"
#include "../library/ControlLib/attitude.h"
#include "../library/logger/log_read.h"
#include "../library/motors/motor.h"
#include "../library/pid/pid.h"

#include "clock.h"
#include "config.h"
#include "epwm.h"
#include "giganandflash.h"
#include "ipc.h"
#include "sbus.h"
#include "scia.h"
#include "scic.h"
#include "spic.h"
#include "sysmonitor.h"
#include "util.h"
#include "copter/control_stable.h"
#include "copter/task/task.h"
#include "ahrs.h"

static void device_init();

int main(void) {

}

void device_init()
{

}
