/*
 * main.c CPU-01
 */
#include "fc_cpu1.hpp"

#include "sram/DualPortedRAM.h"
#include "../library/ControlLib/attitude.h"
#include "../library/logger/log_read.h"
#include "../library/motors/motor.h"
#include "../library/pid/pid.h"

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
    device_init();

   	for(;;) {
        if(epwm_400)
            four_hundred_hz_task();

        if(epwm_100)
            one_hundred_hz_task();

        if(epwm_50)
            fifty_hz_task();

        if(epwm_25)
            twentyfive_hz_task();

        if(epwm_10)
            ten_hz_task();

        if(epwm_2)
            two_hz_task();

        if(epwm_1)
            one_hz_task();

		// Background services.
		sysmonitor();
   		// protection delay.
   		DELAY_US(10);
   	}
}

void device_init()
{
    // Initial system clock.
    InitSysCtrl();

    // Give control of partial GS memory to cpu2, matched with cmd file.
    EALLOW;
        MemCfgRegs.GSxMSEL.bit.MSEL_GS10 = 1;
        MemCfgRegs.GSxMSEL.bit.MSEL_GS11 = 1;
        MemCfgRegs.GSxMSEL.bit.MSEL_GS12 = 1;
        MemCfgRegs.GSxMSEL.bit.MSEL_GS13 = 1;
        MemCfgRegs.GSxMSEL.bit.MSEL_GS15 = 1;
    EDIS;

    // Give control of partial prepheral to cpu2.
    EALLOW;
        DevCfgRegs.CPUSEL0.bit.EPWM7 = 1;
        DevCfgRegs.CPUSEL0.bit.EPWM8 = 1;
        DevCfgRegs.CPUSEL5.bit.SCI_A = 1;
        DevCfgRegs.CPUSEL5.bit.SCI_D = 1;
        DevCfgRegs.CPUSEL6.bit.SPI_B = 1;
        DevCfgRegs.CPUSEL7.bit.I2C_A = 1;
        DevCfgRegs.CPUSEL11.bit.ADC_B = 1;
        DevCfgRegs.CPUSEL8.bit.CAN_A = 1;
    EDIS;

//    IPCBootCPU2(C1C2_BROM_BOOTMODE_BOOT_FROM_FLASH);

    // GPIO initialization. Only call it in CPU1.
    InitGpio();
    util_led_init_gpio();
    util_i2ca_init_gpio(); // Baro on cpu2
    util_scia_init_gpio(); // Receive RTK data on cpu2
    util_scib_init_gpio(); // SBus
    util_scic_init_gpio(); // Send RTCM data to RTK
    util_epwm_init_gpio();
    util_spib_init_gpio(); // ADIS on cpu2
    util_spic_init_gpio(); // eeprom/flash/gd

    // Disable CPU interrupts
    DINT;
    // Initialize the PIE control registers to their default state.
    InitPieCtrl();

    // Disable CPU interrupts and clear all CPU interrupt flags.
    IER = 0x0000;
    IFR = 0x0000;
    // Initialize the PIE vector table with pointers to the shell Interrupt.
    InitPieVectTable();

    // Regist isr callback functions.
    EALLOW;
        PieVectTable.TIMER0_INT = &clock_tick_isr;      // CPU timer0 interrupt
        PieVectTable.SPIC_RX_INT = &spic_rx_fifo_isr;   // SPIC rx interrupt
        PieVectTable.SPIC_TX_INT = &spic_tx_fifo_isr;   // SPIC tx interrupt
        PieVectTable.SCIC_RX_INT = &scic_rx_fifo_isr;   // SCIC rx interrupt
        PieVectTable.SCIC_TX_INT = &scic_tx_fifo_isr;   // SCIC tx interrupt
        PieVectTable.SCIB_RX_INT = &sbus_rx_fifo_isr;   // SCIB rx interrupt
        PieVectTable.EPWM1_INT = &epwm_1_isr;           // EPWM 1 interrupt
        PieVectTable.IPC0_INT = &CPU02toCPU01IPC0IntHandler;
        PieVectTable.IPC1_INT = &CPU02toCPU01IPC1IntHandler;
    EDIS;

    // Init modules required by setup procedure.
    clock_init();
    clock_tick_start();
    spic_init();
    ipc_init();
    emif1_init();

    // Enable all interrupts.
    IER |= M_INT1;                      // Enable group 1 interrupts (timer0 & ipc0 ipc1 ipc2 & xint1)
    IER |= M_INT3;                      // Enable group 3 interrupts (epwm1)
    IER |= M_INT6;                      // Enable group 6 interrupts (spic & spia)
    IER |= M_INT8;                      // Enable group 8 interrupts (scic & scid)
    IER |= M_INT9;                      // Enable group 9 interrupts (scia & scib)

    // Enable PIE interrupts.
    PieCtrlRegs.PIEIER1.bit.INTx7 = 1;      // CPU timer0 ISR
    PieCtrlRegs.PIEIER1.bit.INTx13 = 1;     // CPU1toCPU2 IPC INT0
    PieCtrlRegs.PIEIER1.bit.INTx14 = 1;     // CPU2toCPU1 IPC INT1
    PieCtrlRegs.PIEIER1.bit.INTx15 = 1;     // CPU2toCPU1 IPC INT2
    PieCtrlRegs.PIEIER3.bit.INTx1 = 1;      // EPWM 1
    PieCtrlRegs.PIEIER6.bit.INTx1 = 1;      // Enable PIE Group 6, INT 1[SPIA Receive Interrupt]
    PieCtrlRegs.PIEIER6.bit.INTx2 = 1;      // Enable PIE Group 6, INT 2[SPIA Transmit Interrupt]
    PieCtrlRegs.PIEIER6.bit.INTx9 = 1;      // SPIC rx
    PieCtrlRegs.PIEIER6.bit.INTx10 = 1;     // SPIC tx
    PieCtrlRegs.PIEIER8.bit.INTx5 = 1;      // SCIC rx
    PieCtrlRegs.PIEIER8.bit.INTx6 = 1;      // SCIC tx
    PieCtrlRegs.PIEIER9.bit.INTx3 = 1;      // SCIB rx

    // Enable global interrupts and higher priority real-time debug events.
    EnableInterrupts();

    //
    ipc_cpu1_wait_cpu2();

    //* Setup
    PosControl_init();

    // Read eeprom load configuration.
    config_restore();

    // Read eeprom load parameters and initialize.
    parameters_restore();
    pid_init(&pid_pitch,MAIN_LOOP_SEC,&parameters.pid_pitch);
    pid_init(&pid_roll,MAIN_LOOP_SEC,&parameters.pid_roll);
    pid_init(&pid_yaw,MAIN_LOOP_SEC,&parameters.pid_yaw);
    pid_init(&_pid_accel_z,MAIN_LOOP_SEC,&parameters.pid_accel_z);
    pi2D_init(&_pi_vel_xy,WPNAV_LOITER_UPDATE_TIME,&parameters.pi_vel_xy);

    motor_init();
    motor_matrix_init();
    attitude_init();
    ahrs_data_init();

    scic_init(CPU_FREQ,115200);
    sbus_init(CPU_FREQ,100000);
    epwm_init(CPU_FREQ/2,IPC_FREQ);//See InitSysPll
    scrap_init();
    IPCLtoRDataTransfer(&g_sIpcController1, pulMsgRam[6], pusCPU01BufferPt,
                        &config, sizeof(Configuration), ENABLE_BLOCKING);
}
