//#############################################################################
// FILE:   HWstarter_main.c
//
// TITLE:  HW Starter

//COMMENTED BY NEIL MAUSHARD USING //NM
//#############################################################################

// Included Files
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <string.h>
#include <math.h>
#include <limits.h>
#include "F28x_Project.h"
#include "driverlib.h"
#include "device.h"
#include "F28379dSerial.h"
#include "LEDPatterns.h"
#include "song.h"
#include "dsp.h"
#include "fpu32/fpu_rfft.h"

#define PI          3.1415926535897932384626433832795
#define TWOPI       6.283185307179586476925286766559
#define HALFPI      1.5707963267948966192313216916398
// The Launchpad's CPU Frequency set to 200 you should not change this value
#define LAUNCHPAD_CPU_FREQUENCY 200


// Interrupt Service Routines predefinition
__interrupt void cpu_timer0_isr(void);
__interrupt void cpu_timer1_isr(void);
__interrupt void cpu_timer2_isr(void);
__interrupt void SWI_isr(void);

// Count variables
uint32_t numTimer0calls = 0;
uint32_t numSWIcalls = 0;
extern uint32_t numRXA;
uint16_t UARTPrint = 0;
uint16_t LEDdisplaynum = 0;

int16_t updown = 1; //NM determines whether to count up(1) or down(1) for brightness cycling of led1
int32_t Adca4Counter = 0; //NM counter for how many times adca_isr is called

int16_t joyXResult; //NM raw measurement of ADCINA2
float joyXVolt; //NM ADCINA2 in volts
float joyXScaled; //NM joyX value between -1 and 1

int16_t joyYResult; //NM raw measurement of ADCINA3
float joyYVolt; //NM ADCINA3 in volts
float joyYScaled; //NM joyY value between -1 and 1

float dutyA = 0.08;
float dutyB = 0.08;
void setEPWM8A_RCServo(float angle);
void setEPWM8B_RCServo(float angle);
float theta1 = 0;
float theta2 = 0;
float angle1 = 0;
float angle2 = 0;

//NM IK vars
float x = -40;
float y = 60;
float kx = 2;
float ky = 2;
float a1 = 75.0;
float a2 = 135.0;
float a3 = 135.0;
float a4 = 75.0;
float a5 = 80.0;
float d23 = 0.0;
float d13 = 0.0;
float alpha1 = 0.0;
float beta1 = 0.0;
float beta5 = 0.0;
float alpha5 = 0.0;

void GPIOconfig(void);

/////////////////////////////////////////////////////////////////////////////////////////////////////////
//adca1 pie interrupt
__interrupt void ADCA_ISR (void)
{
    //NM Joystick X
    joyXResult = AdcaResultRegs.ADCRESULT2;
    // Here covert ADCINA2 to volts
    joyXVolt = joyXResult*3.0/4096.0; // NM 3 volts is equal to a value of 4095, so divide by 4096, then scale by 3V
    joyXScaled = (joyXVolt - 1.5)/1.5; //NM scale to a value between -1 and 1

    //NM Joystick Y
    joyYResult = AdcaResultRegs.ADCRESULT1;
    // Here covert ADCINA3 to volts
    joyYVolt = joyYResult*3.0/4096.0; // NM 3 volts is equal to a value of 4095, so divide by 4096, then scale by 3V
    joyYScaled = (joyYVolt - 1.5)/1.5; //NM scale to a value between -1 and 1

    // Print ADCINA4’s voltage value to TeraTerm every 100ms by setting UARTPrint to one every 100th
    //time in this function.
    Adca4Counter += 1;
    if (Adca4Counter == 100) { //NM every 100th call, print to TeraTerm and reset the counter
        UARTPrint = 1;
        Adca4Counter = 0;
    }
    AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; //clear interrupt flag
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////

void main(void)
{
    // PLL, WatchDog, enable Peripheral Clocks
    // This example function is found in the F2837xD_SysCtrl.c file.
    InitSysCtrl();

    InitGpio();

    //////////////////////////////////////////////////////////////////////////////////////////////////
    //NM initialize epwm8
    EPwm8Regs.TBCTL.bit.CTRMODE = 0; //NM count up mode
    EPwm8Regs.TBCTL.bit.FREE_SOFT = 2; //NM sets free soft emulation mode to free run
    EPwm8Regs.TBCTL.bit.PHSEN = 0; //NM do not load time base counter from time base phase register
    EPwm8Regs.TBCTL.bit.CLKDIV = 4; //NM set clock divide to /16
    EPwm8Regs.TBCTR = 0; //NM start timer at 0
    EPwm8Regs.TBPRD = 62500; //NM sets period to 50Hz since 50MHz/(50Hz*16) = 62500
    EPwm8Regs.CMPA.bit.CMPA = dutyA*EPwm8Regs.TBPRD; //NM duty cycle = CMPA/TBPRD, so CMPA = duty cycle * TBPRD
    EPwm8Regs.AQCTLA.bit.CAU = 1; //NM set pin low when timer counts up to CMPA
    EPwm8Regs.AQCTLA.bit.ZRO = 2; //NM set pin high when TBCTR is zero
    EPwm8Regs.TBPHS.bit.TBPHS = 0; //NM as instructed in document
    EPwm8Regs.CMPB.bit.CMPB = dutyB*EPwm8Regs.TBPRD; //NM two separate duty cycle vars, one for 8A one for 8B
    EPwm8Regs.AQCTLB.bit.CBU = 1; //NM set pin low when timer counts up to CMPB
    EPwm8Regs.AQCTLB.bit.ZRO = 2; //NM set pn high when TBCTR resets back to 0

    GPIO_SetupPinMux(14, GPIO_MUX_CPU1, 1); //NM Set GPIO14 to EPWM8A
    GPIO_SetupPinMux(15, GPIO_MUX_CPU1, 1); //NM Set GPIO15 to EPWM8B

    EALLOW; // Below are protected registers
    GpioCtrlRegs.GPAPUD.bit.GPIO14 = 1; // For EPWM8A
    GpioCtrlRegs.GPAPUD.bit.GPIO15 = 1; // For EPWM8B
    EDIS;

    ///////////////////////////////////////////////////////////////////////////////////////////////////

    GPIOconfig();

    // Clear all interrupts and initialize PIE vector table:
    // Disable CPU interrupts
    DINT;

    // Initialize the PIE control registers to their default state.
    // The default state is all PIE interrupts disabled and flags
    // are cleared.
    // This function is found in the F2837xD_PieCtrl.c file.
    InitPieCtrl();

    // Disable CPU interrupts and clear all CPU interrupt flags:
    IER = 0x0000;
    IFR = 0x0000;

    // Initialize the PIE vector table with pointers to the shell Interrupt
    // Service Routines (ISR).
    // This will populate the entire table, even if the interrupt
    // is not used in this example.  This is useful for debug purposes.
    // The shell ISR routines are found in F2837xD_DefaultIsr.c.
    // This function is found in F2837xD_PieVect.c.
    InitPieVectTable();

    // Interrupts that are used in this example are re-mapped to
    // ISR functions found within this project
    EALLOW;  // This is needed to write to EALLOW protected registers
    PieVectTable.TIMER0_INT = &cpu_timer0_isr;
    PieVectTable.TIMER1_INT = &cpu_timer1_isr;
    PieVectTable.TIMER2_INT = &cpu_timer2_isr;
    PieVectTable.SCIA_RX_INT = &RXAINT_recv_ready;
    PieVectTable.SCIB_RX_INT = &RXBINT_recv_ready;
    PieVectTable.SCIC_RX_INT = &RXCINT_recv_ready;
    PieVectTable.SCID_RX_INT = &RXDINT_recv_ready;
    PieVectTable.SCIA_TX_INT = &TXAINT_data_sent;
    PieVectTable.SCIB_TX_INT = &TXBINT_data_sent;
    PieVectTable.SCIC_TX_INT = &TXCINT_data_sent;
    PieVectTable.SCID_TX_INT = &TXDINT_data_sent;
    PieVectTable.ADCA1_INT = &ADCA_ISR; // NM maps our interrupt to a specific memory location



    PieVectTable.EMIF_ERROR_INT = &SWI_isr;
    EDIS;    // This is needed to disable write to EALLOW protected registers


    // Initialize the CpuTimers Device Peripheral. This function can be
    // found in F2837xD_CpuTimers.c
    InitCpuTimers();

    // Configure CPU-Timer 0, 1, and 2 to interrupt every given period:
    // 200MHz CPU Freq,                       Period (in uSeconds)
    ConfigCpuTimer(&CpuTimer0, LAUNCHPAD_CPU_FREQUENCY, 10000);
    ConfigCpuTimer(&CpuTimer1, LAUNCHPAD_CPU_FREQUENCY, 20000);
    ConfigCpuTimer(&CpuTimer2, LAUNCHPAD_CPU_FREQUENCY, 1000); //NM call timer2 interrupt once per ms

    // Enable CpuTimer Interrupt bit TIE
    CpuTimer0Regs.TCR.all = 0x4000;
    CpuTimer1Regs.TCR.all = 0x4000;
    CpuTimer2Regs.TCR.all = 0x4000;

    init_serialSCIA(&SerialA,115200);
    //    init_serialSCIC(&SerialC,115200);
    //    init_serialSCID(&SerialD,115200);

    ////////////////////////////////////////////////////////////////////
    //NM Exercise 3 code
    EALLOW;
    EPwm4Regs.ETSEL.bit.SOCAEN = 0; // Disable SOC on A group
    EPwm4Regs.TBCTL.bit.CTRMODE = 3; // freeze counter
    EPwm4Regs.ETSEL.bit.SOCASEL = 2; //NM 2 enables event when TBCTR = TBPRD //Select event when counter equal to PRD
    EPwm4Regs.ETPS.bit.SOCAPRD = 1; //NM generates SOCA pulse on first event // Generate pulse on 1st event (\pulse" is the same as \trigger")
    EPwm4Regs.TBCTR = 0x0; // Clear counter
    EPwm4Regs.TBPHS.bit.TBPHS = 0x0000; // Phase is 0
    EPwm4Regs.TBCTL.bit.PHSEN = 0; // Disable phase loading
    EPwm4Regs.TBCTL.bit.CLKDIV = 0; // divide by 1 50Mhz Clock
    EPwm4Regs.TBPRD = 50000;//NM 50MHz/1kHz =  // Set Period to 1ms sample. The input clock is 50MHz.
    // Notice here that we are not setting CMPA or CMPB because we are not using the PWM signal
    EPwm4Regs.ETSEL.bit.SOCAEN = 1; //enable SOCA
    EPwm4Regs.TBCTL.bit.CTRMODE = 0;//NM 0 sets it to count up mode //unfreeze, and enter up count mode
    EDIS;

    EALLOW;
    //write configurations for ADCA
    AdcaRegs.ADCCTL2.bit.PRESCALE = 6; //set ADCCLK divider to /4
    AdcSetMode(ADC_ADCA, ADC_RESOLUTION_12BIT, ADC_SIGNALMODE_SINGLE); //read calibration settings
    //Set pulse positions to late
    AdcaRegs.ADCCTL1.bit.INTPULSEPOS = 1;
    //power up the ADCs
    AdcaRegs.ADCCTL1.bit.ADCPWDNZ = 1;
    //delay for 1ms to allow ADC time to power up
    DELAY_US(1000);
    //Select the channels to convert and the end of conversion flag
    //ADCA
    AdcaRegs.ADCSOC0CTL.bit.CHSEL = 4;//NM selects ADCIN4 for SOC0 //SOC0 will convert Channel you choose Does not have to be A0
    AdcaRegs.ADCSOC0CTL.bit.ACQPS = 99; //sample window is acqps + 1 SYSCLK cycles = 500ns
    AdcaRegs.ADCSOC0CTL.bit.TRIGSEL = 11; //NM selects ePWM4 ADCSOCA as trigger for SOC0 // EPWM4 ADCSOCA

    ///////////////////////////////////
    //Exercise 4
    AdcaRegs.ADCSOC1CTL.bit.CHSEL = 2; //NM sample from ADCINA2 //SOC1 will conv Channel you choose Does not have to be A1
    AdcaRegs.ADCSOC1CTL.bit.ACQPS = 99; //sample window is acqps + 1 SYSCLK cycles = 500ns
    AdcaRegs.ADCSOC1CTL.bit.TRIGSEL = 11; //NM trigger from ePWM4 SOCA for SOC 1// EPWM4 ADCSOCA

    AdcaRegs.ADCSOC2CTL.bit.CHSEL = 3; //NM sample from ADCIN3 //SOC2 will conv Channel you choose Does not have to be A2
    AdcaRegs.ADCSOC2CTL.bit.ACQPS = 99; //sample window is acqps + 1 SYSCLK cycles = 500ns
    AdcaRegs.ADCSOC2CTL.bit.TRIGSEL = 11; //NM selectes ePWEM4 SOCA as trigger for SOC2 // EPWM4 ADCSOCA

    AdcaRegs.ADCINTSEL1N2.bit.INT1SEL = 2; //NM select EOC2 since we want SOC2 to be the final conversion //set to last or only SOC that is converted, and it will set INT1 flag

    AdcaRegs.ADCINTSEL1N2.bit.INT1E = 1; //enable INT1 flag
    AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; //make sure INT1 flag is cleared
    EDIS;
    //////////////////////////////////////////////////////////////////////

    // Enable CPU int1 which is connected to CPU-Timer 0, CPU int13
    // which is connected to CPU-Timer 1, and CPU int 14, which is connected
    // to CPU-Timer 2:  int 12 is for the SWI.  
    IER |= M_INT1;
    IER |= M_INT8;  // SCIC SCID
    IER |= M_INT9;  // SCIA
    IER |= M_INT12;
    IER |= M_INT13;
    IER |= M_INT14;

    // Enable TINT0 in the PIE: Group 1 interrupt 7
    PieCtrlRegs.PIEIER1.bit.INTx7 = 1;
    // Enable SWI in the PIE: Group 12 interrupt 9
    PieCtrlRegs.PIEIER12.bit.INTx9 = 1;

    PieCtrlRegs.PIEIER1.bit.INTx1 = 1; //NM enables our interrupt which is group 1 interrupt 1

    // Enable global Interrupts and higher priority real-time debug events
    EINT;  // Enable Global interrupt INTM
    ERTM;  // Enable Global realtime interrupt DBGM

    // IDLE loop. Just sit and loop forever (optional):
    while(1)
    {
        if (UARTPrint == 1 ) {
                serial_printf(&SerialA,"Joystick X: %.2f\r\n Joystick Y: %.2f\r\n", joyXScaled, joyYScaled);
            UARTPrint = 0;
        }
    }
}

// SWI_isr,  Using this interrupt as a Software started interrupt
__interrupt void SWI_isr(void) {

    // These three lines of code allow SWI_isr, to be interrupted by other interrupt functions
    // making it lower priority than all other Hardware interrupts.
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP12;
    asm("       NOP");                    // Wait one cycle
    EINT;                                 // Clear INTM to enable interrupts

    // Insert SWI ISR Code here.......

    numSWIcalls++;
    DINT;
}

// cpu_timer0_isr - CPU Timer0 ISR
__interrupt void cpu_timer0_isr(void)
{
    CpuTimer0.InterruptCount++;

    numTimer0calls++;
    //NM integrate joystick/////////////////////////////////////////////////////////////
    //NM intgrate joystick X
    if (joyXScaled < -0.1){ //NM if joystick X not at center +- 0.1, go up or down depending on sign
        x = x - kx*joyXScaled*joyXScaled;
    }
    if (joyXScaled > 0.1){
        x = x + kx*joyXScaled*joyXScaled;
    }
    //NM integrate joystick Y
    if (joyYScaled < -0.1){ //NM if joystick Y not at center +- 0.1, go up or down depending on sign
        y = y - ky*joyYScaled*joyYScaled;
    }
    if (joyYScaled > 0.1){
        y = y + ky*joyYScaled*joyYScaled;
    }
    //NM constrain
    if (x > 102.0){x = 102.0;}
    if (x < -182.0) {x = -182.0;}
    if (y < 60){y = 60;}
    if (y > sqrt((40.0 + 114.165)*(40.0 + 114.165) - (x + 40.0)*(x + 40.0))){
        y = sqrt((40.0 + 114.165)*(40.0 + 114.165) - (x + 40.0)*(x + 40.0));
    }
    //IK///////////////////////////////////////////////////////////
    d23 = sqrt(x*x + y*y); //NM mm dist from point 2 to 3
    d13 = sqrt((x + a5)*(x + a5) + y*y);//NM mm dist from point 1 to 3
    alpha1 = acos((a1*a1 - a2*a2 + d23*d23)/(2*a1*d23)); //NM rads intermediate angle
    beta1 = atan2(y,-x); //NM rads intermediate angle
    beta5 = acos((a4*a4 - a3*a3 + d13*d13)/(2*a4*d13)); //NM rads intermediate angle
    alpha5 = atan2(y,x + a5); //NM rads intermediate angle

    //NM th1 = 180 when angle1 = 34.5
    //NM th2 = 0 when angle2 = -14.5

    theta1 = alpha5 + beta5;// NM rads
    theta2 = PI - alpha1 - beta1;// NM rads

    angle1 = theta1*180.0/PI + 34.5 - 180; //NM deg with offset
    angle2 = theta2*180.0/PI - 14.5;//NM deg with offset
    ///////////////////////////////////////////////////////////////

    setEPWM8A_RCServo(angle1); //NM set servos to respective angles
    setEPWM8B_RCServo(angle2);

    // Blink LaunchPad Red LED
    GpioDataRegs.GPBTOGGLE.bit.GPIO34 = 1;

    // Acknowledge this interrupt to receive more interrupts from group 1
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}

// cpu_timer1_isr - CPU Timer1 ISR
__interrupt void cpu_timer1_isr(void)
{


    CpuTimer1.InterruptCount++;
}

// cpu_timer2_isr CPU Timer2 ISR
__interrupt void cpu_timer2_isr(void)
{


    // Blink LaunchPad Blue LED
    GpioDataRegs.GPATOGGLE.bit.GPIO31 = 1;

    CpuTimer2.InterruptCount++;

//  if ((CpuTimer2.InterruptCount % 50) == 0) {
//      UARTPrint = 1;
//  }
}
void setEPWM8A_RCServo(float angle){
    if (angle > 90.0){angle = 90.0;} //NM saturate
    if (angle < -90.0){angle = -90.0;}
    dutyA = -0.04/90.0*angle + 0.08; //NM set duty cycle to be between 0.04 and 0.12
    EPwm8Regs.CMPA.bit.CMPA = dutyA*EPwm8Regs.TBPRD; //NM control motor via duty cycle
}

void setEPWM8B_RCServo(float angle){
    if (angle > 90.0){angle = 90.0;} //NM saturate
    if (angle < -90.0){angle = -90.0;}
    dutyB = -0.04/90.0*angle + 0.08; // NM same as above function
    EPwm8Regs.CMPB.bit.CMPB = dutyB*EPwm8Regs.TBPRD;
}

void GPIOconfig(void){
    // Blue LED on LaunchPad
    GPIO_SetupPinMux(31, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(31, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPASET.bit.GPIO31 = 1;

    // Red LED on LaunchPad
    GPIO_SetupPinMux(34, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(34, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPBSET.bit.GPIO34 = 1;

    // LED1 and PWM Pin
    //GPIO_SetupPinMux(22, GPIO_MUX_CPU1, 0);
    //GPIO_SetupPinOptions(22, GPIO_OUTPUT, GPIO_PUSHPULL);
    //GpioDataRegs.GPACLEAR.bit.GPIO22 = 1;

    // LED2
    GPIO_SetupPinMux(94, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(94, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPCCLEAR.bit.GPIO94 = 1;

    // LED3
    GPIO_SetupPinMux(95, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(95, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPCCLEAR.bit.GPIO95 = 1;

    // LED4
    GPIO_SetupPinMux(97, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(97, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPDCLEAR.bit.GPIO97 = 1;

    // LED5
    GPIO_SetupPinMux(111, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(111, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPDCLEAR.bit.GPIO111 = 1;

    // LED6
    GPIO_SetupPinMux(130, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(130, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPECLEAR.bit.GPIO130 = 1;

    // LED7
    GPIO_SetupPinMux(131, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(131, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPECLEAR.bit.GPIO131 = 1;

    // LED8
    GPIO_SetupPinMux(25, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(25, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPACLEAR.bit.GPIO25 = 1;

    // LED9
    GPIO_SetupPinMux(26, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(26, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPACLEAR.bit.GPIO26 = 1;

    // LED10
    GPIO_SetupPinMux(27, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(27, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPACLEAR.bit.GPIO27 = 1;

    // LED11
    GPIO_SetupPinMux(60, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(60, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPBCLEAR.bit.GPIO60 = 1;

    // LED12
    GPIO_SetupPinMux(61, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(61, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPBCLEAR.bit.GPIO61 = 1;

    // LED13
    GPIO_SetupPinMux(157, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(157, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPECLEAR.bit.GPIO157 = 1;

    // LED14
    GPIO_SetupPinMux(158, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(158, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPECLEAR.bit.GPIO158 = 1;

    // LED15
    GPIO_SetupPinMux(159, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(159, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPECLEAR.bit.GPIO159 = 1;

    // LED16
    GPIO_SetupPinMux(160, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(160, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPFCLEAR.bit.GPIO160 = 1;

    //WIZNET Reset
    GPIO_SetupPinMux(0, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(0, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPASET.bit.GPIO0 = 1;

    //ESP8266 Reset
    GPIO_SetupPinMux(1, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(1, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPASET.bit.GPIO1 = 1;

    //SPIRAM  CS  Chip Select
    GPIO_SetupPinMux(19, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(19, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPASET.bit.GPIO19 = 1;

    //DRV8874 #1 DIR  Direction
    GPIO_SetupPinMux(29, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(29, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPASET.bit.GPIO29 = 1;

    //DRV8874 #2 DIR  Direction
    GPIO_SetupPinMux(32, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(32, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPBSET.bit.GPIO32 = 1;

    //DAN28027  CS  Chip Select
    GPIO_SetupPinMux(9, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(9, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPASET.bit.GPIO9 = 1;

    //MPU9250  CS  Chip Select
    GPIO_SetupPinMux(66, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(66, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;

    //WIZNET  CS  Chip Select
    GPIO_SetupPinMux(125, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(125, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPDSET.bit.GPIO125 = 1;

    //PushButton 1
    GPIO_SetupPinMux(4, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(4, GPIO_INPUT, GPIO_PULLUP);

    //PushButton 2
    GPIO_SetupPinMux(5, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(5, GPIO_INPUT, GPIO_PULLUP);

    //PushButton 3
    GPIO_SetupPinMux(6, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(6, GPIO_INPUT, GPIO_PULLUP);

    //PushButton 4
    GPIO_SetupPinMux(7, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(7, GPIO_INPUT, GPIO_PULLUP);

    //Joy Stick Pushbutton
    GPIO_SetupPinMux(8, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(8, GPIO_INPUT, GPIO_PULLUP);
}
