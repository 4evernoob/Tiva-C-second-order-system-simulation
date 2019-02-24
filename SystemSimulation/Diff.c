#include <stdint.h>
#include "inc/tm4c123gh6pm.h"
#include <stdbool.h>
#include "inc/hw_types.h"
#include "inc/hw_memmap.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/pwm.h"
#include "driverlib/fpu.h"
#include "inc/hw_pwm.h"
#include "driverlib/pin_map.h"
#include "driverlib/interrupt.h"
#include "driverlib/timer.h"
#include "driverlib/pwm.h"
#include "driverlib/adc.h"
#include  "math.h"
#include "inc/hw_ssi.h"
#include "driverlib/ssi.h"


//*****************************************************************************
// If anything fails this got your back
//*****************************************************************************
#ifdef DEBUG
void
__error__(char *pcFilename, uint32_t ui32Line)
{
}
#endif
#define	NDERIVS	2		//Number of derivatives

void CalcDerivs(int numDerivs, float *x, float t, float *k1, float u1);

void CalcDerivs(int numDerivs, float *x, float t, float *k1, float u1) {
	//system parameters
	float m=24;
	float l = .094;
	float i = 1.266;
	float b = 2.288; //2.288
	float g = 9.81;
    //Test system
	k1[0] = x[1];
	k1[1] = -1 * x[1] - 8 * x[0] + u1;
	//Pendulum system
	//	k1[1]= (1/(m*l*l+i))*(-b*x[1]-m*g*l*sin(x[0])+u1);

}
volatile unsigned int cont = 0;
volatile unsigned long outport = 0;
volatile uint32_t spidat = 0;
volatile uint32_t valor[1];
volatile uint32_t va[16];
volatile unsigned long periodo = 0;
volatile float yexact, ystar, yp;
volatile float yd[5000];	//Last 5 seconds of data
volatile float x[NDERIVS], k1[NDERIVS];		//State space and derivatives
volatile float h = 0.001, t = 0.0, tmax = 3;//step size ,initial time y maximum time
volatile float u = 0;
volatile int i;
volatile int c = 0;

int main(void) {

	//Clock 400/2.5*2= 80 MHz
	SysCtlClockSet(
			SYSCTL_SYSDIV_2_5 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN
					| SYSCTL_XTAL_16MHZ);  //80MHz
	//Floating point unit
	FPUEnable();
	FPULazyStackingEnable();

	//Peripheral configuration
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
	GPIOPinTypeGPIOOutput(GPIO_PORTD_BASE, GPIO_PIN_3);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);

	//Timer configuration
	TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC);
	periodo = (SysCtlClockGet() / 1000) / 2;
	TimerLoadSet(TIMER0_BASE, TIMER_A, periodo - 1);
	IntEnable(INT_TIMER0A);
	TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
	IntMasterEnable();

	// ADC COnfiguration

	SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
	GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_3);
	ADCSequenceConfigure(ADC0_BASE, 3, ADC_TRIGGER_PROCESSOR, 0);
	ADCSequenceStepConfigure(ADC0_BASE, 3, 0,
			ADC_CTL_CH0 | ADC_CTL_IE | ADC_CTL_END);
	ADCSequenceEnable(ADC0_BASE, 3);
	ADCIntClear(ADC0_BASE, 3);
	// SPI COnfiguration
	SysCtlPeripheralEnable(SYSCTL_PERIPH_SSI0);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
	GPIOPinConfigure(GPIO_PA2_SSI0CLK);
	GPIOPinConfigure(GPIO_PA3_SSI0FSS);
	GPIOPinConfigure(GPIO_PA4_SSI0RX);
	GPIOPinConfigure(GPIO_PA5_SSI0TX);
	GPIOPinTypeSSI(GPIO_PORTA_BASE, GPIO_PIN_5 | GPIO_PIN_4 | GPIO_PIN_2);
	SSIConfigSetExpClk(SSI0_BASE, SysCtlClockGet() / 10, SSI_FRF_MOTO_MODE_0,
			SSI_MODE_MASTER, SysCtlClockGet() / 20, 16);

	//Timer and SPI Habilitation

	TimerEnable(TIMER0_BASE, TIMER_A);

	SSIEnable(SSI0_BASE);
	x[0] = 0;							//Initial conditions
	x[1] = 0;
	while (1) {//endless loop

	}
}

//Interruptions

void Timer0IntHandler(void) {
	// Clear the timer interrupt
	TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
	IntMasterDisable();
	ADCProcessorTrigger(ADC0_BASE, 3);
	while (!ADCIntStatus(ADC0_BASE, 3, false)) {
	}

	ADCIntClear(ADC0_BASE, 3);
	ADCSequenceDataGet(ADC0_BASE, 3, valor);
//Input value and value limiter
	u = valor[0] * 0.00805860801 / 2;
	if (c == 5000)
		c = 0;
	//CUrrent system value
	CalcDerivs(NDERIVS, x, t, k1, u);
	ystar = x[0];					//Get our aproximated output.
	yp = x[0];
	if ((c < 5000))
		yd[c] = x[0];

	for (i = 0; i < NDERIVS; i++) {	//variable actualization
		x[i] = x[i] + h * k1[i];
	}
	c++;
	t = t + h;
	cont = round(yp * 1240.90);
//Sent data to SPI DAC
	spidat = (uint16_t) cont;

	GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_3, 0);
	SSIDataPut(SSI0_BASE, spidat | 0x3000);
	while (SSIBusy(SSI0_BASE)) {
	}

	GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_3, GPIO_PIN_3);
	IntMasterEnable();

}
