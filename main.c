/**********************************************************************************************
 * Tiva C. TM4C123GXL
 * Posición con Encoder absoluto de 8 bits
 * by <hdezgwilliam@gmail.com> and <fer_peralta10@hotmail.com>
 * William Egardo Hernández Guzmán * Fernando Peralta Sánchez
 **********************************************************************************************/
#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include "inc/tm4c123gh6pm.h"
#include "stdlib.h"
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_uart.h"
#include "inc/hw_gpio.h"
#include "inc/hw_timer.h"
#include "inc/hw_types.h"

#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"

#include "driverlib/debug.h"

#include "driverlib/systick.h"
#include "driverlib/interrupt.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "driverlib/udma.h"
#include "driverlib/gpio.h"
#include "driverlib/timer.h"
#include "driverlib/adc.h"
#include "driverlib/fpu.h"

#include <string.h>

uint32_t B=0, muestra, ui32Period;
#define        FS        500 	// Número de muestras por segundo

volatile uint32_t ServoCount = 0;

volatile double Angulo=0;
volatile int pin_state[8]={0,0,0,0,0,0,0,0};
volatile uint32_t input_pin[8];
volatile uint32_t base_pin[8];
volatile uint32_t EncoderNum = 0;
volatile uint32_t pin = 0;

volatile int SetPoint=0;
double yn1=0, yn2=0;
uint32_t ServoBase[6];
uint32_t ServoPin[6];
uint32_t ServoPos[6];
uint32_t ServoPosTemp[6];
uint32_t ServoNumber = 0;


void Interrupt(){

	//Remember to clear the interrupt flags
  uint32_t status=0;

  status = TimerIntStatus(TIMER5_BASE,true);
  TimerIntClear(TIMER5_BASE,status);

  /*
   * 	This increments the counter
   *	The counter works like a timer in PWM mode in count up mode.
   */

}
/**************************************************************************
 * Función que inicia el ADC con las siguientes características:
 * - El Timer0 dispara el ADC basado en la frecuencia de muestreo
 * - El ADC provoca una interrupción cada vez que ha leido una muestra
 * - El canal de entrada del ADC es el canal 0
 * - El ADC usas el secuenciador 3, una sola muestra por lectura
 **************************************************************************/
void init_ADC()
{
    /***********************************************
     * Configuración de terminales de entrada/salida
     ***********************************************/
    // Habilitación del GPIOE. Entrada analógica en  AIN0 (PE3)
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
    GPIOPinTypeGPIOInput(GPIO_PORTE_BASE, GPIO_PIN_3);
    GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_3);
    // Habilitación de GPIOF y las terminales conectadas al Led RGB
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3);
    /******************************************************
     * Configuración del temporizador para disparar el ADC
     ******************************************************/
     // Timer 0 en funcionamiento periodico
     SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
     TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC);
     // Cálculo del periodo de muestreo en ciclos de reloj
     ui32Period = SysCtlClockGet()/FS;
     TimerLoadSet(TIMER0_BASE, TIMER_A, ui32Period - 1);

     //TimerADCEventSet(TIMER0_BASE,    TIMER_ADC_TIMEOUT_A);

    /*******************************************************
     * Configuración del ADC
     *******************************************************/
     // Habilitación del ADC
     SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);
     // Entrada analógica en AIN0 (PE3)
     GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_3);
     // Uso del secuenciador 3 (FIFO depth 1) para tomar solo una muestra por disparo
     ADCSequenceDisable(ADC0_BASE, 3); // Deshabilitado para programarlo
     // Habilitación del timer que disparará el ADC
     TimerControlTrigger(TIMER0_BASE, TIMER_A, 1);
     // Configuración del secuenciador
     ADCSequenceConfigure(ADC0_BASE, 3, ADC_TRIGGER_TIMER, 0);
     //
     ADCSequenceStepConfigure(ADC0_BASE, 3, 0, ADC_CTL_CH0 | ADC_CTL_IE | ADC_CTL_END);
     ADCSequenceEnable(ADC0_BASE, 3);
     ADCIntEnable(ADC0_BASE, 3);
     IntEnable(INT_ADC0SS3); //habilitamos la interrupcion del secuenciador 3
     TimerEnable(TIMER0_BASE, TIMER_A);
}

int32_t Encoder_Pin(uint32_t peripheral, uint32_t base, uint32_t pin){

	if(EncoderNum < 8){
		//Enable the GPIO you want to use
		SysCtlPeripheralEnable(peripheral);
		SysCtlDelay(3);
		//Set pin as output
		GPIOPinTypeGPIOInput(base,pin);
		//Save which GPIO and pin you want to use
		base_pin[EncoderNum] = base;
		input_pin[EncoderNum] = pin;
		//Resistores de PULL-UP
		GPIOPadConfigSet(base_pin[EncoderNum], input_pin[EncoderNum],
				GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);
		//Increment variable.
		EncoderNum++;
	}
	else
		return -1;

	return 0;
}
/*
  Timer setup
*/
void TimerBegin(){


  /*
   * 200Khz - 5uS. (um servo precisa de um pulse de  1000uS a  2000uS ou 1mS a 2mS)
   * 200-400
   */
  //We set the load value so the timer interrupts each 1ms
  uint32_t Period;
  Period = 40000;

  SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER5);
  SysCtlDelay(3);
  /*
    Configure the timer as periodic, by omission it's in count down mode.
    It counts from the load value to 0 and then resets back to the load value.
  REMEMBER: You need to configure the timer before setting the load and match
  */
  TimerConfigure(TIMER5_BASE, TIMER_CFG_PERIODIC);
  TimerLoadSet(TIMER5_BASE, TIMER_A, Period -1);

  TimerIntRegister(TIMER5_BASE, TIMER_A, Interrupt);

  /*
    Enable the timeout interrupt. In count down mode it's when the timer reaches
  0 and resets back to load. In count up mode it's when the timer reaches load
  and resets back to 0.
  */
  TimerIntEnable(TIMER5_BASE, TIMER_TIMA_TIMEOUT);

  TimerEnable(TIMER5_BASE, TIMER_A);
}

void Read_input_encoder(){
  int dec_position = 0;
  int i;
  for(i = 0; i < 8; i++ ){
    pin_state[i] = !GPIOPinRead(base_pin[i], input_pin[i]); //Bitwise for active LOW
    //GPIOPinWrite(ServoBase[i],ServoPin[i], ServoPin[i]);
  }

  // Gray Code Decoder
  for(i = 7; i >= 0; i--){
    dec_position = (dec_position << 1) | (pin_state[i] ^ (dec_position&0x1));
  } dec_position = dec_position*1.4118;
  Angulo = 224 - dec_position;
  dec_position = 0;
}

int main(void) {
	FPULazyStackingEnable();
    FPUEnable();              // Se activa la Unidad de Punto Flotante
    // Reloj principal a 80 MHz para la Tiva C Launchpad 123G
	SysCtlClockSet(SYSCTL_SYSDIV_2_5|SYSCTL_USE_PLL|SYSCTL_OSC_MAIN|SYSCTL_XTAL_16MHZ);

	/************************************
	* Habilitación pines de entrada Encoder
	*************************************/
	// Habilitación de GPIOB
	Encoder_Pin(SYSCTL_PERIPH_GPIOB, GPIO_PORTB_BASE, GPIO_PIN_3 );
	// Habilitación de GPIOC
	Encoder_Pin(SYSCTL_PERIPH_GPIOC, GPIO_PORTC_BASE, GPIO_PIN_4 );
	Encoder_Pin(SYSCTL_PERIPH_GPIOC, GPIO_PORTC_BASE, GPIO_PIN_5 );
	Encoder_Pin(SYSCTL_PERIPH_GPIOC, GPIO_PORTC_BASE, GPIO_PIN_6 );
	Encoder_Pin(SYSCTL_PERIPH_GPIOC, GPIO_PORTC_BASE, GPIO_PIN_7 );
	// Habilitación de GPIOD
	Encoder_Pin(SYSCTL_PERIPH_GPIOD, GPIO_PORTD_BASE, GPIO_PIN_6 );
	Encoder_Pin(SYSCTL_PERIPH_GPIOD, GPIO_PORTD_BASE, GPIO_PIN_7 );
	// Habilitación de GPIOF
	Encoder_Pin(SYSCTL_PERIPH_GPIOF, GPIO_PORTF_BASE, GPIO_PIN_4 );

	/************************************
	* Habilitación global de interrupciones
	*************************************/
	IntMasterEnable();

	/********************************
	* Rutinas de inicio de periféricos
	*********************************/
	init_ADC();

	/********************************
	* Interrupción Timer a 2ms
	*********************************/
	//TimerBegin();

	//ServoAttach(SYSCTL_PERIPH_GPIOF,GPIO_PORTF_BASE,GPIO_PIN_1);
	double y=0;
	while(1){
		Read_input_encoder();
		if ( B== 1){
			y = (muestra/43.90) + 1.69*yn1 - 0.7225*yn2 ;
			yn2 = yn1; yn1 = y;
			B=0;

			SetPoint = y*0.2442;
		}
	}


	//return 0;
}
/**************************************************
* Rutina de atención a la interrupción del ADC
*
* En esta rutina se realiza todo elproceso del filtrado
* - Lectura de la muestra de entrada del ADC
* - Calculo de la muestra de salida
* - Actualización de los retardos
* - Envio de la muestra de salida al DAC
**************************************************/
void ADC0SS3IntHandler(void){
    ADCIntClear(ADC0_BASE,3);
    ADCSequenceDataGet(ADC0_BASE,3,&muestra);
    B=1;
}

