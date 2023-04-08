
#include <stdio.h>	
#include <string.h>
#include "NUC1xx.h"
#include "Driver\DrvSYS.h"
#include "Driver\DrvUART.h"
#include "Driver\DrvGPIO.h"
#include "Driver\DrvADC.h"
#include "LCD_Driver.h"
#include "Driver_PWM_Servo.h"
//
#define  ONESHOT  0   // counting and interrupt when reach TCMPR number, then stop
#define  PERIODIC 1   // counting and interrupt when reach TCMPR number, then counting from 0 again
#define  TOGGLE   2   // keep counting and interrupt when reach TCMPR number, tout toggled (between 0 and 1)
#define  CONTINUOUS 3 // keep counting and interrupt when reach TCMPR number
//
#define INIT_RED DrvGPIO_Open(E_GPB, 14, E_IO_OUTPUT) 
#define RED_ON DrvGPIO_ClrBit(E_GPB, 14)
#define RED_OFF DrvGPIO_SetBit(E_GPB, 14)
#define INIT_YELLOW1 DrvGPIO_Open(E_GPB, 10, E_IO_OUTPUT) 
#define YELLOW1_ON DrvGPIO_ClrBit(E_GPB, 10)
#define YELLOW1_OFF DrvGPIO_SetBit(E_GPB, 10)
#define INIT_YELLOW2 DrvGPIO_Open(E_GPB, 12, E_IO_OUTPUT) 
#define YELLOW2_ON DrvGPIO_ClrBit(E_GPB, 12)
#define YELLOW2_OFF DrvGPIO_SetBit(E_GPB, 12) 
#define INIT_GREEN DrvGPIO_Open(E_GPB, 13, E_IO_OUTPUT) 
#define GREEN_ON DrvGPIO_ClrBit(E_GPB, 13)
#define GREEN_OFF DrvGPIO_SetBit(E_GPB, 13)
//
#define CLOSED 50  // 0.5ms
#define OPEN 100 // 2ms
#define DARK_SNS 300
//STATE MACHINE
#define READY 0
#define LIGHTS 1
#define RACING 2
#define DONE 3
#define FAULT 4
 //////////////

/*GPA0 - LDR0
	GPA1 - LDR1
	GPB0 - HC05 - TX
	GPB1 - HC05 - RX
	GPA12- SG90 -PWM
	GPA13- DF9  -PWM
	GPB14 - RED
	GPB13 - GREEN
	GPB12 - YELLOW2
	GPB10 - YELLOW1


*/
// Timer0 initialize to tick every 1ms
void InitTIMER0(void)
{
	/* Step 1. Enable and Select Timer clock source */          
	SYSCLK->CLKSEL1.TMR0_S = 0;	//Select 12Mhz for Timer0 clock source 
	SYSCLK->APBCLK.TMR0_EN =1;	//Enable Timer0 clock source

	/* Step 2. Select Operation mode */	
	TIMER0->TCSR.MODE = PERIODIC;		//Select once mode for operation mode

	/* Step 3. Select Time out period = (Period of timer clock input) * (8-bit Prescale + 1) * (24-bit TCMP)*/
	TIMER0->TCSR.PRESCALE = 11;	// Set Prescale [0~255]
	TIMER0->TCMPR = 1000;		// Set TCMPR [0~16777215]
	//Timeout period = (1 / 12MHz) * ( 11 + 1 ) * 1,000 = 1 ms

	/* Step 4. Enable interrupt */
	TIMER0->TCSR.IE = 1;
	TIMER0->TISR.TIF = 1;		//Write 1 to clear for safty		
	NVIC_EnableIRQ(TMR0_IRQn);	//Enable Timer0 Interrupt

	/* Step 5. Enable Timer module */
	TIMER0->TCSR.CRST = 1;	//Reset up counter
	TIMER0->TCSR.CEN = 0;		//Enable Timer0

//	TIMER0->TCSR.TDR_EN=1;		// Enable TDR function
}

uint32_t Timing_Counter_0=0;

void TMR0_IRQHandler(void) // Timer0 interrupt subroutine 
{
	Timing_Counter_0++;
	TIMER0->TISR.TIF =1; 	   
}

void Init_LED() // Initialize GPIO pins
{
	INIT_GREEN;
	INIT_YELLOW1;
	INIT_YELLOW2;
	INIT_RED;	
}

double finish_line_LDR(uint8_t car)
{
	int16_t adc_value;
	DrvADC_StartConvert();
	while(DrvADC_IsConversionDone()==FALSE); // wait till conversion flag = 1, conversion is done
	adc_value = ADC->ADDR[car].RSLT & 0xFFF; 	// input 12-bit ADC value
	ADC->ADSR.ADF=1;		     // write 1 to clear the flag
	return adc_value;
}

	volatile uint8_t comRbuf[9];
	volatile uint8_t comRbytes = 0;
	uint32_t response_car1 = 0;
	uint32_t response_car2 = 0;
	uint32_t total_car1 = 0;
	uint32_t total_car2 = 0;
	char rCMD[16];
	uint8_t state = READY;

void send_UART(char *cmd, uint32_t data){
	char sendTEXT[6];
	strcpy(sendTEXT, cmd);
	if(data == 0)
		strcat(sendTEXT,"0000");
	else
		sprintf(sendTEXT+2,"%d",data);
	DrvUART_Write(UART_PORT0, sendTEXT, 6);
}

void UART_INT_HANDLE(void)
{
	char TEXT[16];
	char sendTEXT[6];
	while(UART0->ISR.RDA_IF==1) 
	{
		comRbuf[comRbytes]=UART0->DATA;
		comRbytes++;		
		if(comRbuf[comRbytes-1] == 'X'){ //Data must end with 'X'
			comRbuf[comRbytes-1] = '\0';	
			sprintf(rCMD,"%s",comRbuf);
		  comRbytes = 0;
			
			if(!strcmp(rCMD, "READY")){ //READY FOR RACE
					state = READY;
			}
			else if(!strcmp(rCMD, "START")){ //START RACE
				if(state == READY )
					state = LIGHTS;
			}
			else if(!strcmp(rCMD, "L1")){ //CAR1 LAUNCHED
				if(state != RACING){
					state = FAULT;
					send_UART("F1", 0000);
					break;
				}
				if(response_car1 != 0)
					break;
				response_car1 = Timing_Counter_0;
				send_UART("R1", Timing_Counter_0);
				PWM_Servo(0, OPEN);		
			}
			else if(!strcmp(rCMD, "L2")){ //CAR2 LAUNCHED
				if(state != RACING){
					state = FAULT;
					send_UART("F2", 0000);
					break;
				}
				if(response_car2 != 0)
					break;
				response_car2 = Timing_Counter_0;
				send_UART("R2", Timing_Counter_0);
				PWM_Servo(1, 80);		
			}
		}
	}
}
void LIGHTS_OFF(){
	RED_OFF;
	GREEN_OFF;
	YELLOW1_OFF;
	YELLOW2_OFF; 
}
void LIGHTS_START(){
	LIGHTS_OFF();
	YELLOW1_ON;
	DrvSYS_Delay(1000000);
	DrvSYS_Delay(1000000);
	YELLOW2_ON;
	DrvSYS_Delay(1000000);
	DrvSYS_Delay(1000000);
	GREEN_ON;
}

void LIGHTS_FAULT(){
	LIGHTS_OFF();
	RED_OFF;
	DrvSYS_Delay(1000000);
	RED_ON;
	DrvSYS_Delay(1000000);
}

/*----------------------------------------------------------------------------
  MAIN function
  ----------------------------------------------------------------------------*/

int16_t light_sns;
int16_t dark_off1;
int16_t dark_off2;
char TEXT[4];

STR_UART_T sParam;

int32_t main (void)
{
	UNLOCKREG();
	SYSCLK->PWRCON.XTL12M_EN = 1;
	SYSCLK->CLKSEL0.HCLK_S = 0;
	DrvSYS_Delay(5000); // wait till 12MHz crystal stable
	DrvSYS_Open(48000000);
	LOCKREG();

	DrvGPIO_InitFunction(E_FUNC_UART0); // Set UART pins
  DrvADC_Open(ADC_SINGLE_END, ADC_CONTINUOUS_OP, 0x03, INTERNAL_HCLK, 1); // ADC1= LDR1 & ADC0	= LDR0

  /* UART Setting */
  sParam.u32BaudRate 	= 9600;
  sParam.u8cDataBits 	= DRVUART_DATABITS_8;
  sParam.u8cStopBits 	= DRVUART_STOPBITS_1;
  sParam.u8cParity 	= DRVUART_PARITY_NONE;
  sParam.u8cRxTriggerLevel= DRVUART_FIFO_1BYTES;

  /* Set UART Configuration */
	if(DrvUART_Open(UART_PORT0,&sParam) != E_SUCCESS);
	DrvUART_EnableInt(UART_PORT0, DRVUART_RDAINT, UART_INT_HANDLE);

	Initial_panel();  //initialize LCD
	clr_all_panel();  //clear LCD
	Init_LED();
	InitTIMER0();
	InitPWM(0);
	InitPWM(1);
	while(1)
	{
			switch(state){
				case READY:
					LIGHTS_OFF();
					dark_off1 = finish_line_LDR(0);
					dark_off2 = finish_line_LDR(1);
					PWM_Servo(0, CLOSED);		
					PWM_Servo(1, 120);
					response_car1 = 0;
					response_car2 = 0;
					total_car1 = 0;
					total_car2 = 0;
				break;

				case LIGHTS:
					LIGHTS_START();
					if(state == FAULT) //PRE LAUNCH
						break;
					TIMER0->TCSR.CRST = 1;	//Reset up counter
					Timing_Counter_0 = 0;
					TIMER0->TCSR.CEN = 1;		//Enable Timer0
					state = RACING;
				break;

				case RACING:
					if(total_car1 == 0 && response_car1 > 0){
						light_sns = finish_line_LDR(0);
						if(light_sns > (dark_off1 + DARK_SNS)){ //Car1 Done
							total_car1 = Timing_Counter_0;
							send_UART("T1", total_car1);
						}
					}
					if(total_car2 ==0 && response_car2 > 0){
						light_sns = finish_line_LDR(1);
						if(light_sns > (dark_off2 + DARK_SNS)){ //Car2 Done
							total_car2 = Timing_Counter_0;
							send_UART("T2", total_car2);
						}
					}
					if(total_car2 != 0 && total_car1 != 0){ //both done
						state = DONE;
						response_car1 = 0;
						response_car2 = 0;
						total_car1 = 0;
						total_car2 = 0;
					}
					else if(Timing_Counter_0 > 4000){ //Time over
						state = FAULT;
						if(total_car1 == 0)
							send_UART("F1", 0000);
						if(total_car2 == 0)
							send_UART("F2", 0000);
					}
				break;
				case DONE:
					LIGHTS_OFF();
				break;
				case FAULT:
					LIGHTS_FAULT();
				break;
			}
	}
}

