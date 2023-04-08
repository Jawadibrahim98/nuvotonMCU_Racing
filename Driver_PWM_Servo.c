//
// Driver_PWM.c
//
#include <stdio.h>
#include "NUC1xx.h"
#include "Driver\DrvUART.h"
#include "Driver\DrvGPIO.h"
#include "Driver\DrvSYS.h"

#define  PWM_CLKSRC_SEL   0        //0: 12M, 1:32K, 2:HCLK, 3:22M
#define  PWM_ClockSource  12000000 // 12M
#define  PWM_PreScaler    119      // clock is divided by (PreScaler + 1)
#define  PWM_ClockDivider 4        // 0: 1/2, 1: 1/4, 2: 1/8, 3: 1/16, 4: 1

#define  SERVO_CYCTIME        2000 // 20ms
#define  SERVO_HITIME_MIN       50 // minimum Hi width = 0.5ms
#define  SERVO_HITIME_MAX      250 // maximum Hi width = 2.5ms

void InitPWM(uint8_t PWM_no)
{
 	/* Step 1. GPIO initial */ 
	switch (PWM_no) {
		case 0 : SYS->GPAMFP.PWM0_AD13=1;     // Enable PWM0 multi-function pin
             SYSCLK->CLKSEL1.PWM01_S = PWM_CLKSRC_SEL; // Select 12Mhz for PWM clock source		
             SYSCLK->APBCLK.PWM01_EN =1;  // Enable PWM0 & PWM1 clock	
	           PWMA->PPR.CP01=1;			      // Prescaler 0~255, Setting 0 to stop output clock
	           PWMA->CSR.CSR0=0;			      // PWM clock = clock source/(Prescaler + 1)/divider
	           PWMA->PCR.CH0MOD=1;			    // 0:One-shot mode, 1:Auto-load mode
 								                          // CNR and CMR will be auto-cleared after setting CH0MOD form 0 to 1.	
	           PWMA->CNR0=0xFFFF;           // CNR : counting down   // PWM output high if CMRx+1 >= CNR
	           PWMA->CMR0=0xFFFF;		        // CMR : fix to compare  // PWM output low  if CMRx+1 <  CNR
	           PWMA->PCR.CH0INV=0;          // Inverter->0:off, 1:on
	           PWMA->PCR.CH0EN=1;			      // PWM function->0:Disable, 1:Enable
  	         PWMA->POE.PWM0=1;			      // Output to pin->0:Diasble, 1:Enable		
		         break;
		case 1 : SYS->GPAMFP.PWM1_AD14=1;     // Enable PWM1 multi-function pin
             SYSCLK->CLKSEL1.PWM01_S = PWM_CLKSRC_SEL; // Select 12Mhz for PWM clock source		
             SYSCLK->APBCLK.PWM01_EN =1;  // Enable PWM0 & PWM1 clock	
	           PWMA->PPR.CP01=1;			      // Prescaler 0~255, Setting 0 to stop output clock
	           PWMA->CSR.CSR1=0;			      // PWM clock = clock source/(Prescaler + 1)/divider
	           PWMA->PCR.CH1MOD=1;			    // 0:One-shot mode, 1:Auto-load mode
								                          // CNR and CMR will be auto-cleared after setting CH1MOD form 0 to 1.	
	           PWMA->CNR1=0xFFFF;           // CNR : counting down   // PWM output high if CMRx+1 >= CNR
	           PWMA->CMR1=0xFFFF;		        // CMR : fix to compare  // PWM output low  if CMRx+1 <  CNR		
	           PWMA->PCR.CH1INV=0;          // Inverter->0:off, 1:on
	           PWMA->PCR.CH1EN=1;			      // PWM function->0:Disable, 1:Enable
  	         PWMA->POE.PWM1=1;			      // Output to pin->0:Diasble, 1:Enable							 
						 break;
		case 2 : SYS->GPAMFP.PWM2_AD15=1;     // Enable PWM2 multi-function pin
             SYSCLK->CLKSEL1.PWM23_S = PWM_CLKSRC_SEL; // Select 12Mhz for PWM clock source		
             SYSCLK->APBCLK.PWM23_EN =1;  // Enable PWM2 & PWM3 clock	
	           PWMA->PPR.CP23=1;			      // Prescaler 0~255, Setting 0 to stop output clock
	           PWMA->CSR.CSR2=0;			      // PWM clock = clock source/(Prescaler + 1)/divider
	           PWMA->PCR.CH2MOD=1;			    // 0:One-shot mode, 1:Auto-load mode
								                          // CNR and CMR will be auto-cleared after setting CH2MOD form 0 to 1.	
	           PWMA->CNR2=0xFFFF;           // CNR : counting down   // PWM output high if CMRx+1 >= CNR
	           PWMA->CMR2=0xFFFF;		        // CMR : fix to compare  // PWM output low  if CMRx+1 <  CNR
	           PWMA->PCR.CH2INV=0;          // Inverter->0:off, 1:on
	           PWMA->PCR.CH2EN=1;			      // PWM function->0:Disable, 1:Enable
  	         PWMA->POE.PWM2=1;			      // Output to pin->0:Diasble, 1:Enable						 
		         break;						 
		case 3 : SYS->GPAMFP.PWM3_I2SMCLK=1;  // Enable PWM3 multi-function pin
             SYSCLK->CLKSEL1.PWM23_S = PWM_CLKSRC_SEL; // Select 12Mhz for PWM clock source		
             SYSCLK->APBCLK.PWM23_EN =1;  // Enable PWM2 & PWM3 clock	
	           PWMA->PPR.CP23=1;			      // Prescaler 0~255, Setting 0 to stop output clock
	           PWMA->CSR.CSR3=0;			      // PWM clock = clock source/(Prescaler + 1)/divider
	           PWMA->PCR.CH3MOD=1;			    // 0:One-shot mode, 1:Auto-load mode
								                          // CNR and CMR will be auto-cleared after setting CH2MOD form 0 to 1.	
	           PWMA->CNR3=0xFFFF;           // CNR : counting down   // PWM output high if CMRx+1 >= CNR
	           PWMA->CMR3=0xFFFF;		        // CMR : fix to compare  // PWM output low  if CMRx+1 <  CNR
	           PWMA->PCR.CH3INV=0;          // Inverter->0:off, 1:on
	           PWMA->PCR.CH3EN=1;			      // PWM function->0:Disable, 1:Enable
  	         PWMA->POE.PWM3=1;			      // Output to pin->0:Diasble, 1:Enable						 
		         break;  
		case 4 : SYS->GPBMFP.TM3_PWM4=1;      // Enable PWM0 multi-function pin
             SYSCLK->CLKSEL2.PWM45_S = PWM_CLKSRC_SEL; // Select 12Mhz for PWM clock source		
             SYSCLK->APBCLK.PWM45_EN =1;  // Enable PWM0 & PWM1 clock	
	           PWMB->PPR.CP01=1;			      // Prescaler 0~255, Setting 0 to stop output clock
	           PWMB->CSR.CSR0=0;			      // PWM clock = clock source/(Prescaler + 1)/divider
	           PWMB->PCR.CH0MOD=1;			    // 0:One-shot mode, 1:Auto-load mode
 								                          // CNR and CMR will be auto-cleared after setting CH0MOD form 0 to 1.	
	           PWMB->CNR0=0xFFFF;           // CNR : counting down   // PWM output high if CMRx+1 >= CNR
	           PWMB->CMR0=0xFFFF;		        // CMR : fix to compare  // PWM output low  if CMRx+1 <  CNR
	           PWMB->PCR.CH0INV=0;          // Inverter->0:off, 1:on
	           PWMB->PCR.CH0EN=1;			      // PWM function->0:Disable, 1:Enable
  	         PWMB->POE.PWM0=1;			      // Output to pin->0:Diasble, 1:Enable		
		         break; 
		case 5 : SYS->GPEMFP.PWM5=1;          // Enable PWM5 multi-function pin
		         SYSCLK->APBCLK.PWM45_EN = 1; // Enable PWM45 clock
             SYSCLK->CLKSEL2.PWM45_S = PWM_CLKSRC_SEL; // Select 12Mhz for PWM clock source		
             SYSCLK->APBCLK.PWM45_EN =1;  // Enable PWM4 & PWM5 clock	
	           PWMB->PPR.CP01=1;			      // Prescaler 0~255, Setting 0 to stop output clock
	           PWMB->CSR.CSR1=0;			      // PWM clock = clock source/(Prescaler + 1)/divider
	           PWMB->PCR.CH1MOD=1;			    // 0:One-shot mode, 1:Auto-load mode
								                          // CNR and CMR will be auto-cleared after setting CH1MOD form 0 to 1.	
	           PWMB->CNR1=0xFFFF;           // CNR : counting down   // PWM output high if CMRx+1 >= CNR
	           PWMB->CMR1=0xFFFF;		        // CMR : fix to compare  // PWM output low  if CMRx+1 <  CNR		
	           PWMB->PCR.CH1INV=0;          // Inverter->0:off, 1:on
	           PWMB->PCR.CH1EN=1;			      // PWM function->0:Disable, 1:Enable
  	         PWMB->POE.PWM1=1;			      // Output to pin->0:Diasble, 1:Enable							 
						 break;
		case 6 : SYS->GPEMFP.PWM6=1;          // Enable PWM2 multi-function pin
             SYSCLK->CLKSEL2.PWM67_S = PWM_CLKSRC_SEL; // Select 12Mhz for PWM clock source		
             SYSCLK->APBCLK.PWM67_EN =1;  // Enable PWM2 & PWM3 clock	
	           PWMB->PPR.CP23=1;			      // Prescaler 0~255, Setting 0 to stop output clock
	           PWMB->CSR.CSR2=0;			      // PWM clock = clock source/(Prescaler + 1)/divider
	           PWMB->PCR.CH2MOD=1;			    // 0:One-shot mode, 1:Auto-load mode
								                          // CNR and CMR will be auto-cleared after setting CH2MOD form 0 to 1.	
	           PWMB->CNR2=0xFFFF;           // CNR : counting down   // PWM output high if CMRx+1 >= CNR
	           PWMB->CMR2=0xFFFF;		        // CMR : fix to compare  // PWM output low  if CMRx+1 <  CNR
	           PWMB->PCR.CH2INV=0;          // Inverter->0:off, 1:on
	           PWMB->PCR.CH2EN=1;			      // PWM function->0:Disable, 1:Enable
  	         PWMB->POE.PWM2=1;			      // Output to pin->0:Diasble, 1:Enable						 
		         break;							 
		case 7 : SYS->GPEMFP.PWM7=1;          // Enable PWM3 multi-function pin
             SYSCLK->CLKSEL2.PWM67_S = PWM_CLKSRC_SEL; // Select 12Mhz for PWM clock source		
             SYSCLK->APBCLK.PWM67_EN =1;  // Enable PWM2 & PWM3 clock	
	           PWMB->PPR.CP23=1;			      // Prescaler 0~255, Setting 0 to stop output clock
	           PWMB->CSR.CSR3=0;			      // PWM clock = clock source/(Prescaler + 1)/divider
	           PWMB->PCR.CH3MOD=1;			    // 0:One-shot mode, 1:Auto-load mode
								                          // CNR and CMR will be auto-cleared after setting CH2MOD form 0 to 1.	
	           PWMB->CNR3=0xFFFF;           // CNR : counting down   // PWM output high if CMRx+1 >= CNR
	           PWMB->CMR3=0xFFFF;		        // CMR : fix to compare  // PWM output low  if CMRx+1 <  CNR
	           PWMB->PCR.CH3INV=0;          // Inverter->0:off, 1:on
	           PWMB->PCR.CH3EN=1;			      // PWM function->0:Disable, 1:Enable
  	         PWMB->POE.PWM3=1;			      // Output to pin->0:Diasble, 1:Enable						 
		         break;
		default :break;
	}
}

void PWM_Stop(uint8_t PWM_no)
{		
	switch(PWM_no) {
		case 0 : PWMA->POE.PWM0=0; break;
		case 1 : PWMA->POE.PWM1=0; break;
		case 2 : PWMA->POE.PWM2=0; break;
		case 3 : PWMA->POE.PWM3=0; break;
		case 4 : PWMB->POE.PWM0=0; break;
		case 5 : PWMB->POE.PWM1=0; break;
		case 6 : PWMB->POE.PWM2=0; break;
		case 7 : PWMB->POE.PWM3=0; break;				
		default: PWMA->POE.PWM0=1;
		         PWMA->POE.PWM1=1;
						 PWMA->POE.PWM2=1;
						 PWMA->POE.PWM3=1;
		         PWMB->POE.PWM0=1;
						 PWMB->POE.PWM1=1;
						 PWMB->POE.PWM2=1;
						 PWMB->POE.PWM3=1;
						 break;
		}
}

// cyctime & hitime unit is 10us
void PWM_Servo(uint8_t PWM_no, uint16_t Servo_HiTime)
{
    //PWM_FreqOut = PWM_Clock / (PWM_PreScaler + 1) / PWM_ClockDivider / (PWM_CNR + 1); 
	  // Duty Cycle = (CMR0+1) / (CNR0+1)

	  //PWM setting	 
		switch(PWM_no) {
			case 0 : PWMA->CSR.CSR0 = 0;             // divider factor = 0: 1/2, 1: 1/4, 2: 1/8, 3: 1/16, 4: 1
	             PWMA->PPR.CP01 = PWM_PreScaler; // set PreScaler
			         PWMA->CNR0 = SERVO_CYCTIME -1;    // set CNR
	             PWMA->CMR0 = Servo_HiTime -1;     // set CMR
						   PWMA->POE.PWM0=1;
			         break;
			case 1 : PWMA->CSR.CSR1 = 0;             // divider factor = 0: 1/2, 1: 1/4, 2: 1/8, 3: 1/16, 4: 1
	             PWMA->PPR.CP01 = PWM_PreScaler; // set PreScaler
			         PWMA->CNR1 = SERVO_CYCTIME -1;    // set CNR
	             PWMA->CMR1 = Servo_HiTime -1;     // set CMR
			         PWMA->POE.PWM1=1;
			         break;
			case 2 : PWMA->CSR.CSR2 = 0;             // diver factor = 0: 1/2, 1: 1/4, 2: 1/8, 3: 1/16, 4: 1
	             PWMA->PPR.CP23 = PWM_PreScaler; // set PreScaler
			         PWMA->CNR2 = SERVO_CYCTIME -1;    // set CNR
	             PWMA->CMR2 = Servo_HiTime -1;     // set CMR
			         PWMA->POE.PWM2=1;
			         break;
			case 3 : PWMA->CSR.CSR3 = 0;             // diver factor = 0: 1/2, 1: 1/4, 2: 1/8, 3: 1/16, 4: 1
	             PWMA->PPR.CP23 = PWM_PreScaler; // set PreScaler
			         PWMA->CNR3 = SERVO_CYCTIME -1;    // set CNR
	             PWMA->CMR3 = Servo_HiTime -1;     // set CMR
							 PWMA->POE.PWM3=1;
			         break;
			case 4 : PWMB->CSR.CSR0 = 0;             // diver factor = 0: 1/2, 1: 1/4, 2: 1/8, 3: 1/16, 4: 1
	             PWMB->PPR.CP01 = PWM_PreScaler; // set PreScaler
			         PWMB->CNR0 = SERVO_CYCTIME -1;    // set CNR
	             PWMB->CMR0 = Servo_HiTime -1;     // set CMR
							 PWMB->POE.PWM0=1;
			         break;
			case 5 : PWMB->CSR.CSR1 = 0;             // diver factor = 0: 1/2, 1: 1/4, 2: 1/8, 3: 1/16, 4: 1
	             PWMB->PPR.CP01 = PWM_PreScaler; // set PreScaler
			         PWMB->CNR1 = SERVO_CYCTIME -1;    // set CNR
	             PWMB->CMR1 = Servo_HiTime -1;     // set CMR
							 PWMB->POE.PWM1=1;
			         break;
			case 6 : PWMB->CSR.CSR2 = 0;             // diver factor = 0: 1/2, 1: 1/4, 2: 1/8, 3: 1/16, 4: 1
	             PWMB->PPR.CP23 = PWM_PreScaler; // set PreScaler
			         PWMB->CNR2 = SERVO_CYCTIME -1;    // set CNR
	             PWMB->CMR2 = Servo_HiTime -1;     // set CMR
							 PWMB->POE.PWM2=1;
			         break;
			case 7 : PWMB->CSR.CSR3 = 0;             // diver factor = 0: 1/2, 1: 1/4, 2: 1/8, 3: 1/16, 4: 1
	             PWMB->PPR.CP23 = PWM_PreScaler; // set PreScaler
			         PWMB->CNR3 = SERVO_CYCTIME -1;    // set CNR
	             PWMB->CMR3 = Servo_HiTime -1;     // set CMR
							 PWMB->POE.PWM3=1;
			         break;			
			default: break;
	}
}














