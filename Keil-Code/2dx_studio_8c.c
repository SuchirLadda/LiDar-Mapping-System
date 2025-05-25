/*  
Author: Suchir Ladda
Date: March 30th 2025
COMP ENG 2DX3 Final Porject
Assigned bus speed: 12 MHz
Measurment Status LED: PF4 - D3
Additional Status LED: PN0 - D2
*/


/*  Time of Flight for 2DX4 -- Studio W8-0
                Code written to support data collection from VL53L1X using the Ultra Light Driver.
                I2C methods written based upon MSP432E4 Reference Manual Chapter 19.
                Specific implementation was based upon format specified in VL53L1X.pdf pg19-21
                Code organized according to en.STSW-IMG009\Example\Src\main.c
                
                The VL53L1X is run with default firmware settings.


            Written by Tom Doyle
            Updated by  Hafez Mousavi Garmaroudi
            Last Update: March 17, 2020
						
						Last Update: March 03, 2022
						Updated by Hafez Mousavi
						__ the dev address can now be written in its original format. 
								Note: the functions  beginTxI2C and  beginRxI2C are modified in vl53l1_platform_2dx4.c file
								
						Modified March 16, 2023 
						by T. Doyle
							- minor modifications made to make compatible with new Keil IDE

*/
#include <stdint.h>
#include "PLL.h"
#include "SysTick.h"
#include "uart.h"
#include "onboardLEDs.h"
#include "tm4c1294ncpdt.h"
#include "VL53L1X_api.h"


#define I2C_MCS_ACK             0x00000008  // Data Acknowledge Enable
#define I2C_MCS_DATACK          0x00000008  // Acknowledge Data
#define I2C_MCS_ADRACK          0x00000004  // Acknowledge Address
#define I2C_MCS_STOP            0x00000004  // Generate STOP
#define I2C_MCS_START           0x00000002  // Generate START
#define I2C_MCS_ERROR           0x00000002  // Error
#define I2C_MCS_RUN             0x00000001  // I2C Master Enable
#define I2C_MCS_BUSY            0x00000001  // I2C Busy
#define I2C_MCR_MFE             0x00000010  // I2C Master Function Enable
#define MAXRETRIES              5           // number of receive attempts before giving up


#define ENABLE_INTERRUPTS()   __asm("cpsie i")
#define DISABLE_INTERRUPTS()  __asm("cpsid i")
#define WAIT_FOR_INTERRUPT()  __asm("wfi")
/*
Initialize I2C - got this straight from the 8c code
*/

// Global variables used as flags that are manipulated by the interrupts
volatile unsigned int rotation = 0;
volatile unsigned int scan_enable = 0;


	
void I2C_Init(void){
  SYSCTL_RCGCI2C_R |= SYSCTL_RCGCI2C_R0;           													// activate I2C0
  SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R1;          												// Enables the clock for Port B, which contains the I2C pins.
  while((SYSCTL_PRGPIO_R&0x0002) == 0){};																		// wait for port b to be ready

    GPIO_PORTB_AFSEL_R |= 0x0C;           																	// 3) enable alt funct on PB2,3       0b00001100
    GPIO_PORTB_ODR_R |= 0x08;             																	// 4) enable open drain on PB3 only

    GPIO_PORTB_DEN_R |= 0x0C;             																	// 5) enable digital I/O on PB2,3
//  GPIO_PORTB_AMSEL_R &= ~0x0C;          																	// 7) disable analog functionality on PB2,3

                                                                            // 6) configure PB2,3 as I2C
//  GPIO_PORTB_PCTL_R = (GPIO_PORTB_PCTL_R&0xFFFF00FF)+0x00003300;
		GPIO_PORTB_PCTL_R = (GPIO_PORTB_PCTL_R&0xFFFF00FF)+0x00002200;    //TED - Bit-masking ensures we only modify PB2 and PB3.
    I2C0_MCR_R = I2C_MCR_MFE;                      													// 9) master function enable
    I2C0_MTPR_R = 0b0000000000000101000000000111011;                       	// 8) configure for 100 kbps clock (added 8 clocks of glitch suppression ~50ns)
//  I2C0_MTPR_R = 0x3B;                                        							// 8) configure for 100 kbps clock
        
}

//just for reference PB2 (I2C0SCL - Clock) and PB3 (I2C0SDA - Data) are used for I2C
//also AFSEL stands for Alternate Function Enable

//VL53L1X Port G Intialization 
//The VL53L1X needs to be reset using XSHUT.  We will use PG0
void PortG_Init(void){
    //Use PortG0
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R6;                // activate clock for Port N
    while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R6) == 0){};    		// allow time for clock to stabilize
    GPIO_PORTG_DIR_R &= 0x00;                               // make PG0 in (HiZ)
		GPIO_PORTG_AFSEL_R &= ~0x01;                            // disable alt funct on PG0
		GPIO_PORTG_DEN_R |= 0x01;                               // enable digital I/O on PG0
                                                            // configure PG0 as GPIO
//  GPIO_PORTN_PCTL_R = (GPIO_PORTN_PCTL_R&0xFFFFFF00)+0x00000000;
			GPIO_PORTG_AMSEL_R &= ~0x01;                            // disable analog functionality on PN0 - AMSEL: Analog Mode Select Register

    return;
}

//Port H Initalization , i dont need to init all of port H
void PortH0H1H2H3_Init(void){
	SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R7;                 
	while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R7) == 0){};         
	GPIO_PORTH_DIR_R = 0b00001111;       
  GPIO_PORTH_DEN_R = 0b00001111;  
	return;
}

/*
void PortH_Init(void){
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R7;                  
    while((SYSCTL_PRGPIO_R & SYSCTL_PRGPIO_R7) == 0){};        
    GPIO_PORTH_DIR_R = 0xFF;  // Set H0-H7 as outputs
    GPIO_PORTH_DEN_R = 0xFF;  // Enable digital functionality for H0-H7
    return;
}
*/

//Initialize Port E as output - only need pin E0 to plug in ad3
void PortE0_Init(void){
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R4;  // Enable clock for Port E
    while((SYSCTL_PRGPIO_R & SYSCTL_PRGPIO_R4) == 0){};  // Wait for clock to stabilize
    GPIO_PORTE_DIR_R |= 0x01;  // Set PE0 as output
    GPIO_PORTE_DEN_R |= 0x01;  // Enable digital function on PE0
    return;
}

void EnableInt(void){
	 ENABLE_INTERRUPTS();//enable interrupts globally
}


void DisableInt(void){
	 DISABLE_INTERRUPTS();//disable interrupts globally
}

// Low power wait
void WaitForInt(void){
	WAIT_FOR_INTERRUPT();//tells the microcontroller to enter a low-power state and wait for an interrupt
}

//i used interrupts bc polling is a huge waste of time and power


//Port J Intialization
void PortJ_Init(void){
	SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R8;					// Activate clock for Port J
	while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R8) == 0){};	// Allow time for clock to stabilize
  GPIO_PORTJ_DIR_R &= ~0x03;    										// Make PJ1 & PJ0 input 
  GPIO_PORTJ_DEN_R |= 0x03;     										// Enable digital I/O on PJ1
	
	GPIO_PORTJ_PCTL_R &= ~0x000000F0;	 								// Configure PJ1 & PJ0 as GPIO 
	GPIO_PORTJ_AMSEL_R &= ~0x03;											// Disable analog functionality on PJ1 & PJ0	
	GPIO_PORTJ_PUR_R |= 0x03;													// Enable pull up resistor
}



void PortJ_Interrupt_Init(void){
    rotation = 0;                     // Start with rotation disabled
    scan_enable = 0;                  // Scanning is initially turned off

    // Configure PJ0 and PJ1 to trigger interrupts on signal edges
    GPIO_PORTJ_IS_R = 0;             // Edge-sensitive interrupts (not level-based)
    GPIO_PORTJ_IBE_R = 0;            // Only one edge will generate an interrupt (not both)
    GPIO_PORTJ_IEV_R = 0;            // Falling edge triggers the interrupt for PJ0 and PJ1

    // Clear any prior interrupt flags to avoid false triggers
    GPIO_PORTJ_ICR_R = 0x03;         // Reset interrupt flags for PJ0 and PJ1

    // Enable interrupt generation for both PJ0 and PJ1
    GPIO_PORTJ_IM_R = 0x03;          // Unmask PJ0 and PJ1 so they can trigger interrupts

    // Register the Port J interrupt with the NVIC (interrupt number 51)
    NVIC_EN1_R = 0x00080000;         // Enable Port J in the NVIC

    // Assign a mid-level priority (5) to Port J interrupts
    NVIC_PRI12_R = 0xA0000000;       // Set priority level for this interrupt source

    // Turn on global interrupt processing at the processor level
    EnableInt();                     // Now the CPU can respond to interrupts
}

// Interrupt Service Routine for Port J — handles button presses on PJ0 and PJ1
void GPIOJ_IRQHandler(void){	        
    if((GPIO_PORTJ_DATA_R & 0x01) == 0) {       // If PJ0 is pressed (active low)
        rotation = !rotation;                   // Flip the rotation flag (toggle motor on/off)
        FlashLED3(2);                            // Briefly flash LED3 as visual feedback
        GPIO_PORTJ_ICR_R = 0x01;                // Clear PJ0 interrupt flag so it can trigger again
    }
    else if((GPIO_PORTJ_DATA_R & 0x02) == 0) {  // If PJ1 is pressed (active low)
        scan_enable = !scan_enable;             // Toggle scanning mode (start or stop)
        ToggleLED2(1);                           // Change state of LED2 to show scan mode change
        GPIO_PORTJ_ICR_R = 0x02;                // Clear PJ1 interrupt flag to reset it
    }
    return;
}


//rotation Stepper Motor
void rotationControl(int direction, int step, int delay){
	//ccw
	if(direction == -1){
		for(int i=0; i<step; i++){
			GPIO_PORTH_DATA_R = 0b00001100;
			SysTick_Wait1ms(delay);
			GPIO_PORTH_DATA_R = 0b00000110;
			SysTick_Wait1ms(delay);
			GPIO_PORTH_DATA_R = 0b00000011;
			SysTick_Wait1ms(delay);
			GPIO_PORTH_DATA_R = 0b00001001;
			SysTick_Wait1ms(delay);
		}

	}
	//cw
	else if(direction == 1){
		for(int i=0; i<step; i++){
			GPIO_PORTH_DATA_R = 0b00001001;
			SysTick_Wait1ms(delay);
			GPIO_PORTH_DATA_R = 0b00000011;
			SysTick_Wait1ms(delay);
			GPIO_PORTH_DATA_R = 0b00000110;
			SysTick_Wait1ms(delay);
			GPIO_PORTH_DATA_R = 0b00001100;
			SysTick_Wait1ms(delay);

		}
	}

}


//got this from 8c too
//XSHUT     This pin is an active-low shutdown input; 
//					the board pulls it up to VDD to enable the sensor by default. 
//					Driving this pin low puts the sensor into hardware standby. This input is not level-shifted.
void VL53L1X_XSHUT(void){
    GPIO_PORTG_DIR_R |= 0x01;                                        // make PG0 out
    GPIO_PORTG_DATA_R &= 0b11111110;                                 //PG0 = 0
    FlashAllLEDs();
    SysTick_Wait10ms(1);
    GPIO_PORTG_DIR_R &= ~0x01;                                            // make PG0 input (HiZ)
}


//Scan and send data with UART
void startScan(int scan, int status, uint16_t	dev, uint8_t dataReady, uint16_t Distance, uint16_t SignalRate, uint16_t AmbientRate, uint16_t SpadNum, uint8_t RangeStatus, int depth, int count){		
		
		SysTick_Wait10ms(10); 
		for(int i = 0; i < 1; i++){
			
			while (dataReady == 0){
				status = VL53L1X_CheckForDataReady(dev, &dataReady);
						FlashLED3(1);
						VL53L1_WaitMs(dev, 5);
			}
			dataReady = 0;
			
			
			status = VL53L1X_GetRangeStatus(dev, &RangeStatus);
			status = VL53L1X_GetDistance(dev, &Distance);					
			status = VL53L1X_GetSignalRate(dev, &SignalRate);
			status = VL53L1X_GetAmbientRate(dev, &AmbientRate);
			status = VL53L1X_GetSpadNb(dev, &SpadNum);

			status = VL53L1X_ClearInterrupt(dev); 
			int step = count;
			if (depth%2 == 1){ 
				step = 512-count;
			}
			sprintf(printf_buffer,"%u, %u, %u, %u, %u\r\n", RangeStatus, Distance, step, depth, SpadNum);
			UART_printf(printf_buffer);
			SysTick_Wait10ms(5);
		}
}

//full rotation
void stepperCounter(int dir, int steps, int delay, int count, int depth, int scan,int status, uint16_t	dev, uint8_t dataReady, uint16_t Distance, uint16_t SignalRate, uint16_t AmbientRate, uint16_t SpadNum, uint8_t RangeStatus ){
	while(1){
		if(scan_enable && scan == 1) {
			startScan(scan, status, dev, dataReady, Distance, SignalRate, AmbientRate, SpadNum, RangeStatus, depth, count);
			scan = 0;
		}
			
		if (rotation == 1){
			rotationControl(dir, 1, delay);
			count++;
		}
		
		// EVERY 11.25 DEGREES FLASH THE DATA ACQUISITION LED, WAIT, AND SET SCAN TO 1
		// scan = 1 everytime we get to 16 steps (every 11.25 degrees)

		if (count % 16 == 0 && rotation == 1 && count != 0){
			if (scan_enable){
				FlashLED2(1);
				scan = 1; 
			}
		}

		
		if (count > 512){
				rotation = 0;
				dir *= -1;
				depth += 1;
				count = 0;
				
		}
	}
}

//Bus Speed check
void busSpeed(){
	while (1){
		GPIO_PORTE_DATA_R ^= 0b00000001;
		SysTick_Wait10ms(1);
	}
}


//Main
int main(void) {
  uint16_t	dev = 0x29;			
	int status=0;
	uint8_t byteData, sensorState=0, myByteArray[10] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF} , i=0;
  uint16_t wordData;
  uint16_t Distance;
  uint16_t SignalRate;
  uint16_t AmbientRate;
  uint16_t SpadNum; 
  uint8_t RangeStatus;
  uint8_t dataReady;

	PLL_Init();	
	SysTick_Init();
	onboardLEDs_Init();
	I2C_Init();
	UART_Init();
	PortJ_Init();
	PortJ_Interrupt_Init();
	PortH0H1H2H3_Init();
	
	// Initialize variables 
	int dir = -1;
	int steps = 512;
	int delay = 5;
	int count = 0;
	int depth = 0;
	int scan = 0;
	
	// uncomment this to show the bus speed
	//PortE0_Init();
	//busSpeed();
	
	status = VL53L1X_GetSensorId(dev, &wordData);

	while(sensorState==0){
		status = VL53L1X_BootState(dev, &sensorState);
		SysTick_Wait10ms(10);
  }
	FlashAllLEDs();
	
	status = VL53L1X_ClearInterrupt(dev); 
  status = VL53L1X_SensorInit(dev);
	Status_Check("SensorInit", status);
	status = VL53L1X_SetDistanceMode(dev, 2); 
  status = VL53L1X_StartRanging(dev);   


	while (1){ 
			WaitForInt(); 
			if (rotation==1){ 
				stepperCounter(dir, steps, delay, count, depth, scan,status, dev, dataReady, Distance, SignalRate, AmbientRate, SpadNum, RangeStatus);
			}
	}
	VL53L1X_StopRanging(dev);
  while(1) {}
}


