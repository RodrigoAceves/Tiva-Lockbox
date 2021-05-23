#include <stdint.h>
#include <stdbool.h>
#include "Rodrigo_Lockbox.h"
#include "inc/hw_types.h"
#include "inc/hw_memmap.h"
#include "inc/hw_gpio.h"
#include "driverlib/sysctl.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom_map.h"
#include "driverlib/gpio.h"
#include "inc/tm4c123gh6pm.h"
#include "driverlib/timer.h"				//include the timer library
#include "driverlib/interrupt.h" 		//include the interrupt library
#include "driverlib/pwm.h"					//include the pwm library

#define HI 0xFF	
#define LO 0x00

char pad[16] = {'1','2','3','A',
								'4','5','6','B',
								'7','8','9','C',
								'*','0','#','D'};
								
uint8_t counter = 0;				//This keeps track of which LED is being triggered
uint32_t sleepCounter;			//This variable keeps track of the idle time
								
uint8_t position = 0;				//This keeps track of the password position
char password[100];					//store the password here
								
uint8_t enterPosition = 0;	//keep track of the input password
char enterPassword[100];		//store the code here

bool passSet = false;
bool locked = false;
bool invalidPW = false;
								
int current = 0;

//-------------------------------------------------------------------------------------------------------
/*
This function initializes the the keypad pins, led pins, and the FSR sensor.
*/
//-------------------------------------------------------------------------------------------------------
								
void PortFunctionInit(void) {
	
		SysCtlDeepSleepPowerSet(SYSCTL_SRAM_LOW_POWER | SYSCTL_FLASH_LOW_POWER);
	
	
    // Enable Peripheral Clocks 
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
		SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
		SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);

    // Enable pin PE1, PE2, PE3, PE4, AND PE5 as output for the LED bar
    GPIOPinTypeGPIOOutput(GPIO_PORTE_BASE, GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5);

    // Enable pin PA2, PA3, PA4, AND PA5 for numberpad
    GPIOPinTypeGPIOOutput(GPIO_PORTA_BASE, GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5);
	
    // Enable pin PD0, PD1, PD2, AND PD3 for GPIOInput
    GPIOPinTypeGPIOInput(GPIO_PORTD_BASE, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3);

		//PC4 enabled as input for reset 
		GPIOPinTypeGPIOInput(GPIO_PORTC_BASE, GPIO_PIN_4);
	
		//PB3 enabled as input for box closed 
		GPIOPinTypeGPIOInput(GPIO_PORTB_BASE, GPIO_PIN_3);		
		
		//PA6 enabled as input for wake up interrupt 
		GPIOPinTypeGPIOInput(GPIO_PORTA_BASE, GPIO_PIN_6);		
}


//-------------------------------------------------------------------------------------------------------
/*
This function allows us to control delay with respect to milliseconds.
*/
//-------------------------------------------------------------------------------------------------------

void delayMS(int ms) {
	SysCtlDelay( (SysCtlClockGet()/(3*1000))*ms ) ;
}

//-------------------------------------------------------------------------------------------------------
/*
This function converts the servo angle to the respective PWM output. 
*/
//-------------------------------------------------------------------------------------------------------

double dutyCycle(double degrees){ //Convert Degrees to duty cycle
	return (degrees == 180)?(540):(degrees/180*540+ 1);
}

//-------------------------------------------------------------------------------------------------------
/*
This is the timer for the periodic interrupt. This interrupt controls the LED array which works as an 
indicator for when the box is locked.
*/
//-------------------------------------------------------------------------------------------------------

void intWake(void){
	//Make PA6 an interrupt sensitive to rising edge
	NVIC_EN0_R |= 0x00000001;
	NVIC_PRI0_R |= 0x00000080; //priority 3
	
	GPIO_PORTA_IM_R |= 0x40; //enable 0b0100.0000
	GPIO_PORTA_IS_R &= ~0x40; //edge
	GPIO_PORTA_IBE_R &= ~0x40; //one edge
	GPIO_PORTA_IEV_R |= 0x40; //rising edge
}

//-------------------------------------------------------------------------------------------------------
/*
This is the wakeHandler which clears the sleep counter. 
*/
//-------------------------------------------------------------------------------------------------------

void wakeHandler(void){
	NVIC_EN0_R &= ~0x00000001;
	IntEnable(INT_TIMER0A); //reenable the timer interrupt
	sleepCounter = 0;
	GPIOIntClear(GPIO_PORTA_BASE, GPIO_PIN_6);
	NVIC_EN0_R |= 0x00000001;
}





void Timer0A_Init(unsigned long period){   
  // Enable Peripheral Clocks 	
  SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
  TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC); 		// Configure for 32-bit timer mode
  TimerLoadSet(TIMER0_BASE, TIMER_A, period - 1);      	// Reload value
	IntPrioritySet(INT_TIMER0A, 0x00);  	 								// Set Timer0A interrupt priority as 0
  IntEnable(INT_TIMER0A);    														// Enable interrupt 19 in NVIC [Timer0A]
	TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);      // Arm timeout interrupt
  TimerEnable(TIMER0_BASE, TIMER_A);    						  	// Enable timer0A    
}

//-------------------------------------------------------------------------------------------------------
/*
The Timer0A Handler is below. This function will control the LED bar.
*/
//-------------------------------------------------------------------------------------------------------

void Timer0A_Handler (void){
	// acknowledge flag for Timer0A timeout
	TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
	
	if(invalidPW){
					GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5, HI);
	} else{
		if(!locked){
			counter++;					//increase the counter value
			counter %= 5;				//find the remainder	
			//sweep through the output LEDs
			GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5, 0x02 << counter);
		}else {
			GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5, LO);
			counter = 0;
		}
	}
}

//-------------------------------------------------------------------------------------------------------
/*
This funciton enables the interrupt.
*/
//-------------------------------------------------------------------------------------------------------

void intResetPass(void){
	//PC4 -> interrupt number	2,	PRI0,	EN0, 0x0000.0100
	
	NVIC_EN0_R |= 0x00000004;		//enable interrupt 2 -> 0b0000.0100
	NVIC_PRI0_R &= ~0x00C00000;	//PRIORITY 1 FOR PORT C
	
	//1 is arm interrupt
	GPIO_PORTC_IM_R	|= 0x10;		//ARM INTERRUPT FOR PC4
	
	//0 is edge sensitive, 1 is level sensitive
	GPIO_PORTC_IS_R &= ~0x10;		//MAKE PC4 EDGE SENSITIVE
	
	//0 is one edge, 1 is both edges
	GPIO_PORTC_IBE_R |= 0x10;	//MAKE PC4 SENSITIVE TO BOTH EDGES
	
////	//1 is rising edge, 0 is falling edge
////	GPIO_PORTD_IEV_R |= 0x10;		//MAKE PC4 SENSITIVE TO THE RISING EDGE
	
	IntMasterEnable();        	// globally enable interrupts
}

//-------------------------------------------------------------------------------------------------------
/*
This is the reset handler. This handler runs when the digital value is triggered.
*/
//-------------------------------------------------------------------------------------------------------

void Reset_handler(void){
	NVIC_EN0_R |= 0x00000004;		//enable interrupt 1 -> 0x0000.0010
	SysCtlDelay(53333);
	NVIC_EN0_R &= ~0x00000004;	//DISABLE INTERRUPT
	
	//clear the PC4 interrupt
	GPIOIntClear(GPIO_PORTC_BASE,GPIO_PIN_4);
	
	//if its pressed then clear the password and unlock the box
	if(GPIOPinRead(GPIO_PORTC_BASE,GPIO_PIN_4) == 0x10){
		//clear all of the password
		for(int i = 0; i < position + 1; i++){
			password[i] = (int) 0;
		}
		position = 0;
	} else{
		//when I let go I want to move the servo to 0 degrees and finishing the unlocking process
		PWMPulseWidthSet(PWM1_BASE, PWM_OUT_5,dutyCycle(0));
		locked = false;
		passSet = false;
	}
}






//-------------------------------------------------------------------------------------------------------
/*
In this clearEnterPw I clear the password that was entered.
*/
//-------------------------------------------------------------------------------------------------------

void clearEnterPw(void){
	for(int i = 0; (i < enterPosition) && (enterPosition == position); i++){
				enterPassword[i] = (int) 0;
				
	}
	enterPosition = 0;
}

//-------------------------------------------------------------------------------------------------------
/*
In this initClosedBox I initialize the interrupt for the switch that detects when the box is closed.
*/
//-------------------------------------------------------------------------------------------------------

void initClosedBox(void){
	//PB3 -- interrupt 1, EN0, PRI0, 0x08
	
	NVIC_EN0_R |= 0x00000002; //enable the interrupt
	NVIC_PRI0_R &= ~0x0000A000; //Set the priority to priority 2 for interrupt 1
	GPIO_PORTB_IM_R |= 0x08; //enable int
	GPIO_PORTB_IS_R &= ~0x08; //edge sensitive
	GPIO_PORTB_IBE_R &= ~0x08; //sensitive to one edge
	GPIO_PORTB_IEV_R |= 0x08; //only sensitive to the rising edge
	
}



//-------------------------------------------------------------------------------------------------------
/*
Box closed handler
*/
//-------------------------------------------------------------------------------------------------------

void closedBox(void){ //on PB3 Rising edge (close switch)
	// There is no need to debounce since its only purpose is to lock the box
	GPIOIntClear(GPIO_PORTB_BASE, GPIO_PIN_3);			//clear the interrupt
	if(passSet){ //if a password has already been set then lock the box
		PWMPulseWidthSet(PWM1_BASE, PWM_OUT_5,dutyCycle(90));
		locked = true;
		clearEnterPw();
	}
}





//-------------------------------------------------------------------------------------------------------
/*
This function initializes the PWM function.
*/
//-------------------------------------------------------------------------------------------------------
void initPWM(void){								//Initialize the PWM on PA7
	
    //Set the clock
   SysCtlClockSet(SYSCTL_SYSDIV_1 | SYSCTL_USE_OSC |   SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ);

   //Configure PWM Clock divide system clock by 64
   SysCtlPWMClockSet(SYSCTL_PWMDIV_64);

  
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF); // Enable the peripherals used by this program.
    SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM1);  // Module 1 covers the LED pins

    //Configure PF1,PF2,PF3 Pins as PWM
    GPIOPinConfigure(GPIO_PF1_M1PWM5);
    GPIOPinTypePWM(GPIO_PORTF_BASE, GPIO_PIN_1);

    //Configure PWM Options
    //PWM_GEN_2 Covers M1PWM4 and M1PWM5
    PWMGenConfigure(PWM1_BASE, PWM_GEN_2, PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC); 

    //Set the Period (expressed in clock ticks)
    PWMGenPeriodSet(PWM1_BASE, PWM_GEN_2, 5000);
    //PWMGenPeriodSet(PWM1_BASE, PWM_GEN_3, 5000);

    // Enable the PWM generator
    PWMGenEnable(PWM1_BASE, PWM_GEN_2);

    // Turn on the Output pins
    PWMOutputState(PWM1_BASE, PWM_OUT_5_BIT, true);
		
}

//-------------------------------------------------------------------------------------------------------
/*
This function takes the inputs of the pad and loads the values onto the password array
*/
//-------------------------------------------------------------------------------------------------------
void padInputs(uint32_t cycles){
	
	if(GPIOPinRead(GPIO_PORTD_BASE,GPIO_PIN_0) == 0x01){
		for(current = 0; current < 4; current++){
			GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5,0x04 << current);
			SysCtlDelay(cycles);																																	//DEBOUNCE
			if(GPIOPinRead(GPIO_PORTD_BASE,GPIO_PIN_0) == 0x01){
				password[position] = pad[current*4];
				while(GPIOPinRead(GPIO_PORTD_BASE,GPIO_PIN_0) == 0x01){
					SysCtlDelay(cycles);
				}
			}
		}
		position++;
		//Make all of the Digital outputs for the keypad high
		GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5, HI);
		SysCtlDelay(cycles);	//'DEBOUNCE'
	}else if(GPIOPinRead(GPIO_PORTD_BASE,GPIO_PIN_1) == 0x02){ 	//CHECK IF PD1 TRIGGERED THIS INTERRUPT
		for(current = 0; current < 4; current++){
			GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5,0x04 << current);
			SysCtlDelay(cycles);																																	//DEBOUNCE
			if(GPIOPinRead(GPIO_PORTD_BASE,GPIO_PIN_1) == 0x02){
				password[position] = pad[1 + current*4];
				while(GPIOPinRead(GPIO_PORTD_BASE,GPIO_PIN_1) == 0x02){
					SysCtlDelay(cycles);
				}
			}
		}
		position++;
		//Make all of the Digital outputs for the keypad high
		GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5, HI);
		SysCtlDelay(cycles);	//'DEBOUNCE'
	}else if(GPIOPinRead(GPIO_PORTD_BASE,GPIO_PIN_2) == 0x04){	//CHECK IF PD2 TRIGGERED THIS INTERRUPT
		for(current = 0; current < 4; current++){
			GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5,0x04 << current);
			SysCtlDelay(cycles);																																	//DEBOUNCE
			if(GPIOPinRead(GPIO_PORTD_BASE,GPIO_PIN_2) == 0x04){
				password[position] = pad[2 + current*4];
				while(GPIOPinRead(GPIO_PORTD_BASE,GPIO_PIN_2) == 0x04){
					SysCtlDelay(cycles);
				}
			}
		}
		position++;
		//Make all of the Digital outputs for the keypad high
		GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5, HI);
		SysCtlDelay(cycles);	//'DEBOUNCE'
	}else if(GPIOPinRead(GPIO_PORTD_BASE,GPIO_PIN_3) == 0x08){	//CHECK IF PD3 TRIGGERED THIS INTERRUPT
		for(current = 0; current < 4; current++){
			GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5,0x04 << current);
			SysCtlDelay(cycles);																																	//DEBOUNCE
			if(GPIOPinRead(GPIO_PORTD_BASE,GPIO_PIN_3) == 0x08){
				password[position] = pad[3 + current*4];
				while(GPIOPinRead(GPIO_PORTD_BASE,GPIO_PIN_3) == 0x08){
					SysCtlDelay(cycles);
				}
			}
		}
		position++;
		//Make all of the Digital outputs for the keypad high
		GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5, HI);
		SysCtlDelay(cycles);	//'DEBOUNCE'
	}
}

//-------------------------------------------------------------------------------------------------------
/*
				Enter the password
*/
//-------------------------------------------------------------------------------------------------------
void enterPadInputs(uint32_t cycles){
	
	if(GPIOPinRead(GPIO_PORTD_BASE,GPIO_PIN_0) == 0x01){
		for(current = 0; current < 4; current++){
			GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5,0x04 << current);
			SysCtlDelay(cycles);																																	//DEBOUNCE
			if(GPIOPinRead(GPIO_PORTD_BASE,GPIO_PIN_0) == 0x01){
				enterPassword[enterPosition] = pad[current*4];
				while(GPIOPinRead(GPIO_PORTD_BASE,GPIO_PIN_0) == 0x01){
					SysCtlDelay(cycles);
				}
			}
		}
		enterPosition++;
		//Make all of the Digital outputs for the keypad high
		GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5, HI);
		SysCtlDelay(cycles);	//'DEBOUNCE'
	}else if(GPIOPinRead(GPIO_PORTD_BASE,GPIO_PIN_1) == 0x02){ 	//CHECK IF PD1 TRIGGERED THIS INTERRUPT
		for(current = 0; current < 4; current++){
			GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5,0x04 << current);
			SysCtlDelay(cycles);																																	//DEBOUNCE
			if(GPIOPinRead(GPIO_PORTD_BASE,GPIO_PIN_1) == 0x02){
				enterPassword[enterPosition] = pad[1 + current*4];
				while(GPIOPinRead(GPIO_PORTD_BASE,GPIO_PIN_1) == 0x02){
					SysCtlDelay(cycles);
				}
			}
		}
		enterPosition++;
		//Make all of the Digital outputs for the keypad high
		GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5, HI);
		SysCtlDelay(cycles);	//'DEBOUNCE'
	}else if(GPIOPinRead(GPIO_PORTD_BASE,GPIO_PIN_2) == 0x04){	//CHECK IF PD2 TRIGGERED THIS INTERRUPT
		for(current = 0; current < 4; current++){
			GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5,0x04 << current);
			SysCtlDelay(cycles);																																	//DEBOUNCE
			if(GPIOPinRead(GPIO_PORTD_BASE,GPIO_PIN_2) == 0x04){
				enterPassword[enterPosition] = pad[2 + current*4];
				while(GPIOPinRead(GPIO_PORTD_BASE,GPIO_PIN_2) == 0x04){
					SysCtlDelay(cycles);
				}
			}
		}
		enterPosition++;
		//Make all of the Digital outputs for the keypad high
		GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5, HI);
		SysCtlDelay(cycles);	//'DEBOUNCE'
	}else if(GPIOPinRead(GPIO_PORTD_BASE,GPIO_PIN_3) == 0x08){	//CHECK IF PD3 TRIGGERED THIS INTERRUPT
		for(current = 0; current < 4; current++){
			GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5,0x04 << current);
			SysCtlDelay(cycles);																																	//DEBOUNCE
			if(GPIOPinRead(GPIO_PORTD_BASE,GPIO_PIN_3) == 0x08){
				enterPassword[enterPosition] = pad[3 + current*4];
				while(GPIOPinRead(GPIO_PORTD_BASE,GPIO_PIN_3) == 0x08){
					SysCtlDelay(cycles);
				}
			}
		}
		enterPosition++;
		//Make all of the Digital outputs for the keypad high
		GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5, HI);
		SysCtlDelay(cycles);	//'DEBOUNCE'
	}
}






//-------------------------------------------------------------------------------------------------------
/*
In this readInPassword function, I read the password that is set into the lockbox.
*/
//-------------------------------------------------------------------------------------------------------
void readInPassword(uint32_t cycles){
	while(!locked && !passSet){ //while the box isnt locked keep reading in the values
			padInputs(cycles); //set the password
			//check if the password ends in ## then stop grabbing inputs
			locked = ((password[position-1] == '#') && (password[position-2] == '#'))?true:false;
			passSet = locked?true:false;
		}
}


//-------------------------------------------------------------------------------------------------------
/*
In this checkInput function, I check that the password that is entered to unlock the box is correct.
*/
//-------------------------------------------------------------------------------------------------------
void checkInput(uint32_t cycles){
	//while the box is locked
		while(locked){
			enterPadInputs(cycles);
			
			sleepCounter++;
			
			int CorrectPW = 0;
			
			if(sleepCounter > 5000000){
				SysCtlDeepSleep();
				clearEnterPw();
				//Disable the timer interrupt
				IntDisable(INT_TIMER0A);
			}
			
			for(int i = 0; (i < position) && (i == CorrectPW); i++){
				if(password[i] == enterPassword[i]){
					CorrectPW++;
				}
			}
			locked = (CorrectPW == position)?false:true;
			CorrectPW = 0;
			
			//clear the Entered Password if the box is unlocked
			if(!locked || (enterPosition == position)){
				clearEnterPw();
				invalidPW = true;
				delayMS(1000);
				invalidPW = false;
			}
		}
}

//-------------------------------------------------------------------------------------------------------
/*
This is the main function.
*/
//-------------------------------------------------------------------------------------------------------

int main(void){
	
	unsigned long period = 8000000; //half second timing
	Timer0A_Init(period);
	initPWM(); 										//INITIALIZE PWM
	PortFunctionInit();						//INITIALIZE PORTS
	intResetPass();								//INITIALIZE RESET
	initClosedBox();							//INITIALIZE BOXCLOSED
	intWake();
	
	//MAKE ALL OF THE OUTPUTS ON PORTA FROM 2 TO 5 HIGH
	GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5, HI);
	
	//debounce time for everything
	uint32_t cycles = 53333;
	
	//JUST FOR TESTING PURPOSES JUST FOR TESTING PURPOSES JUST FOR TESTING PURPOSES JUST FOR TESTING PURPOSES JUST FOR TESTING PURPOSES JUST FOR TESTING PURPOSES
	
	password[0] = pad[0]; 
	password[1] = pad[1];
	
	
	PWMPulseWidthSet(PWM1_BASE, PWM_OUT_5,dutyCycle(90));
	delayMS(2000);
	PWMPulseWidthSet(PWM1_BASE, PWM_OUT_5,dutyCycle(0));
	delayMS(2000);
	
	//JUST FOR TESTING PURPOSES JUST FOR TESTING PURPOSES JUST FOR TESTING PURPOSES JUST FOR TESTING PURPOSES JUST FOR TESTING PURPOSES JUST FOR TESTING PURPOSES
	
	while(1){
		
		//Set the password
		readInPassword(cycles);
		
		if(locked){
		//After reading in the password, lock the box
			PWMPulseWidthSet(PWM1_BASE, PWM_OUT_5,dutyCycle(90));
			delayMS(2000);
		}
			
		//check if the correct password is entered
		checkInput(cycles);
		
		if(!locked){
			PWMPulseWidthSet(PWM1_BASE, PWM_OUT_5,dutyCycle(0));
			delayMS(2000);
			sleepCounter = 0;
			IntEnable(INT_TIMER0A);
		}
	}	
}
