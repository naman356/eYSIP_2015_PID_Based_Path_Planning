/************************************************************************************
Author : Dhirendra Sagar , Uttam Kumar Gupta

AVR Studio Version 6

Date: 25th June 2015

*********************************************************************************/

#define F_CPU 14745600
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#include <math.h> //included to support power function
#include "lcd.h"

void port_init();
void timer5_init();
void velocity(unsigned char, unsigned char);
void motors_delay();

unsigned char data;			//to store received data from UDR1
signed int speed=0 ;

//Function to configure ports to enable robot's motion
void motion_pin_config (void) 
{
	DDRA = DDRA | 0x0F;
	PORTA = PORTA & 0xF0;
	DDRL = DDRL | 0x18;   //Setting PL3 and PL4 pins as output for PWM generation
	PORTL = PORTL | 0x18; //PL3 and PL4 pins are for velocity control using PWM.
}

//Function to Initialize PORTS
void port_init()
{
	motion_pin_config();	
}

/*
  //Function Name -  Timer 5
  //Timer 5 initialized in PWM mode for velocity control
  // Prescale:256
  // PWM 8bit fast, TOP=0x00FF
  // Timer Frequency:225.000Hz
*/
void timer5_init()
{
	TCCR5B = 0x00;	//Stop
	TCNT5H = 0xFF;	//Counter higher 8-bit value to which OCR5xH value is compared with
	TCNT5L = 0x01;	//Counter lower 8-bit value to which OCR5xH value is compared with
	OCR5AH = 0x00;	//Output compare register high value for Left Motor
	OCR5AL = 0xFF;	//Output compare register low value for Left Motor
	OCR5BH = 0x00;	//Output compare register high value for Right Motor
	OCR5BL = 0xFF;	//Output compare register low value for Right Motor
	OCR5CH = 0x00;	//Output compare register high value for Motor C1
	OCR5CL = 0xFF;	//Output compare register low value for Motor C1
	TCCR5A = 0xA9;	/*{COM5A1=1, COM5A0=0; COM5B1=1, COM5B0=0; COM5C1=1 COM5C0=0}
 					  For Overriding normal port functionality to OCRnA outputs.
				  	  {WGM51=0, WGM50=1} Along With WGM52 in TCCR5B for Selecting FAST PWM 8-bit Mode*/

	TCCR5B = 0x0B;	//WGM12=1; CS12=0, CS11=1, CS10=1 (Prescaler=64)
}

/*
  //Function Name -  velocity
  //Logic - control velocity of motors
*/
void velocity (unsigned char left_motor, unsigned char right_motor)
{
	OCR5AL = (unsigned char)left_motor;
	OCR5BL = (unsigned char)right_motor;
}

/*
  //Function Name -  motion set
  //Logic - assign motion direction
*/
void motion_set (unsigned char Direction)
{
	unsigned char PortARestore = 0;

	Direction &= 0x0F; 		// removing upper nibbel for the protection
	PortARestore = PORTA; 		// reading the PORTA original status
	PortARestore &= 0xF0; 		// making lower direction nibbel to 0
	PortARestore |= Direction; // adding lower nibbel for forward command and restoring the PORTA status
	PORTA = PortARestore; 		// executing the command
}

void forward (void) //both wheels forward
{
	motion_set(0x06);
}

void back (void) //both wheels backward
{
	motion_set(0x09);
}

void left (void) //Left wheel backward, Right wheel forward
{
	motion_set(0x05);
}

void right (void) //Left wheel forward, Right wheel backward
{
	motion_set(0x0A);
}

void fast_left(void)
{
	motion_set(0x05);
}

void fast_right(void)
{
	
	motion_set(0x0A);
}

void soft_left (void) //Left wheel stationary, Right wheel forward
{
	motion_set(0x04);
}

void soft_right (void) //Left wheel forward, Right wheel is stationary
{
	motion_set(0x02);
}

void soft_left_2 (void) //Left wheel backward, right wheel stationary
{
	motion_set(0x01);
}

void soft_right_2 (void) //Left wheel stationary, Right wheel backward
{
	motion_set(0x08);
}


void stop (void) //hard stop
{
	motion_set(0x00);
}
//Function To Initialize UART0
// desired baud rate:9600
// actual baud rate:9600 (error 0.0%)
// char size: 8 bit
// parity: Disabled
void uart0_init(void)
{
	UCSR0B = 0x00; //disable while setting baud rate
	UCSR0A = 0x00;
	UCSR0C = 0x06;
	UBRR0L = 0x5F; //set baud rate lo
	UBRR0H = 0x00; //set baud rate hi
	UCSR0B = 0x98;
}

SIGNAL(SIG_USART0_RECV) 		// ISR for receive complete interrupt
{
	data = UDR0; 				//making copy of data from UDR0 in 'data' variable

	 				//echo data back to PC
	if(data < 0x78)
	{
		speed = 2*data + data%2;
		//lcd_print(2,1,value,3);
		//i++;
		UDR0 = data;
	}
	
	if(data == 0xf8) // To check for forward movement
	{
		forward();  //forward
		velocity(255,255);
	}
	
	else if(data == 0xf2) // To check for backward movement
	{
		back();
		velocity(255,255);
	}
	
	else if(data == 0xff) // To check for right movement
	{
		forward();  //forward
		velocity(255,speed);
	}
	
	else if(data == 0xfe) // To check for left movement
	{
		forward();  //forward
		velocity(speed,255);
	}
	
	else if (data == 0xfc) // move left at same point
	{
		left();
		velocity(255,255);
	}
		
	else if (data == 0xfb) // move left at same point
	{
		right();
		velocity(255,255);
	}
	
	else if(data == 0xfd)   // stop
	{
		PORTA=0x00; //stop
	}
	
	else   // nothing do
	{
	}
}

/*
  //Function Name -  init_devices
  //Logic - intitialization
*/
void init_devices (void)
{
 	cli(); //Clears the global interrupts
    port_init();
	timer5_init();
	uart0_init(); //Initailize UART1 for serial communiaction
	sei();   //Enables the global interrupts
}

/*
  //Function Name -  Main
  //Input - Nothing
  //Output - control left and right Motor speeds
  //Logic - add pid correction error here.
*/
int main()
{
	init_devices();
	lcd_set_4bit();
	lcd_init();
		
	while(1);			
}
						
				