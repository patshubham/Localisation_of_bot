#include<avr/io.h>
#include<util/delay.h>
#include<stdlib.h>
#include<string.h>
#include<avr/interrupt.h>
#include<avr/sfr_defs.h>
#include<math.h>
#define lcofangle 0
#define lcofdist 0
int countr=0,countl=0,right_channelA=0,left_channelA=0,right_channelB=0,left_channelB=0;
float x=0.0, y=0.0, theta=0.0, d_theta=0.0;
int init;
int a=0,b=0;
char snum[5];
char d;
int overFlow=0;
#define max 200;		//max velocity of bot
#define baseVelocity 150;	//define base velocity for bot
float error;
float preError=0;	//previous error
float kp,kd,x,y;	//x and y are coordinate
float tError=0;	//pid value
float totError(float,float);
float rVelocity=0, lVelocity=0;	//left wheel velocity and right wheel velocity
float reqAngle, presentAngle;	// required angle is angle between x axis and required point
int turn;
#define PI 3.141592654


// define some macros
//#define BAUDrate 9600
#define F_CPU 8000000UL                                 // define baud
#define ubrr 51


int main(void)
{
	DDRD &= ~(1<<PD5); //INPUT B		PIN D5
	DDRD &= ~(1<<PD2); //INPUT B		PIN D2

	
	TCCR2 |= (1<<CS22) | (1<<CS21);	//pre scaling of 1024
	TIMSK |= (1<<TOIE2);	//Timer overflow interrupt enable
	 EIMSK |= (1<<INT0);  //ENABLING INTERRUPT Right wheel INPUT A	PIN D0
	 EICRA |= (1<<ISC00) ; //calling interrupt at Logic change
	 
	 EIMSK |= (1<<INT3);  //ENABLING INTERRUPT Left wheel INPUT A	PIN D3tcn
	 EICRA |= (1<<ISC30) ; //calling interrupt at Logic change
	 sei(); //CALLING INTERRUPT VECTOR
	 
	 DDRE |=	(1<<PE5);
	 DDRB |=	(1<<PB6);
	 
	 TCCR1A |= (1<<COM1B1) | (1<<WGM11);	//TIMER 1 fast pwm NON inverting	ICR1
	 TCCR1B |= (1<<WGM13) | (1<<WGM12);
	 
	 TCCR3A |= (1<<COM3C1) | (1<<WGM31);	//TIMER 3 fast pwm non inverting	ICR3
	 TCCR3B |= (1<<WGM33) | (1<<WGM32);
	 
	 ICR1 = 10000;
	 ICR3 = 10000;
	 
	 
    /* Replace with your application code */
    while (1) 
    {
		 reqAngle = atan(y/x);
		 reqAngle = (reqAngle * 180) / PI;
		 tError=totError(reqAngle,presentAngle);
		 lVelocity= baseVelocity+tError;
		 rVelocity=baseVelocity-tError;
		 
		 OCR1B = (lVelocity*10000)/200;
		 OCR3C = (rVelocity*10000)/200;
		 
    }
}

ISR(TIMER2_OVF_vect)
{
	overFlow++;
	if (overFlow==3)
	{
		a= countl;b=countr;
		countr=0;
		countl=0;
		
		d_theta = (a-b)*lcofangle;                                                   // calculating d_theta
		
		x+=(a+b)*sin(d_theta/2)*lcofdist*sin(theta + (d_theta/2))/d_theta;           // incrementing in x
		
		y+=(a+b)*sin(d_theta/2)*lcofdist*cos(theta + (d_theta/2))/d_theta;			// incrementing in y
		
		theta+=d_theta;
		overFlow=0;
	}
}
ISR(INT1_vect)
{
	right_channelB =bit_is_set(PIND,2);
	right_channelA =bit_is_set(PIND,0);
	
	if(right_channelA==right_channelB)		//clockwise
	{
		countr++;
		
		
	}
	//anticlockwise
	else if(right_channelA!=right_channelB)
	{
		countr--;
		
		
	}
	

	// convert 123 to string [buf]
	/*itoa(countr, snum, 10);
	usart_transmit_string(snum);
	USART_Transmit("__");
	_delay_ms(1000);
	clearstring(snum);*/
}
ISR(INT0_vect)
{
	left_channelA=bit_is_set(PIND,3);
	left_channelB=bit_is_set(PIND,5);
	
	if(left_channelA==left_channelB)		//clockwise
	{
		countl++;
		
		
	}
	//anticlockwise
	else if(left_channelA!=left_channelB)
	{
		countl--;
		
		
	}
	

	
	/*itoa(countl, snum, 10);
	usart_transmit_string(snum);
	USART_Transmit("__");
	clearstring(snum);*/
	
}

float totError(float reqAngle,float presentAngle)
{
	float e;
	error=reqAngle-presentAngle;
	e=(kp*error)+(kd*(preError- error));
	preError=error;
	if(e>50)
	{
		e=50;
	}
	else if (e<-50)
	{
		e=-50;
	}
	return e;
}