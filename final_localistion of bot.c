#include<avr/io.h>
#include<util/delay.h>
#include<stdlib.h>
#include "USART_128.h"
#include<string.h>
#include<avr/interrupt.h>
#include<avr/sfr_defs.h>
#include<math.h>
#define lcofangle 0
#define lcofdist 0
int right_channelA=0,left_channelA=0,right_channelB=0,left_channelB=0;
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
#define F_CPU 8000000UL                                 // define baud
#define ubrr 51



volatile int countr=0,countl=0,pstater,laststater,pstatel,laststatel;
volatile int flag = 1;
int init;


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


int main(void)
{   USART_Init(51,0);
	DDRD &= ~(1<<PD5); //encoder1 B
	DDRD &= ~(1<<PD4);//encoder1 A   
	DDRD &= ~(1<<PD6);//encoder2 A
	DDRD &= ~(1<<PD7);//encoder2 B
	laststatel=bit_is_set(PIND,4);
	laststater=bit_is_set(PIND,6);
	TCCR2 |= (1<<CS22) | (1<<CS21);	//pre scaling of 1024
	TIMSK |= (1<<TOIE2);	//Timer overflow interrupt enable
	sei();
	DDRE |=	(1<<PE5);//pwm pin
	DDRB |=	(1<<PB6);//pwm pin
	
	TCCR1A |= (1<<COM1B1) | (1<<WGM11);	//TIMER 1 fast pwm NON inverting	ICR1
	TCCR1B |= (1<<WGM13) | (1<<WGM12);
	
	TCCR3A |= (1<<COM3C1) | (1<<WGM31);	//TIMER 3 fast pwm non inverting	ICR3
	TCCR3B |= (1<<WGM33) | (1<<WGM32);
	
	ICR1 = 10000;
	ICR3 = 10000;
	
	while (1)
	{
		
		pstatel=bit_is_set(PIND,4);
		//stateb=bit_is_set(PIND,5);
		if(laststatel!=pstatel)		//clockwise
		{
			if(!pstatel)
			{
				if(pstatel!=bit_is_set(PIND,5))
				countl++;
				else
				countl--;
				USART_TransmitNumber(countl,0);
				USART_Transmitchar(' ',0);
			}
			laststatel=pstatel;
		}
		pstater=bit_is_set(PIND,4);
		//stateb=bit_is_set(PIND,5);
		if(laststater!=pstater)		//clockwise
		{
			if(!pstater)
			{
				if(pstater!=bit_is_set(PIND,5))
				countr++;
				else
				countr--;
				USART_TransmitNumber(countr,0);
				USART_Transmitchar(' ',0);
			}
			laststater=pstater;
		}
		
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