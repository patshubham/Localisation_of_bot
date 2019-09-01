#include <avr/io.h>
#include <math.h>
#define max 200;		//max velocity of bot
#define baseVelocity 150;	//define base velocity for bot
float error;
float preError=0;	//previous error
float kp,kd,x,y;	//x and y are coordinate 
float tError=0;	//pid value 
float rVelocity=0, lVelocity=0;	//left wheel velocity and right wheel velocity
float reqAngle, presentAngle;	// required angle is angle between x axis and required point
int turn;	
#define PI 3.141592654


int main()
{
	DDRE |=	(1<<PE5);
	DDRB |=	(1<<PB6);
	
	TCCR1A |= (1<<COM1B1) | (1<<WGM11);	//TIMER 1 fast pwm NON inverting	ICR1
	TCCR1B |= (1<<WGM13) | (1<<WGM12);
	 
	TCCR3A |= (1<<COM3C1) | (1<<WGM31);	//TIMER 3 fast pwm non inverting	ICR3
	TCCR3B |= (1<<WGM33) | (1<<WGM32);
	
	ICR1 = 10000;
	ICR3 = 10000;
		
		
		
		
		
	while (1)
	{	 reqAngle = atan(y/x);
		reqAngle = (reqAngle * 180) / PI;
		tError=totError(reqAngle,presentAngle);
		lVelocity= baseVelocity+tError;
		rVelocity=baseVelocity-tError;
		
		OCR1B = (lVelocity*10000)/200;
		OCR3C = (rVelocity*10000)/200;
	
	}
	
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
 
 