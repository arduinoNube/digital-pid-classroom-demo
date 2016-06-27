//*****************************************************************************
//
// MSP432 main.c template - Empty main
//
//****************************************************************************

#include "msp.h"
#include <stdio.h>

void init();
void initPWM();
void initEncoder();
void timer0_0IsrHandler();
void timer0_NIsrHandler();
void timer1_0IsrHandler();
void setPWM(float);     // this will automatically scale it to the min and max values
void setDIR(uint8_t);   // 1 = positive, 0 = negative (relative to encoder count)
void initControlLoop();

volatile uint16_t uMax;
volatile uint16_t uMin;
volatile int16_t pos;

volatile int16_t pos_d;
volatile float error;
volatile float u;
volatile int Kp,Kd,Ki;
volatile float error_i,error_d,error_old;
volatile float dt;
volatile int control_type = 10;
volatile int antiWindup;
volatile int btnFlag = 0;

// To Do
// 1. Serial gain selection
// 2. Serial ID selection
// 3. Serial anti-windup selector
// 4. DONE Add hard stops so flag doesn't break (DO THIS FIRST)
// 5. Longer usb cable or power from main supply
// 6. DONE Make flag more visible
// 7. Set it up on a box
// 8. Lights in the room
// Examples
// 1. P Control
// 2. PI Control
// 3. PD Control
// 4. PID Control (No anti-windup)
// 5. Anti-windup
// 6. Sampling time and Zero Order Hold
// 7. Integration Error
// 8. Uneven sampling times
// 9.

void main(void)
{
    WDTCTL = WDTPW | WDTHOLD;           // Stop watchdog timer
	init();
	while(1)
	{
		if(btnFlag == 1)
		{
			int i;
			for(i=0; i<20000; i++) // Debouncing delay
			{
				if(1) btnFlag = 1;
			}
			btnFlag = 0;
		}
	}
}

void init()
{
	//printf("\n");
	// P1.0 is the LED
	// P1.4 is the switch
	P1DIR  = BIT0;  // Port 1.0 is an output;
	P1DIR &= ~BIT4; // Port 1.4 is an input;
	P1REN |=  BIT4; // Enabling pull up/down resistors on switch
	P1OUT &= ~BIT4; // Pulldown resistor selected
	P1OUT &= ~BIT0; // Pin 0 LOW by default

	// P2.4 is the PWM pin
	// P2.6 is the BRAKE pin
	// P2.7 is the DIRECTION pin
	P2DIR = BIT4 | BIT6 | BIT7;  // Port 2.4, 2.6, and 2.7 are all outputs to drive the motor
	P2OUT = 0x00;  // All pins LOW by default

	// P4.1 is channel A of the optical encoder
	// P4.2 is channel B of the optical encoder
	P4DIR &= ~BIT0 & ~BIT1; // Pins are inputs
	P4REN |=  BIT0 | BIT1;  // Pins have pull up/down resistors enabled
	P4OUT |=  BIT0 | BIT1;  // Pins have pull up resistors
	P4OUT  =  BIT0;  // LED ON by default

	// P5.5  is the pin connected to the button that changes the control system type when pressed
	P5DIR &= ~BIT5;   	// Pin is an input pin
	P5REN |=  BIT5;   	// Pin has pull up/down resistor enabled
	P5OUT |=  BIT5;   	// Pin has pull up resistor
	P5IES  = BIT5; 		// Falling edge interrupt
	NVIC_EnableIRQ(PORT5_IRQn); 	// Enabling the Port 5 interrupt
	NVIC_SetPriority(PORT5_IRQn,4); // Lowest priority interrupt
	P5IE   =  BIT5;  	// Enabling interrupt

	// Setting the clock speed to 48 MHz
	CSKEY = 0x695A;  	 // Unlocking the clock registers
	CSCTL0 = 0;      	 // reset DCO settings
	CSCTL0 = DCORSEL_5;  // Setting to 48 MHz
	CSCTL1 = SELA__REFOCLK | SELS__DCOCLK | SELM__DCOCLK; // ACLK = REFOCLK, SMCLK = MCLK = DCOCLK  (SMCLK is the one used by the PWM timers)
	CSKEY = 0;			 // Lock registers

	// Encoder Interrupts
	initEncoder();

	// Initialize PWM
	initPWM();

	// Initialize PWM
	initControlLoop();
}

void timer0_0IsrHandler()
{
	// Timer reached TA0CCR0 value
	TA0CCTL0 &= ~CCIFG;  // Clearing the interrupt flag
	TA0CTL   &= ~TAIFG;  // Clear TAIFG flag
	P2OUT    |= BIT4;    // Set the PWM pin HIGH
	TA0CCTL1 &= ~CCIFG;  // Clearing the interrupt flag
	TA0CCTL1 |= CCIE;    // Timer overflowed so reenabling the TA0CCTL1 counter interrupt
}

void timer0_NIsrHandler()
{
	// Reached PWM threshold value
	if(TA0CCTL1 & CCIFG)
	{
		TA0CCTL1 &= ~CCIFG; // Clearing the interrupt flag
		P2OUT    &= ~BIT4;	// Set PWM pin LOW
	}
}

void initPWM()
{
	// PWM Timer (TIMER0) setup
	NVIC_EnableIRQ(TA0_0_IRQn); // Enable interrupts
	NVIC_EnableIRQ(TA0_N_IRQn);

	NVIC_SetPriority(TA0_0_IRQn,2);
	NVIC_SetPriority(TA0_N_IRQn,2);

	// TA0CCTL0, configure the timer for SMCLK, no division, up mode, and clear it now.
	TA0CTL = TASSEL_2 | ID_0 | MC_1 | TACLR | TAIE;

	// PWM Frequency
	// TA0CCR0, interrupt value (max is 2^16);
	TA0CCR0 = 4800;  // 48Mhz/10kHz = 4800 counts to get 10KHz
	uMax    = TA0CCR0;  // This scales the range of values (and the resolution) of the PWM signal

	int factor = 1;
	char c = TA0CTL;
	c &= BIT6 | BIT7;

	switch(c)
	{
	case 0x00:
		factor = 1;
		break;

	case 0x40:
		factor = 2;
		break;

	case 0x80:
		factor = 4;
		break;

	case 0xc0:
		factor = 8;
		break;
	}
	uMin	= uMax*0.019375/(factor)*2; // For some reason, the timer goes nuts if there aren't enough steps

	// Duty Cytle
	//TA0CCR1 = uMax*0.5;  // Default 50% duty cycle
	TA0CCR1 = 0;
	// (240 clock counts, or 5us, are required to set the digital pin in the interrupt handler
	// so the realistic range is 240 to uMax-240)

	// Enable the timer interrupts
	TA0CCTL0 = CCIE;
	TA0CCTL1 = CCIE;
}

void setPWM(float val)
{
	// Rescaling so that it's always a percentage no matter the frequency/count# of TA0CCR0
	val = val*1.0/65535.0*uMax;
	//printf("\n%u\t",val);
	//printf("%f\t",v);
	//printf("%u\n",uMin);
	if(val < uMin) val = uMin;
	if(val > uMax) val = uMax;
	TA0CCR1 = val;
}

void setDIR(uint8_t val)
{
	if(val == 0)
	{
		P2OUT |= BIT7;
	}
	else
	{
		P2OUT &= ~BIT7;
	}
}

void initEncoder()
{
	// Setting the rising or falling edge bits based on if the pin is high or low at this mooment.
	P4IES = 0x00;
	if(P4IN & BIT1)
	{
		P4IES  |=  BIT1;
	}

	if(P4IN & BIT2)
	{
		P4IES  |=  BIT2;
	}

	P4IFG &= ~BIT1 & ~BIT2; // Clearing interrupt flags
	P4IE   =  BIT1 | BIT2;  // Enabling interrutps
	NVIC_EnableIRQ(PORT4_IRQn);
	NVIC_SetPriority(PORT4_IRQn,0);
	pos = 0;
}

void encoderISR()
{
	short val = 0;
	val = P4IN;
	if(P4IFG & BIT1)  // A
	{
		if(P4IES & BIT1) // Falling edge
		{
			if(val & BIT2) // B is HIGH
			{
				pos++;
			}
			else
			{
				pos--;
			}
		}
		else // Rising edge
		{
			if(val & BIT2) // B is HIGH
			{
				pos--;
			}
			else
			{
				pos++;
			}
		}
		P4IES = P4IES ^ BIT1;
	}
	if(P4IFG & BIT2)  // B
	{
		if(P4IES & BIT2) // Falling edge
		{
			if(val & BIT1) // A is HIGH
			{
				pos--;
			}
			else
			{
				pos++;
			}
		}
		else // Rising edge
		{
			if(val & BIT1) // A is HIGH
			{
				pos++;
			}
			else
			{
				pos--;
			}
		}
		P4IES = P4IES ^ BIT2;
	}
	if(pos >  2048)  pos = pos - 4096;
	if(pos < -2048)  pos = pos + 4096;
	P4IFG = 0;
	//printf("Encoder moved!\n");
}

void buttonISR()
{
	if((P5IFG & BIT5) && btnFlag == 0) // P5.5 interrupt was triggered and the debouncing loop has finished
	{
		TA1CCTL0 &= ~CCIE;  // Disabling the control loop timer
		//NVIC_DisableIRQ(TA1_0_IRQn);      // disable the control loop timer interrupts
		control_type++;
		if(control_type > 10)
		{
			control_type = 0;
		}
		initControlLoop();
		btnFlag = 1;
	}
//	if(P1OUT & BIT0)
//	{
//		P1OUT &= ~BIT0;  // LED Toggle
//	}
//	else
//	{
//		P1OUT |= BIT0; // LED Toggle
//	}
	//printf("%s","Button ISR");
	P5IFG &= ~BIT5; // Clearing interrupt flag
}

void initControlLoop()
{
	u     		= 0;
	pos_d 		= 0;
	error 		= 0;
	error_i 	= 0;
	error_d 	= 0;
	error_old 	= 0;

	NVIC_EnableIRQ(TA1_0_IRQn);      // Enable interrupts
	NVIC_SetPriority(TA1_0_IRQn,3);  // Second highest priority
	P1OUT &= ~BIT0;  // LED OFF
	switch(control_type)
	{
	case 0:  // P low gain (low stiffness, doesn't oscillate)
		Kp = 200;
		Ki = 0;
		Kd = 0;
		antiWindup = 0;
		// TA1CCTL0, configure the timer for SMCLK, /8, up mode, and clear it now.
		TA1CTL = TASSEL_2 | ID_3 | MC_1 | TACLR | TAIE;
		dt = 0.01;
		//printf("%s%d\n\n","Proportional Control, Kp = ",Kp);
		break;
	case 1:  // P best gain (good compromise, used in the next 3 cases)
		Kp = 500;
		Ki = 0;
		Kd = 0;
		antiWindup = 0;
		// TA1CCTL0, configure the timer for SMCLK, /8, up mode, and clear it now.
		TA1CTL = TASSEL_2 | ID_3 | MC_1 | TACLR | TAIE;
		dt = 0.01;
		//printf("%s%d\n","Proportional Control, Kp = ",Kp);
		//printf("%s\n\n","(control loop running at 100 Hz)");
		break;
	case 2:  // P high gain (high stiffness, high oscillations)
		Kp = 1000;
		Ki = 0;
		Kd = 0;
		antiWindup = 0;
		// TA1CCTL0, configure the timer for SMCLK, /8, up mode, and clear it now.
		TA1CTL = TASSEL_2 | ID_3 | MC_1 | TACLR | TAIE;
		dt = 0.01;
		//printf("%s%d\n\n","Proportional Control, Kp = ",Kp);
		break;


	case 3:  // PD
		Kp = 500;
		Ki = 0;
		Kd = 25;
		antiWindup = 0;
		// TA1CCTL0, configure the timer for SMCLK, /8, up mode, and clear it now.
		TA1CTL = TASSEL_2 | ID_3 | MC_1 | TACLR | TAIE;
		dt = 0.01;
		//printf("%s%d\n","Proportional Control, Kp = ",Kp);
		//printf("%s%d\n","Derivative Control,   Kd = ",Kd);
		//printf("%s\n\n","(control loop running at 100 Hz)");
		break;

	case 4:  // PI
		Kp = 400;
		Ki = 400;
		Kd = 0;
		antiWindup = 0;
		// TA1CCTL0, configure the timer for SMCLK, /8, up mode, and clear it now.
		TA1CTL = TASSEL_2 | ID_3 | MC_1 | TACLR | TAIE;
		dt = 0.01;
		//printf("%s%d\n","Proportional Control, Kp = ",Kp);
		//printf("%s%d\n","Integral Control,     Ki = ",Ki);
		//printf("%s\n\n","(No antiwindup, control loop running at 100 Hz)");
		break;

	case 5:  // PID (ok gains, no anti-windup)
		Kp = 500;
		Ki = 375;
		Kd = 25;
		antiWindup = 0;
		// TA1CCTL0, configure the timer for SMCLK, /8, up mode, and clear it now.
		TA1CTL = TASSEL_2 | ID_3 | MC_1 | TACLR | TAIE;
		dt = 0.01;
		//printf("%s%d\n","Proportional Control, Kp = ",Kp);
		//printf("%s%d\n","Derivative Control,   Kd = ",Kd);
		//printf("%s%d\n","Integral Control,     Ki = ",Ki);
		//printf("%s\n\n","(No antiwindup, control loop running at 100 Hz)");
		break;

	case 6:  // PID with anti-windup
		Kp = 500;
		Ki = 375;
		Kd = 25;
		antiWindup = 1;
		// TA1CCTL0, configure the timer for SMCLK, /8, up mode, and clear it now.
		TA1CTL = TASSEL_2 | ID_3 | MC_1 | TACLR | TAIE;
		dt = 0.01;
		//printf("%s%d\n",  "Proportional Control, Kp = ",Kp);
		//printf("%s%d\n","Derivative Control,   Kd = ",Kd);
		//printf("%s%d\n","Integral Control,     Ki = ",Ki);
		//printf("%s\n\n",  "(Using antiwindup, control loop running at 100 Hz)");
		break;

	case 7:  // PID super high gains (unstable because slow control loop 100 Hz)
		Kp 	  		= 10000;
		Ki    		= 6000;
		Kd	  		= 350;
		antiWindup = 1;
		// TA1CCTL0, configure the timer for SMCLK, /8, up mode, and clear it now.
		TA1CTL = TASSEL_2 | ID_3 | MC_1 | TACLR | TAIE;
		dt = 0.01;
		//printf("%s%d\n","Proportional Control, Kp = ",Kp);
		//printf("%s%d\n","Derivative Control,   Kd = ",Kd);
		//printf("%s%d\n","Integral Control,     Ki = ",Ki);
		//printf("%s\n\n","(Using antiwindup, control loop running at 100 Hz)");
		break;

	case 8:  // PID super high gains (more stable because control loop a little faster 400 Hz)
		Kp 	  		= 10000;
		Ki    		= 6000;
		Kd	  		= 350;
		antiWindup = 1;
		// TA1CCTL0, configure the timer for SMCLK, /2, up mode, and clear it now.
		TA1CTL = TASSEL_2 | ID_2 | MC_1 | TACLR | TAIE;
		dt = 0.0025;
		//printf("%s%d\n","Proportional Control, Kp = ",Kp);
		//printf("%s%d\n","Derivative Control,   Kd = ",Kd);
		//printf("%s%d\n","Integral Control,     Ki = ",Ki);
		//printf("%s\n\n","(Using antiwindup, control loop running at 400 Hz)");
		break;

	case 9:  // PID super high gains (stable because control loop is much faster 800 Hz)
		Kp 	  		= 10000;
		Ki    		= 6000;
		Kd	  		= 350;
		antiWindup = 1;
		// TA1CCTL0, configure the timer for SMCLK, /0, up mode, and clear it now.
		TA1CTL = TASSEL_2 | ID_0 | MC_1 | TACLR | TAIE;
		dt = 0.00125;
		//printf("%s%d\n","Proportional Control, Kp = ",Kp);
		//printf("%s%d\n","Derivative Control,   Kd = ",Kd);
		//printf("%s%d\n","Integral Control,     Ki = ",Ki);
		//printf("%s\n\n","(Using antiwindup, control loop running at 800 Hz)");
		break;

	default:
		// Turning the PWM signal off;
		setPWM(0);

		// control loop off, LED on
		P1OUT |= BIT0;

		// Disable the timer interrupts
		TA1CCTL0 |= !CCIE;

		//printf("%s\n","Controller off.  Press and hold the button to cycle through the different control schemes.");
		break;
	}

	// Set the timer count value
	TA1CCR0 = 48000;  // 48Mhz/4/60kHz = 200 hz control loop (8 is the clock divider ID_0,1,2,3, etc.)

	if(control_type <= 9)
	{
		// Enable the timer interrupts
		TA1CCTL0 = CCIE;
	}

	//printf("%s%d\n","Pos=",pos);
	//printf("%s%d\n","Type=",control_type);

}

void timer1_0IsrHandler()
{
	//printf("%x\t",P1OUT);
	TA1CCTL0 &= ~CCIFG;  // Clearing the interrupt flag
	TA1CTL   &= ~TAIFG;  // Clear TAIFG flag

	// Control Logic
	error = pos_d - pos;

	error_d = (error-error_old)/dt;

	u = Kp*error + Ki*error_i + Kd*error_d;

	// Anti-windup
	if(abs(u) >= 65535 && (((error >= 0) && (error_i >= 0)) || ((error < 0) && (error_i < 0))))
	{
		if(antiWindup)
		{
			error_i = error_i;
		}
		else  // If no antiwindup
		{
			error_i = error_i + dt*1*error;  // rectangular integration
		}
		//P1OUT &= ~BIT0;
	}
	else
	{
		error_i = error_i + dt*1*error;  // rectangular integration
		//P1OUT |= BIT0;
	}
	error_old = error;
	//printf("%i\t",error);
	//printf("%i\n",u);

	if(u>=0)
	{
		setDIR(0);
		setPWM(abs(u));
	}
	else
	{
		setDIR(1);
		setPWM(abs(u));
	}

	if(abs(u) < 65535)
	{
		//P1OUT &= ~BIT0;
	}
	else
	{
		//P1OUT |= BIT0;
	}
	//printf("%x\n",P1OUT);
}
