/*
School project: Signal generator tachometer

Program to drive a motor controlled tachometer which input is a signal generator.

*/ 


#define F_CPU 16000000UL
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <stdlib.h>

// -----------------------------
// ------ DEFINES --------------

#define TIMER1_100MS_CNT	(6249)	// Reference value for Timer1 in CTC mode. Divider = 256
									// Comparison accuracy =>  6250 * 1/(16000000/256) = 100ms
#define MAX_STEPS			(600)	// Maximum steps for motor
#define STEP_PULSE_LEN		(24)	// Time of one pulse (Timer0 reference value): full stepping (24+1)*1/(16000000/1024) = 1,28ms/step
									// Half stepping (7+1)*1/(16000000/1024) = 0,512 ms/halfstep = 1,024ms/step
#define MAX_STEP_SEQ		(3)		// Number of steps in a stepsequence-1
#define MAX_FREQ			(200)	// Input signal max frequency (200/100ms=2000Hz)
#define MIN_FREQ			(30)	// Input signal min frequency (30/100ms=300Hz)
#define K					(3306UL)// Slope to calculate the new location of the pointer
#define MIN_STEP			(5)		// Minimum steps of motor
#define ADC_LIMITS			(38)	// ADC conversion number of limits from potentiometer

// -------------------------------------
// ----- MOTOR DRIVING DIRECTIONS ------

typedef enum {
	M_CW  = 1,		// Clockwise
	M_CCW = -1		// Counter-clockwise
} M_DIR;

// ---------------------------------------------
// -- Timer0 and Timer1 control values ----------

typedef enum {
	TIMER_INIT  = 0,	// Case 0 = timer initialisation
	TIMER_START = 1,	// Case 1 = timer start 
	TIMER_STOP  = 2		// Case 2 = timer stop
}TIMER_STATE;

// --------------------------------------------
// --------- VARIABLES ------------------------

volatile uint8_t	gucStepSeqInd =			0;		// Current index of stepsequence
volatile M_DIR		direction =				M_CW;	// Direction of motor M_CW or M_CCW
volatile uint16_t	gu100msPulseCounter =	0;		// Variables for pulse counting of the last 100 ms periods
volatile uint16_t	gu100msPulses =			0;
volatile uint16_t	guStepCounter =			0;		// Keep track of the number of steps taken while driving the motor
volatile int16_t	giNextStepPos =			0;		// Tachometers pointer location new value
volatile int16_t	giDeltaStepsPos =		0;		// Number of steps to take (= new location - current location)
volatile uint16_t	guCurStepPos =			0;		// Current location of the pointer
volatile uint16_t	guOffset =				19;		// Steps compensation value, +10 degrees from point zero

// ---------------------------------------------
// -------- DRIVING SEQUENCE -----------------------

const uint8_t aStepSequence[] = {
	(_BV(PD4) | _BV(PD6)),	// A+ & B+
	(_BV(PD4) | _BV(PD7)),	// A+ & B-
	(_BV(PD5) | _BV(PD7)),	// A- & B-
	(_BV(PD5) | _BV(PD6))	// A- & B+
};

/*
	    [0]   [1]  [2]  [3]
	   +----+----+
	PD4          |____+____  A+
		         +----+----
	PD5_____+____|           A-

	   +----+         +----
	PD6     |____+____|      B+
		    +----+----+
	PD7 ____|         |____  B-
	------------------------> CW
    <------------------------ CCW
*/

// ------------------------------------
// -------- FUNCTIONS ---------
// ------------------------------------

uint16_t readADC();		// Function for potentiometer value
void Timer0(TIMER_STATE state);		// Timer0
void Timer1(TIMER_STATE state);		// Timer1
void TakeSteps(int16_t ushowManySteps);	// Steps: positive values = CW, negative values = CCW



// ------ MAIN STARTS ----

int main(void)
{
	uint16_t uChangePos;	// ADC result
	uint8_t ucLimitInd;		// ADC array limit-value index
	
	uint16_t auLimits[ADC_LIMITS] = {	// ADC conversion limits to array auLimits, 4,78/1003 = 4,7657 mV
		22,		// [0] > 0,10V
		44,		// [1]	(4,78/1003)*44 = 0,2V
		66,		// [2]
		88,		// [3]
		110,	// [4]
		132,	// [5]
		154,	// [6]
		176,	// [7]
		198,	// [8]
		220,	// [9]
		242,	// [10]
		264,	// [11]
		286,	// [12]
		308,	// [13]
		330,	// [14]
		352,	// [15]
		374,	// [16]
		396,	// [17]
		418,	// [18] > 1,99V --- UNDER 2V ----
		643,	// [19] > 3,06V --- OVER 3V ---- 
		663,	// [20] 
		683,	// [21]
		703,	// [22]
		723,	// [23]
		743,	// [24]
		763,	// [25]
		783,	// [26]
		803,	// [27]
		823,	// [28]
		843,	// [29]
		863,	// [30]
		883,	// [31]
		903,	// [32]
		923,	// [33]
		943,	// [34]
		963,	// [35]
		983,	// [36]
		1003	// [37]	> 4,78V
	};
	
	DDRD = (_BV(PD4) | _BV(PD5) | _BV(PD6) | _BV(PD7));	// Pins PD4,PD5,PD6 and PD7 as output
	DDRB &= ~(_BV(PB0));	// PB0 input
	
	sei();	// Interruptions
	Timer0(TIMER_INIT);		// Initialize timers
	Timer1(TIMER_INIT);
	Timer1(TIMER_START);	// Start Timer1 		
	
	TakeSteps(-MAX_STEPS-1);	// Drive the motor to 0 position
	
	ADMUX |= (_BV(MUX1) | _BV(MUX0));	// ADC3 as input (UNO #A3)
	ADMUX |= _BV(REFS0);	// Vref = 5V
	ADCSRA |= (_BV(ADPS2) | _BV(ADPS1) | _BV(ADPS0));	// Divider 128: fadc = 16MHz/128=125kHz
	ADCSRA &= ~(_BV(ADATE));
	ADCSRB &= ~(_BV(ADTS2) | _BV(ADTS1) | _BV(ADTS0));
	
	while(1) {
		uChangePos = readADC();	// Make a single ADC conversion and read it to uChangePos
		
		for (ucLimitInd = 0; ucLimitInd < ADC_LIMITS; ucLimitInd++) {	// Find the limit from the array
			if (uChangePos < auLimits[ucLimitInd]) break;	// When the limit is found, jump out
		}
		if (ucLimitInd < 18 || ucLimitInd > 19) {	// If the potentiometer voltage is under 2V or over 3V
			guOffset = ucLimitInd;					// set guOffset value = value found from the adc conversion array index value
		}
		if (gu100msPulses < MIN_FREQ) {	// If inputsignal frequency < 300Hz, set new location to zero
			giNextStepPos = 0;
		}
		else {
			giNextStepPos = K*(gu100msPulses-30UL)/1000UL+guOffset;	// Count the new location
		}		
		if (giNextStepPos > MAX_STEPS) {	// If new location over max, set it to max
			giNextStepPos = MAX_STEPS;		
		}
		else if (giNextStepPos < 0) {	// If new location is negative, set it to zero
			giNextStepPos = 0;
		}
		
		giDeltaStepsPos = giNextStepPos - guCurStepPos;	// Calculate the steps to take (= new location - current location) 
		if (abs(giDeltaStepsPos) > MIN_STEP) {	// If the steps to take is under min steps
			TakeSteps(giDeltaStepsPos);			// take the steps to take into TakeSteps function
			guCurStepPos = guCurStepPos + giDeltaStepsPos;	// Remember the current location
		}	
	}
}
// ------ MAIN ENDS -----------

// --------------------------------------------
// ------ FINETUNING WITH POTENTIOMETER -------
// function readADC controls the finetuning

uint16_t readADC()
{
	uint16_t uADCval;	// ADC conversion value
	
	ADCSRA |= _BV(ADEN);	// Enable ADC
	ADCSRA |= _BV(ADSC);	// Start the conversion by enabling ADSC bit to ADCSRA
	
	while (ADCSRA & _BV(ADSC));	// Wait till the conversion is complete, ie. till ADC bit is reseted
	
	uADCval = ADC;	// Read the value
	ADCSRA &= ~(_BV(ADEN));	// Stop the ADC, no need to keep it running
	
	return uADCval;	// Return the value
}
// readADC ENDS --------------------------------

// ---------------------------------------------
// --------- MOVING THE POINTER ----------------
// functionj takeSteps controls the pointer movement
// ushowManySteps is the number of steps to take (positive = CW, negative = CCW)

void TakeSteps(int16_t ushowManySteps)
{
	if (ushowManySteps > 0) {	// If the number of steps to take it positive, move clockwise
		direction = M_CW;
	}
	else {
		direction = M_CCW;	// If negative, move counter-clockwise
		ushowManySteps = -ushowManySteps;
	}

	guStepCounter = 0;	// Initialize the step counter to zero

	Timer0(TIMER_START);	// Timer0 calculates the time of one pulse (STEP_PULSE_LEN = (24+1)*1/(16000000/1024) = 1,28ms/step)
	while(!(guStepCounter == ushowManySteps)) {	// Interrupthandler ISR(TIMER0_COMPA_vect) is called at the falling edge
		;
	}
		
	Timer0(TIMER_STOP);	// Stop timer0, no need to keep running
}
// takeSteps ends ------------------------

// ------------------------------------------
// --------- TIMER0 -------------------------

void Timer0(TIMER_STATE state)
{
	switch (state) {
		case TIMER_INIT:	// Timer0 state properties
			TCCR0A |= (_BV(WGM01));	// CTC state (state 2)
		break;
		
		case TIMER_START:	// Start the counter
			OCR0A = STEP_PULSE_LEN;	// Stepping speed to constant value (=positive during the pulse)
			TCNT0 = 0x00;	// Reset the counter
			TIFR0  |= _BV(OCIE0A);	// Possible waiting IRQ's to zero
			TIMSK0 |= (_BV(OCIE0A));	// Put timer0 comparison A interrupts ON, stepping is done in function ISR(TIMER0_COMPA_vect)
			TCCR0B |= (_BV(CS00) | _BV(CS02));	// Divider 1024, start timer0
		break;
		
		case TIMER_STOP:
		default:
			TCCR0B &= ~(_BV(CS00) | _BV(CS02));	// Stop timer0
			TIMSK0 &= ~(_BV(OCIE0A));	// Timer0 interrups OFF
		break;
	}
}
// Timer0 ends ------------------------------

// ---------------------------------------------
// ---------- TIMER1 ---------------------------
void Timer1(TIMER_STATE state)
{
	switch (state) {
		case TIMER_INIT:	// Timer1 state properties
			TCCR1B |= (_BV(WGM12));	// CTC state (state 2)
			TCCR1B &= ~(_BV(ICES1));	// Catch from falling edge of pulse
		break;
		
		case TIMER_START:	// Start counter
			OCR1A = TIMER1_100MS_CNT;	// 100 ms as register setting
			TCNT1 = 0x00;	// Reset counter
			TIFR1 |= (_BV(ICF1)  | _BV(OCF1A));	// IRQ's to zero		
			TIMSK1 |= (_BV(ICIE1) | _BV(OCIE1A));	// Input ON			
			TCCR1B |= (_BV(CS12));	// Timer1 ON
		break;
		
		case TIMER_STOP:	// Stop counter
		default:
			TCCR1B &= ~(_BV(CS12));	// Stop timer1
			TIMSK1 &= ~(_BV(ICIE1) | _BV(OCIE1A));	// Timer1 interrupts OFF
		break;
	}
}
// Timer 1 ends ----------------------------------

// --------------------------------------------------
// -------- ISR TIMER1 COMPARE ----------------------
// comparison ISR is called when comparison value matches
// timer0 works in CTC state, counts from zero to OCR1A value
ISR(TIMER1_COMPA_vect)
{
	gu100msPulses  = gu100msPulseCounter;	// 100 ms time has passed, remember the counted pulses and zero the pulse counter
	gu100msPulseCounter = 0;	// gu100msPulses is the time of the last 100ms pulses
}
// ISR TIMER1 COMPARE ends -----------------------

// --------------------------------------------------
// ------- ISR TIMER0 COMPARE -----------------------
// this interrupthandler is called when TCNT0 == OCR0A
ISR(TIMER0_COMPA_vect)
{
	PORTD = aStepSequence[gucStepSeqInd];	// Take the stepsequence index to motor, write port D stepsequence index
	gucStepSeqInd += direction;	// Next step in sequence to correct direction
	gucStepSeqInd &= MAX_STEP_SEQ;	// Keep the sequence index between 0-3
	guStepCounter++;	// Count the steps taken
}
// ISR TIMER0 COMPARE ends -----------------------

// -----------------------------------------------
// ------ ISR TIMER1 CAPTURE ---------------------
// counts the given pulses inside the 100ms period
// ISR interrupthandler is called on input pulse falling edge
ISR(TIMER1_CAPT_vect)
{
	ICR1 = 0x00;	// Input-register to zero
	gu100msPulseCounter++;	// count up the 100ms pulsecounter
}
// ISR TIMER1 CAPTURE ends -----------------------
