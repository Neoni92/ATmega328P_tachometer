/*
 * Harjoitustyo.c
 *
 * 
 * 
 *
 * Ohjelma analogisen kierroslukumittarin prototyypin ohjaukseen. 
 */ 


#define F_CPU 16000000UL
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <stdlib.h>

// -----------------------------
// ------ DEFINET --------------

#define TIMER1_100MS_CNT	(6249)		// Vertailuarvo Timer1:lle CTC tilassa. Jakaja = 256
										// => vertailun t‰sm‰ys 6250 * 1/(16000000/256) = 100ms
#define MAX_STEPS			(600)		// moottorin askelien maksimim‰‰r‰
#define STEP_PULSE_LEN		(24)		// Yhden pulssin aika (Timer0 vertailuarvo): full stepping (24+1)*1/(16000000/1024) = 1,28ms/step
										// half stepping (7+1)*1/(16000000/1024) = 0,512 ms/halfstep = 1,024ms/step
#define MAX_STEP_SEQ		(3)			// askeleiden m‰‰r‰ askelsekvenssiss‰-1
#define MAX_FREQ			(200)		// tulosignaalin maksimitaajuus (200/100ms=2000Hz)
#define MIN_FREQ			(30)		// tulosignaalin minimitaajuus (30/100ms=300Hz)
#define K					(3306UL)	// kulmakerroin osoittimen uuden sijainnin laskemiseen
#define MIN_STEP			(5)			// moottorin askelien minimim‰‰r‰
#define ADC_LIMITS			(38)		// ADC muunnoksen eri raja-arvojen lukum‰‰r‰ potentiometrilt‰

// ------------------------------
// ----- MOOTTORIN SUUNNAT ------

typedef enum {
	M_CW  = 1,		// Myˆt‰p‰iv‰‰n
	M_CCW = -1		// Vastap‰iv‰‰n
} M_DIR;

// ---------------------------------------------
// -- Timer0 ja Timer1 kontrolliarvot ----------

typedef enum {
	TIMER_INIT  = 0,		// case 0 = timerin alustus
	TIMER_START = 1,		// case 1 = timerin aloitus
	TIMER_STOP  = 2			// case 2 = timerin lopetus
}TIMER_STATE;

// --------------------------------------------
// --------- MUUTTUJAT ------------------------

volatile uint8_t	gucStepSeqInd =			0;		// askelsekvenssin t‰m‰nhetkinen indeksi
volatile M_DIR		direction =				M_CW;	// moottorin suunta, M_CW tai M_CCW
volatile uint16_t	gu100msPulseCounter =	0;		// muuttujat pulssien laskuun viimeisimmille
volatile uint16_t	gu100msPulses =			0;		// 100ms jaksoille
volatile uint16_t	guStepCounter =			0;		// s‰ilytt‰‰ lukua otetuista askeleista moottoria ajettaessa
volatile int16_t	giNextStepPos =			0;		// moottorin viisarin uuden sijainnin arvo
volatile int16_t	giDeltaStepsPos =		0;		// moottorin otettavien askelten m‰‰r‰ (= uusi sijainti - nykyinen sijainti)
volatile uint16_t	guCurStepPos =			0;		// moottorin viisarin nykyinen sijainti
volatile uint16_t	guOffset =				19;		// askelten kompensaatioarvo, +10 astetta nollakohdasta

// ---------------------------------------------
// -------- AJOSEKVENSSI -----------------------

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
// -------- FUNKTIO ESITTELYT ---------
// ------------------------------------

uint16_t readADC();						// Potentiometrin arvon funktio
void Timer0(TIMER_STATE state);			// Timer0
void Timer1(TIMER_STATE state);			// Timer1
void TakeSteps(int16_t ushowManySteps);	// Askeleet: positiiviset arvot = CW, negatiiviset = CCW



// ------------- MAIN ALKAA ----------------------------------------------------------

int main(void)
{
	uint16_t uChangePos;					// ADC:n tulos
	uint8_t ucLimitInd;						// ADC taulukon raja-arvojen indeksi
	
	uint16_t auLimits[ADC_LIMITS] = {		// ADC muunnoksen raja-arvot taulukkoon auLimits
		22,		// [0] > 0,10V				// 4,78/1003 = 4,7657 mV
		44,		// [1]						// (4,78/1003)*44 = 0,2V
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
		418,	// [18] > 1,99V --- ALLE 2V ----
		643,	// [19] > 3,06V --- YLI 3V ---- 
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
	
	DDRD = (_BV(PD4) | _BV(PD5) | _BV(PD6) | _BV(PD7));		// pinnit PD4,PD5,PD6 ja PD7 outputiksi
	DDRB &= ~(_BV(PB0));									// PB0 inputiksi
	
	sei();													// keskeytykset k‰yttˆˆn
	Timer0(TIMER_INIT);										// alustetaan ajastimet
	Timer1(TIMER_INIT);
	Timer1(TIMER_START);									// k‰ynnistet‰‰n Timer1 		
	
	TakeSteps(-MAX_STEPS-1);								// ajetaan moottori 0 asentoon
	
	ADMUX |= (_BV(MUX1) | _BV(MUX0));						// ADC3 inputiksi (UNO #A3)
	ADMUX |= _BV(REFS0);									// Vref = 5V
	ADCSRA |= (_BV(ADPS2) | _BV(ADPS1) | _BV(ADPS0));		// jakaja 128: fadc = 16MHz/128=125kHz
	ADCSRA &= ~(_BV(ADATE));
	ADCSRB &= ~(_BV(ADTS2) | _BV(ADTS1) | _BV(ADTS0));
	
	while(1) {
		uChangePos = readADC();											// tehd‰‰n single AD muunnos
																		// ja luetaan tulos uChangePos muuttujaan
		
		for (ucLimitInd = 0; ucLimitInd < ADC_LIMITS; ucLimitInd++) {	// etsit‰‰n raja-arvo taulukosta
			if (uChangePos < auLimits[ucLimitInd]) break;				// kun raja-arvo lˆytyy, hyp‰t‰‰n pois
		}
		if (ucLimitInd < 18 || ucLimitInd > 19) {						// jos potentiometrin j‰nnite on alle 2V tai yli 3V
			guOffset = ucLimitInd;										// asetetaan guOffsetille arvoksi adc muunnostaulukosta lˆytyv‰ indeksiarvo
		}
		if (gu100msPulses < MIN_FREQ) {									// jos tulosignaalin taajuus < 300Hz asetetaan uusi sijainti nollaksi
			giNextStepPos = 0;
		}
		else {
			giNextStepPos = K*(gu100msPulses-30UL)/1000UL+guOffset;		// lasketaan uusi sijainti
		}		
		if (giNextStepPos > MAX_STEPS) {								// jos uusi sijainti yli sallitun, asetetaan sijainti maksimiin
			giNextStepPos = MAX_STEPS;		
		}
		else if (giNextStepPos < 0) {									// jos uusi sijainti negatiivinen, asetetaan sijainti nollaan
			giNextStepPos = 0;
		}
		
		giDeltaStepsPos = giNextStepPos - guCurStepPos;					// lasketaan otettavien askelten m‰‰r‰ (= uusi sijainti - nykyinen sijainti) 
		if (abs(giDeltaStepsPos) > MIN_STEP) {							// jos otettavien askelten m‰‰r‰ suurempi kuin minimi askelm‰‰r‰
			TakeSteps(giDeltaStepsPos);									// vied‰‰n otettavien askelten m‰‰r‰ TakeSteps funktioon
			guCurStepPos = guCurStepPos + giDeltaStepsPos;				// otetaan taas talteen t‰m‰nhetkinen sijainti
		}	
	}
}
// ------ MAIN PƒƒTTYY ----------------------------------------------------------

// ----------------------------------------------------
// ------ HIENOSƒƒT÷ POTENTIOMETRILLƒ -----------------
// funktio readADC kontrolloi hienos‰‰tˆ‰

uint16_t readADC()
{
	uint16_t uADCval;				// ADC muunnoksen arvo
	
	ADCSRA |= _BV(ADEN);			// sallitaan ADC
	ADCSRA |= _BV(ADSC);			// k‰ynnistet‰‰n muunnos asettamalla ADSC bitti ADCSRA:aan
	
	while (ADCSRA & _BV(ADSC));		// odotetaan kunnes ADC muunnos valmis ts. kunnes ADC bitti on resetoitunut
	
	uADCval = ADC;					// luetaan tulos
	ADCSRA &= ~(_BV(ADEN));			// pys‰ytet‰‰n ADC, ei tarvetta pit‰‰ sit‰ k‰ynniss‰
	
	return uADCval;					// palautetaan ADC:n tulos
}
// readADC p‰‰ttyy -------------------------------------------

// -------------------------------------------------
// --------- VIISARIN LIIKUTTAMINEN ----------------
// funktio takeSteps kontrolloi viisarin liikuttamista
// ushowManySteps on askelten lukum‰‰r‰ liikuttamiseen (positiivinen = CW, negatiivinen = CCW)

void TakeSteps(int16_t ushowManySteps)
{
	if (ushowManySteps > 0) {						// jos otettavien askelten m‰‰r‰ positiivinen, liikutettava suunta on myˆt‰p‰iv‰‰n
		direction = M_CW;
	}
	else {
		direction = M_CCW;							// jos taas negatiivinen, liikutettava suunta vastap‰iv‰‰n
		ushowManySteps = -ushowManySteps;
	}

	guStepCounter = 0;								// alustetaan askelten laskuri nollaksi

	Timer0(TIMER_START);							// Timer0 laskee nollasta yhden pulssin aikaa (STEP_PULSE_LEN = (24+1)*1/(16000000/1024) = 1,28ms/step)
	while(!(guStepCounter == ushowManySteps)) {		// keskeytysk‰sittelij‰‰ ISR(TIMER0_COMPA_vect) kutsutaan pulssin laskevalla reunalla
		;
	}
		
	Timer0(TIMER_STOP);								// pys‰ytet‰‰n timer0, ei tarvetta pit‰‰ k‰ynniss‰
}
// takeSteps p‰‰ttyy ------------------------

// ------------------------------------------
// --------- TIMER0 -------------------------

void Timer0(TIMER_STATE state)
{
	switch (state) {
		case TIMER_INIT:							// timer0:n tilan asetukset
			TCCR0A |= (_BV(WGM01));					// CTC tila (tila 2)
		break;
		
		case TIMER_START:							// laskurin aloitus
			OCR0A = STEP_PULSE_LEN;					// askelnopeus vakioarvoksi (=positiivinen pulssin aika)
			TCNT0 = 0x00;							// resetoidaan laskuri
			TIFR0  |= _BV(OCIE0A);					// nollataan mahdolliset odottavat IRQ:t
			TIMSK0 |= (_BV(OCIE0A));				// laitetaan timer0 vertailu A keskeytykset p‰‰lle, askellus suoritetaan funktiossa ISR(TIMER0_COMPA_vect)
			TCCR0B |= (_BV(CS00) | _BV(CS02));		// jakajana 1024, k‰ynnistet‰‰n timer0
		break;
		
		case TIMER_STOP:
		default:
			TCCR0B &= ~(_BV(CS00) | _BV(CS02));		// pys‰ytet‰‰n timer0
			TIMSK0 &= ~(_BV(OCIE0A));				// laitetaan timer0 keskeytykset pois p‰‰lt‰
		break;
	}
}
// Timer0 p‰‰ttyy ------------------------------

// ---------------------------------------------
// ---------- TIMER1 ---------------------------
void Timer1(TIMER_STATE state)
{
	switch (state) {
		case TIMER_INIT:							// timer1:n tilan asetukset
			TCCR1B |= (_BV(WGM12));					// CTC tila (tila 2)
			TCCR1B &= ~(_BV(ICES1));				// kaappaus pulssin laskevasta reunasta
		break;
		
		case TIMER_START:							// laskurin aloitus
			OCR1A = TIMER1_100MS_CNT;				// 100 ms rekisteriasetukseksi
			TCNT1 = 0x00;							// laskurin resetointi
			TIFR1 |= (_BV(ICF1)  | _BV(OCF1A));		// nollataan IRQ:t		
			TIMSK1 |= (_BV(ICIE1) | _BV(OCIE1A));	// input p‰‰lle			
			TCCR1B |= (_BV(CS12));					// timer1 p‰‰lle
		break;
		
		case TIMER_STOP:							// laskurin pys‰ytys
		default:
			TCCR1B &= ~(_BV(CS12));					// pys‰ytet‰‰n timer1
			TIMSK1 &= ~(_BV(ICIE1) | _BV(OCIE1A));	// timer1 keskeytykset pois p‰‰lt‰
		break;
	}
}
// Timer 1 p‰‰ttyy ----------------------------------

// --------------------------------------------------
// -------- ISR TIMER1 COMPARE ----------------------
// vertailu ISR:‰‰ kutsutaan vertailuarvon t‰sm‰tess‰
// timer0 toimii CTC tilassa, laskee nollasta OCR1A arvoon
ISR(TIMER1_COMPA_vect)
{
	gu100msPulses  = gu100msPulseCounter;	// 100 ms aika on kulunut, otetaan talteen laskettujen pulssien m‰‰r‰ ja nollataan pulssilaskuri
	gu100msPulseCounter = 0;				// gu100msPulses sis‰lt‰‰ ajan viimeisimmille 100ms pulsseille
}
// ISR TIMER1 COMPARE p‰‰ttyy -----------------------

// --------------------------------------------------
// ------- ISR TIMER0 COMPARE -----------------------
// t‰t‰ keskeytysk‰sittej‰‰ kutsutaan aina kun TCNT0 == OCR0A
ISR(TIMER0_COMPA_vect)
{
	PORTD = aStepSequence[gucStepSeqInd];	// vied‰‰n askelsekvenssin indeksi moottorille, kirjoitetaan portti D:n askelsekvenssi indeksi
	gucStepSeqInd += direction;				// seuraava askel sekvenssiss‰ oikeaan suuntaan
	gucStepSeqInd &= MAX_STEP_SEQ;			// pidet‰‰n sekvenssin indeksi v‰lill‰ 0-3
	guStepCounter++;						// lasketaan otettujen askelten m‰‰r‰
}
// ISR TIMER0 COMPARE p‰‰ttyy -----------------------

// --------------------------------------------------
// ------ ISR TIMER1 CAPTURE ------------------------
// laskee saatujen pulssien m‰‰r‰‰ 100ms aikajakson sis‰ll‰
// ISR keskeytysk‰sittelij‰‰ kutsutaan tulon pulssin laskevalla reunalla
ISR(TIMER1_CAPT_vect)
{
	ICR1 = 0x00;				// nollataan tulo rekisteri
	gu100msPulseCounter++;		// kasvatetaan 100ms pulssilaskuria
}
// ISR TIMER1 CAPTURE p‰‰ttyy -----------------------
