#define F_CPU 16000000UL


#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdlib.h>
#define DEBUG   // enables UART output (send_reading etc.)
#include <string.h>
#include <util/delay.h>

// Constant for Timer1 50 Hz PWM (ICR mode)
#define PWM_TOP 2500

// Converter of [0;255] range of x to [0,PWM_top] range for OCR of Timer1 (16 bit)
#define D1B(x) (uint16_t)(((x)*(uint32_t)(PWM_top))>>8)


// #define BAUD 57600UL
#define BAUD 9600UL
#define UBRR ((F_CPU)/((BAUD)*(16UL))-1)

#define SCL_CLOCK 100000L

const uint16_t Servo_angle [256]={ //  +/-90 degrees
85,85,85,85,85,85,85,85,85,85,85,85,85,85,85,85, // 0...15
108,108,108,108,108,108,108,108,108,108,108,108,108,108,108,108, // 16...31 0.9ms pulse(108) 
119,119,119,119,119,119,119,119,119,119,119,119,119,119,119,119, // 32...47
131,131,131,131,131,131,131,131,131,131,131,131,131,131,131,131, // 48...63
142,142,142,142,142,142,142,142,142,142,142,142,142,142,142,142, // 64...79
154,154,154,154,154,154,154,154,154,154,154,154,154,154,154,154, // 80...95
165,165,165,165,165,165,165,165,165,165,165,165,165,165,165,165, // 96...111
177,177,177,177,177,177,177,177,177,177,177,177,188,188,188,188, // center - 1.5ms pulse (188) 112...127
188,188,188,188,198,198,198,198,198,198,198,198,198,198,198,198, // 128...143
208,208,208,208,208,208,208,208,208,208,208,208,208,208,208,208, // 143...159
218,218,218,218,218,218,218,218,218,218,218,218,218,218,218,218, // 160...175
228,228,228,228,228,228,228,228,228,228,228,228,228,228,228,228, // 175...191
238,238,238,238,238,238,238,238,238,238,238,238,238,238,238,238, // 192...207
248,248,248,248,248,248,248,248,248,248,248,248,248,248,248,248, // 208...223
270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270, // 224...239
290,290,290,290,290,290,290,290,290,290,290,290,290,290,290,290  // 240...255 
};


void gpio_init() {
  cli();
  DDRB=((1<<PB3)|(1<<PB2)|(1<<PB1)); // PB3-PB1 are set as outputs for PWM
  PORTB=((1<<PB5)|(1<<PB4)|(1<<PB3)|(1<<PB0)); //enablet pull-up resistors on PB5, PB4 and PB0; PB3-HI (D3 OFF), PB2 and PB1 are set to LOW
    
  DDRC=0; // all pins are inputs. Not really needed as 0 is default value. 
  PORTC=((1<<PC4)|(1<<PC4)); // temporary pulling up TWI pins. The pins' function will be overriden once TWI is enabled.
  DIDR0=((1<<PC3)|(1<<PC2)|(1<<PC1)|(1<<PC0)); // disable digital inputs on analog pins.

  DDRD=(1<<PD7)|(1<<PD6)|(1<<PD5)|(1<<PD4)|(1<<PD1); //Set PD1 (TX) and PD7-PD4 (power control) as outputs.
  PORTD=((1<<PD1)|(1<<PD2)|(1<<PD3)); // Set PD1(TX) to HIGH (idle) and enable pull-ups on PD3 and PD2.
  sei(); 
} //end gpio_init


void uart_tx_init() {
  UBRR0H = (uint8_t)((UBRR)>>8); // Set the UART speed as defined by UBRR
  UBRR0L = (uint8_t)UBRR;
  UCSR0B|=(1<<TXCIE0)|(1<<TXEN0); //(1<<UDRIE0) Enable TX and TX IRQ.
  UCSR0C=(3<<UCSZ00); // Asynchronous UART, 8-N-1
}// end UART init

void uart_init() { // TX and RX init with IRQ
  UBRR0H = (uint8_t)((UBRR)>>8); // Set the UART speed as defined by UBRR
  UBRR0L = (uint8_t)UBRR;
  UCSR0B|=(1<<TXCIE0)|(1<<TXEN0); // Enable TX and TX IRQ.
  UCSR0B|=(1<<RXCIE0)|(1<<RXEN0); // Enable RX and RX IRQ 
  UCSR0C=(3<<UCSZ00); // Asynchronous UART, 8-N-1
}// end UART init

// ======= PWM0 and PWM1 control (16-bit timer1) ===================
void timer1_50Hz_init(uint8_t en_IRQ) { //en_IRQ eanbles 
  TCCR1A|=(1<<COM1A1)|(1<<COM1B1); // non-inv PWM on channels A and B
  TCCR1B|=(1<<WGM13);  //PWM, Phase and Frequency Correct. TOP=ICR1.
  ICR1=PWM_TOP; //50Hz PWM
  OCR1A=Servo_angle[127]; 
  OCR1B=0; 
  TCCR1B|=((1<<CS11)|(1<<CS10)); //timer prescaler
  if (en_IRQ) TIMSK1|=(1<<ICIE1); // enable Input Capture Interrupt. NOTE: the ISR MUST be defined!!! 
}

void timer0_init () { 
// ======= PWM2 and D4 control (8-bit timer0) ===================
  TCCR0A|=(1<<COM0A1)|(1<<COM0B1); //Clear on Compare Match when up-counting. Set on Compare Match when down-counting.  
  TCCR0A|=(1<<WGM00);  //PWM, Phase Correct            
  OCR0A=0; 
  OCR0B=0; 
  TCCR0B|=((1<<CS01)|(1<<CS00)); 
}

void adc_init (uint8_t channel, uint8_t en_IRQ) {
// ADC init
  ADMUX=((1<<ADLAR)|(channel&0x0F)); // "left-aligned" result for easy 8-bit reading. 
  												// AVcc as Aref |(1<<REFS0)
  												// Sets ADC to the specified channel. Can be changed later.
  ADCSRA=(1<<ADEN); 
  if (en_IRQ) ADCSRA|=(1<<ADIE); // enable ADC Complete Interrupt. NOTE: the ISR MUST be defined!!!
  ADCSRA|=(1<<ADPS0)|(1<<ADPS1)|(1<<ADPS2); // ADC clock prescaler
  ADCSRA|=(1<<ADATE); // Continuosly running mode
  ADCSRA|=(1<<ADSC); // Start ADC
}

void twi_init(){
// TWI init
TWSR=0; // no prescaler
TWBR=(uint8_t)(((F_CPU/SCL_CLOCK)-16)>>1);  //setting SCL; must be >10 for stable operation
}

#define BAT_min 108
//Vbatt min~=13.5V (ADC=108)
#define BAT_warn 133
//Vbatt warn~=14V (ADC=133)

// Averaging filter
// If you use an ultrasonic sensor (US) in analog mode, this value MUST be set to 1 as per the sensor's manufacturer application notes.
// If you wish to filter the values from US sensor, you should use median or mode filter.
// If you use an IR sensor, you can set it to a reasonable value, something between 4 and 10 should work well.
#define ADC_sample_max 4

// Distance threshold for IR sensor. Change it for the US one.
// Note that IR and US sensors have different distance-voltage curves.
// You can use the "ADC_DEBUG" option (see below) to get the corresponding ADC reading.
#define DIST_TH 76
// 1.5/5*255

// Threshold for the distance readings variation
#define DELTA 5

//================== IR sensor settings (TA#1) ==================================
// ADC channel the IR sensor is plugged into (white 3-pin header). Change to ADC_data.ADC0...ADC7 as needed.
#define IR_ADC   ADC_data.ADC6

// Vref set with RV1 (measured on AREF with the DMM), in mV. PUT YOUR MEASURED VALUE HERE.
// Must be above the sensor's output at d1 (~1.6 V), otherwise ADC_D1 > 255.
#define VREF_mV  2500UL

#define D1_cm 16
#define D2_cm 49

// GP2Y0A21 curve (datasheet Fig. "Analog output voltage vs distance") approximated by
//   V = 32.7 V*cm / (d + 4.2 cm)    (fits 10 cm->2.3 V, 30 cm->0.95 V, 80 cm->0.4 V)
// With 8-bit left-aligned ADC: ADC = V*256/Vref  =>  ADC = IR_K/(10*d + 42), d in cm
// and the inverse:  d[mm] = IR_K/ADC - 42.   Integer only, no floats.
#define IR_K          (327000UL*256UL/VREF_mV)
#define IR_ADC_AT(d)  ((uint8_t)(IR_K/(10UL*(d)+42UL)))

// LED thresholds. If your calibration (table 1) differs from the datasheet,
// just replace these with the ADC values you measured at d1 and d2.
#define ADC_D1   IR_ADC_AT(D1_cm)   // ~166 @ Vref=2.5V
#define ADC_D2   IR_ADC_AT(D2_cm)   // ~63  @ Vref=2.5V
#define ADC_80   IR_ADC_AT(80)      // below this: beyond sensor range

// LED L half period: 1.5 s period -> toggle every 750 ms
#define L_HALF_ms 750
//================================================================================

//***************************************************************************************************************
//un-comment the line below to enable printing of ADC readings through the serial port
//#define ADC_DEBUG
// COM port settings: 9600, 8-N-1, None
// Note 1: for this connection, the flow control must be set to "None".
// Note 2: Use any terminal software to display the data (e.g. Hyperterminal)
// Note 3: Arduino IDE must be closed before launching the terminal, 
// and the terminal must be closed before launching Arduino IDE. I.e. only one software should access the serial port.
// Note 4: Since unconnected ADC inputs are floating, the displayed values of unconnected channels are random.
//***************************************************************************************************************

// custom delay function that uses Timer1 IRQ. 20ms increments.
#define DELAY_20ms(x)   TIMSK1&=~(1<<ICIE1);\
                      delay_ms=0;\
                      TIMSK1|=(1<<ICIE1);\
                      while (delay_ms<=x)

#define BLINK(x) PORTB^=(1<<PB3);\
 				_delay_ms(x)

static volatile uint8_t RX_buff, servo_idx, ADC_sample, V_batt;
static volatile uint16_t time, delay_ms, ADC_acc; 
static volatile uint8_t L_flash; // 1 = obstacle outside [d1;d2] -> flash LED L (kept out of the bit-field to avoid RMW races with the ISR)

volatile struct {
  uint8_t TX_finished:1;
  uint8_t sample:1;
  uint8_t mode:1;
  uint8_t stop:1;
//  uint8_t ADC_ready:1;
  uint8_t T1_ovf0:2;
  uint8_t T1_ovf1:2;
} flags;

extern const uint16_t Servo_angle [256];

ISR(__vector_default) { //capture it all
}

ISR(TIMER1_CAPT_vect){ //system tick: 50Hz, 20ms
  static uint16_t L_ms;
  time++;
  delay_ms+=20;
  flags.sample=1; // starts IMU sampling
  flags.T1_ovf0++;
  flags.T1_ovf1++;

  // LED L flashing, T=1.5 s (toggles every 750 ms; 20 ms ticks -> 740/760 ms alternating, 1.5 s on average)
  if (L_flash) {
    L_ms+=20;
    if (L_ms>=L_HALF_ms) {
      L_ms-=L_HALF_ms;
      PORTB^=(1<<PB5);
    }
  } else {
    L_ms=0;
    PORTB&=~(1<<PB5); // L off while the obstacle is inside [d1;d2]
  }
}


#ifdef DEBUG
static const char CRLF[3]={13, 10};
static volatile uint8_t *msg, TX_buffer1[20], TX_buffer2[10];

//=============================================================
// UART TX ISR
// it loads UDR as long as the "msg" contains non-zero characters
// once it finds a zero character, it sets TX_finished flag, so a new process can start transmission
ISR (USART_TX_vect) {
  msg++;
  if (*msg) UDR0=*msg;
  else flags.TX_finished=1;
}

//========================================================================
// 
void send_int(int16_t data, uint8_t base, uint8_t crlf){
	while (!flags.TX_finished); //waiting for other transmission to complete
	flags.TX_finished=0;
	itoa(data, (char*) &TX_buffer1[0], base);
	if (crlf) strcat((char*)TX_buffer1, CRLF);
	msg=TX_buffer1;
	UDR0=*msg;
}

void send_reading (int16_t value, char label[], uint8_t crlf) {
	while (!flags.TX_finished); //waiting for other transmission to complete
	flags.TX_finished=0;
	strcpy ((char*)TX_buffer1, label);
	itoa(value, (char*)&TX_buffer2[0], 10);
	strcat((char*)TX_buffer1, (char*)TX_buffer2);
	if (crlf) strcat((char*)TX_buffer1, CRLF); // add CR LF if necesary
	msg=TX_buffer1;
	UDR0=*msg;
}

void send_string (uint8_t *str) {
	while (!flags.TX_finished); //waiting for other transmission to complete
	msg=str;
	UDR0=*msg; 
	flags.TX_finished=0;
}
#endif
//=========================================================================


static volatile struct {
  uint8_t ADC0;
  uint8_t ADC1;
  uint8_t ADC2;
  uint8_t ADC3;
  uint8_t ADC6;
  uint8_t ADC7;
} ADC_data; 

static volatile struct {
  uint16_t pulse0;
  uint16_t pulse1;
//  uint16_t pulse3;
  uint16_t t_start0;
  uint16_t t_end0;
  uint16_t t_start1;
  uint16_t t_end1;
} PULSE_data; 



//=========================================================================
// For ADC readings printing ===================================
#ifdef DEBUG
#define PPS 10
// defines the frequency of printing of readings (in 20 ms ticks).
// 50 corresponds to one set per second; 10 -> 5 sets per second.
// Do not set it too low because you might overload the TX ISR.
static volatile char data_str[]="   ;   ;   .\r\n";
static volatile uint8_t TX_delay=PPS; 
#endif
//=========================================================================


ISR (INT0_vect) {// timing capture for US sensor
	if (PIND&(1<<PD2)) { //raising edge IRQ - start of pulse
		flags.T1_ovf0=0;
		PULSE_data.t_start0=TCNT1; // There is no need to disable global interrupts as they are already disabled while and ISR is being executed.
	}
	if (!(PIND&(1<<PD2))) { //falling edge IRQ - end of pulse
		PULSE_data.t_end0=TCNT1;
		if (flags.T1_ovf0==1) PULSE_data.pulse0=PWM_TOP-PULSE_data.t_start0+PULSE_data.t_end0; //one ovf
			else if (flags.T1_ovf0==2) PULSE_data.pulse0=PWM_TOP-PULSE_data.t_start0+PULSE_data.t_end0+PWM_TOP; //two ovfs
			else if (flags.T1_ovf0==0) PULSE_data.pulse0=PULSE_data.t_end0-PULSE_data.t_start0; //no ovf
			else PULSE_data.pulse0=0xFFFF; // something went wrong		
	}
}

// >>>>>>>>>>>>>>>>>>>>>>>>> ADC <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
ISR (ADC_vect){ // the ADC runs on interrupt and populates ADC_data structure.
// You can use the ADC readings in the structure.
//  PORTB^=(1<<PB3); // to check ISR timing
  if (ADC_sample==0) { // scraping the first reading after the channel was changed
    ADC_acc=0; //reset the accumulator
    ADC_sample++;
    return; 
  }
  if (ADC_sample<=ADC_sample_max){ // averaging filter: accumulating readings
    ADC_acc+=ADCH;
    ADC_sample++;
    return;
  }
  if (ADC_sample>ADC_sample_max){
    ADC_sample=0;
	ADC_acc=(ADC_acc/ADC_sample_max); // averaging filter: dividing accumulated readings by the number of readings
    switch (ADMUX&7) {  // checking, which ADC channel was read. NOTE: the ADC multiplexer register is used - no need for a variable to keep track.
      case 0: { // channel 0
        ADMUX=(ADMUX&0xF0)|0x01; //switching to the next channel
// If your next sensor uses different Aref, you can change it here.
//		ADMUX=...?
        ADC_data.ADC0=(uint8_t)(ADC_acc); // storing the filtered reading in ADC_data.
        return; 
      }
      case 1: { 
        ADMUX=(ADMUX&0xF0)|0x02; 
        ADC_data.ADC1=(uint8_t)(ADC_acc);
        return; 
      }
      case 2: { 
        ADMUX=(ADMUX&0xF0)|0x03; 
        ADC_data.ADC2=(uint8_t)(ADC_acc);
        return; 
      }
      case 3: { 
        ADMUX=(ADMUX&0xF0)|0x06; 
        ADC_data.ADC3=(uint8_t)(ADC_acc);
        return; 
      }

      case 6: { 
        ADMUX=(ADMUX&0xF0)|0x07; 
        ADC_data.ADC6=(uint8_t)(ADC_acc);
        return; 
      }

      case 7: { 
        ADMUX=(ADMUX&0xF0)|0x00; 
        ADC_data.ADC7=(uint8_t)(ADC_acc);
        return; 
      }
      default: { //if something goes wrong - switching to Channel 0
        ADMUX=(ADMUX&0xF0)|0x00;    
      }
    }
  }
}
//================================================================================



void setup()
{
  gpio_init(); // initialise GPIOs
  DDRB|=(1<<PB5);    // LED "L" as output (gpio_init leaves it as input with pull-up)
  PORTB&=~(1<<PB5);  // L off

#ifdef DEBUG
  uart_tx_init();
  flags.TX_finished=1;
  uint8_t st3[]="DEBUG mode:\n\r";
  send_string(st3);
#endif


// -------- Timer/PWM init
// ======= PWM2 and D4 control (8-bit timer0) ===================
  timer0_init();

// ======= PWM0 and PWM1 control (16-bit timer1) ===================
  timer1_50Hz_init(1); // IRQ enabled: 20 ms system tick (sampling + LED L timing)

// ======= Timer2: PWM for D3 on PB3/OC2A ===========
// Phase-correct PWM, non-inverting, prescaler 64 -> ~490 Hz.
// D3 is active LOW, so OCR2A = 255 - brightness (OCR2A=0 -> pin always LOW -> 100%, 255 -> always HIGH -> 0%).
  OCR2A=255; // D3 off
  TCCR2A=(1<<COM2A1)|(1<<WGM20);
  TCCR2B=(1<<CS22);


// ADC init
  adc_init (7, 1); // channel 7, ADC IRQ enabled


//BLINK(500); // not used: PB3 is now driven by Timer2

  sei(); 

	flags.sample=1;
}

void loop() {  // LOOP until infinity

		if (flags.sample) { // set every 20 ms by Timer1 ISR
			flags.sample=0;

#ifdef ADC_DEBUG
	uint8_t *ADC_ptr=&ADC_data.ADC0;
	for (uint8_t i=0; i<6; i++){
		send_reading (*ADC_ptr, "ADC=", 1);	
		ADC_ptr++;
	} 
	send_string((uint8_t*)"===========\r\n");
#endif

			uint8_t adc=IR_ADC;
			uint8_t bright;

			// ---- D3 brightness and LED L: done on raw ADC values, no conversion to cm ----
			if (adc>ADC_D1) {        // closer than d1
				bright=255;
				L_flash=1;
			} else if (adc<ADC_D2) { // farther than d2
				bright=0;
				L_flash=1;
			} else {                 // inside [d1;d2]
				// Distance is proportional to 1/ADC, so brightness linear in distance is
				// 255*(1/ADC_D2 - 1/adc)/(1/ADC_D2 - 1/ADC_D1) = 255*ADC_D1*(adc-ADC_D2) / (adc*(ADC_D1-ADC_D2))
				bright=(uint8_t)( (255UL*ADC_D1*(adc-ADC_D2)) / ((uint32_t)adc*(ADC_D1-ADC_D2)) );
				L_flash=0;
			}
			OCR2A=255-bright;

#ifdef DEBUG
			// ---- UART: ADC reading and distance (mm), PPS ticks apart ----
			if (--TX_delay==0) {
				TX_delay=PPS;
				int16_t dist_mm;
				if (adc<ADC_80) dist_mm=-1; // beyond the sensor's 80 cm range
				else dist_mm=(int16_t)(IR_K/adc)-42;
				send_reading(adc, "ADC=", 0);
				send_reading(dist_mm, "  d_mm=", 1);
			}
#endif

		} //end if-sample

} // end loop