#include <avr/io.h>
#include <avr/pgmspace.h>
#include <avr/interrupt.h>
#include <avr/wdt.h>
#include <util/delay.h>
#include <stdio.h>
#include <stdlib.h>

#define BAUD 115200
#include <util/setbaud.h>

#define DEBUG 1
void uart_init(void) {
   UBRR0H = UBRRH_VALUE;
   UBRR0L = UBRRL_VALUE;

   #if USE_2X
      UCSR0A |= _BV(U2X0);
   #else
      UCSR0A &= ~(_BV(U2X0));
   #endif

   UCSR0C = _BV(UCSZ01) | _BV(UCSZ00); /* 8-bit data */
   UCSR0B = _BV(RXEN0) | _BV(TXEN0);   /* Enable RX and TX */
}

void uart_putchar(char c) {
   loop_until_bit_is_set(UCSR0A, UDRE0); /* Wait until data register empty. */
   UDR0 = c;
}

char uart_getchar(void) {
   loop_until_bit_is_set(UCSR0A, RXC0); /* Wait until data exists. */
   return UDR0;
}

FILE uart_output = FDEV_SETUP_STREAM(uart_putchar, NULL, _FDEV_SETUP_WRITE);
FILE uart_input = FDEV_SETUP_STREAM(NULL, uart_getchar, _FDEV_SETUP_READ);
FILE uart_io = FDEV_SETUP_STREAM(uart_putchar, uart_getchar, _FDEV_SETUP_RW);

void InitADC()
{
 ADMUX |= (1<<REFS0);						// Select Vref=AVcc
 ADCSRA |= (1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0)|(1<<ADEN);    	//set prescaller to 128 and enable ADC 
}

uint16_t ReadADC(uint8_t ADCport)
{
 ADMUX = (ADMUX & 0xF0) | (ADCport & 0x0F);			//select ADC port with safety mask
 ADCSRA |= (1<<ADSC);						//single conversion mode
 while( ADCSRA & (1<<ADSC) );					// wait until ADC conversion is complete
 return ADC;
}

void moveForward(int dutycyclel, int dutycycler)
{
    DDRD |= (1 << DDD3);			// PD3
    DDRB |= (1 << DDB2); 			// PB2
    DDRD |= (1 << DDD5);			// PD5
    DDRD |= (1 << DDD6);			// PD6

    // Timer0 Registers (for PD5,PD6)
    OCR0A = 0;					// Set PWM Duty Cycle for PD6 (max 255)
    OCR0B = dutycycler;				// Set PWM Duty Cycle for PD5 (max 255)
    TCCR0A |= (1 << COM0A1);			// set none-inverting mode
    TCCR0A |= (1 << COM0B1);			// set none-inverting mode
    TCCR0A |= (1 << WGM01) | (1 << WGM00);	// set fast PWM Mode with OCRnx as top value
    TCCR0B |= (1 << CS01);			// set prescaler to 8 and starts PWM
    
    // Timer2 Registers (for PD3)
    //OCR2B = floor(dutycycle*65/80);				// Set PWM Duty Cycle(max 255)
    OCR2B = dutycyclel;				// Set PWM Duty Cycle(max 255)
    TCCR2A |= (1 << COM2B1);                    // set none-inverting mode
    TCCR2A |= (1 << WGM21) | (1 << WGM20);      // set fast PWM Mode with OCRnx as top value
    TCCR2B |= (1 << CS21);                      // set prescaler to 8 and starts PWM

    // Timer1 Registers (for PB2)
    ICR1 = 0x00FF;		 	        // set TOP of the counter
    OCR1B = 0;					// Set PWM Duty Cycle
    TCCR1A |= (1 << COM1B1);			// set none-inverting mode
    TCCR1A |= (1 << WGM11);			// set fast PWM Mode with OCRnx as top value
    TCCR1B |= (1 << WGM12)|(1 << WGM13);	// set fast PWM Mode with OCRnx as top value
    TCCR1B |= (1 << CS11);			// set prescaler to 8 and starts PWM
}
void moveBackward(int dutycyclel, int dutycycler)
{
    DDRD |= (1 << DDD3);			// PD3
    DDRB |= (1 << DDB2); 			// PB2
    DDRD |= (1 << DDD5);			// PD5
    DDRD |= (1 << DDD6);			// PD6

    // Timer0 Registers (for PD5,PD6)
    OCR0A = dutycycler;				// Set PWM Duty Cycle for PD6 (max 255)
    OCR0B = 0;					// Set PWM Duty Cycle for PD5 (max 255)
    TCCR0A |= (1 << COM0A1);			// set none-inverting mode
    TCCR0A |= (1 << COM0B1);			// set none-inverting mode
    TCCR0A |= (1 << WGM01) | (1 << WGM00);	// set fast PWM Mode with OCRnx as top value
    TCCR0B |= (1 << CS01);			// set prescaler to 8 and starts PWM
    
    // Timer2 Registers (for PD3)
    OCR2B = 0;					// Set PWM Duty Cycle(max 255)
    TCCR2A |= (1 << COM2B1);                    // set none-inverting mode
    TCCR2A |= (1 << WGM21) | (1 << WGM20);      // set fast PWM Mode with OCRnx as top value
    TCCR2B |= (1 << CS21);                      // set prescaler to 8 and starts PWM

    // Timer1 Registers (for PB2)
    ICR1 = 0x00FF;		 	        // set TOP of the counter
    OCR1B = dutycyclel;				// Set PWM Duty Cycle
    //OCR1B = floor(dutycycle*65/80);				// Set PWM Duty Cycle
    TCCR1A |= (1 << COM1B1);			// set none-inverting mode
    TCCR1A |= (1 << WGM11);			// set fast PWM Mode with OCRnx as top value
    TCCR1B |= (1 << WGM12)|(1 << WGM13);	// set fast PWM Mode with OCRnx as top value
    TCCR1B |= (1 << CS11);			// set prescaler to 8 and starts PWM
}
void moveLeft(int dutycyclel, int dutycycler)
{
    DDRD |= (1 << DDD3);			// PD3
    DDRB |= (1 << DDB2); 			// PB2
    DDRD |= (1 << DDD5);			// PD5
    DDRD |= (1 << DDD6);			// PD6

    // Timer0 Registers (for PD5,PD6)
    OCR0A = 0;					// Set PWM Duty Cycle for PD6 (max 255)
    OCR0B = dutycycler;				// Set PWM Duty Cycle for PD5 (max 255)
    TCCR0A |= (1 << COM0A1);			// set none-inverting mode
    TCCR0A |= (1 << COM0B1);			// set none-inverting mode
    TCCR0A |= (1 << WGM01) | (1 << WGM00);	// set fast PWM Mode with OCRnx as top value
    TCCR0B |= (1 << CS01);			// set prescaler to 8 and starts PWM
    
    // Timer2 Registers (for PD3)
    OCR2B = 0;				// Set PWM Duty Cycle(max 255)
    TCCR2A |= (1 << COM2B1);                    // set none-inverting mode
    TCCR2A |= (1 << WGM21) | (1 << WGM20);      // set fast PWM Mode with OCRnx as top value
    TCCR2B |= (1 << CS21);                      // set prescaler to 8 and starts PWM

    // Timer1 Registers (for PB2)
    ICR1 = 0x00FF;		 	        // set TOP of the counter
    OCR1B = dutycyclel;				// Set PWM Duty Cycle
    TCCR1A |= (1 << COM1B1);			// set none-inverting mode
    TCCR1A |= (1 << WGM11);			// set fast PWM Mode with OCRnx as top value
    TCCR1B |= (1 << WGM12)|(1 << WGM13);	// set fast PWM Mode with OCRnx as top value
    TCCR1B |= (1 << CS11);			// set prescaler to 8 and starts PWM
}
void moveRight(int dutycyclel,int dutycycler)
{
    DDRD |= (1 << DDD3);			// PD3
    DDRB |= (1 << DDB2); 			// PB2
    DDRD |= (1 << DDD5);			// PD5
    DDRD |= (1 << DDD6);			// PD6

    // Timer0 Registers (for PD5,PD6)
    OCR0A = dutycycler;				// Set PWM Duty Cycle for PD6 (max 255)
    OCR0B = 0;					// Set PWM Duty Cycle for PD5 (max 255)
    TCCR0A |= (1 << COM0A1);			// set none-inverting mode
    TCCR0A |= (1 << COM0B1);			// set none-inverting mode
    TCCR0A |= (1 << WGM01) | (1 << WGM00);	// set fast PWM Mode with OCRnx as top value
    TCCR0B |= (1 << CS01);			// set prescaler to 8 and starts PWM
    
    // Timer2 Registers (for PD3)
    OCR2B = dutycyclel;				// Set PWM Duty Cycle(max 255)
    TCCR2A |= (1 << COM2B1);                    // set none-inverting mode
    TCCR2A |= (1 << WGM21) | (1 << WGM20);      // set fast PWM Mode with OCRnx as top value
    TCCR2B |= (1 << CS21);                      // set prescaler to 8 and starts PWM

    // Timer1 Registers (for PB2)
    ICR1 = 0x00FF;		 	        // set TOP of the counter
    OCR1B = 0;					// Set PWM Duty Cycle
    TCCR1A |= (1 << COM1B1);			// set none-inverting mode
    TCCR1A |= (1 << WGM11);			// set fast PWM Mode with OCRnx as top value
    TCCR1B |= (1 << WGM12)|(1 << WGM13);	// set fast PWM Mode with OCRnx as top value
    TCCR1B |= (1 << CS11);			// set prescaler to 8 and starts PWM
}
void moveStop()
{
    DDRD &= ~(1 << DDD3);			// PD3
    DDRB &= ~(1 << DDB2); 			// PB2
    DDRD &= ~(1 << DDD5);			// PD5
    DDRD &= ~(1 << DDD6);			// PD6
}

void auto_mode() 
{
	while(1) {
		//printf("Auto Mode: Enter 's' to stop and 'm' for manual mode\r\n");
		char mode = getchar();
  	        //if( bit_is_set(UCSR0A, RXC0) != 0) {
		//	char mode = UDR0;
			if(mode == 's') {
				moveStop();
				printf("Robot Stops\r\n");
			}
			if(mode == 'm') {
				printf("Manual mode enabled\r\n");
				manual_mode();
			}
//		}else{
//			_delay_ms(50);
//			int adc_val0=ReadADC(0);
//			_delay_ms(50);
//			int adc_val1=ReadADC(1);
//			_delay_ms(50);
//			int adc_val2=ReadADC(2);
//			printf("Auto Mode - Sensor Readings :  ClifL = %d, ClifR = %d, Bump = %d\r\n",adc_val0,adc_val1,adc_val2);
//												     
//			  if( (adc_val0 < 100) ) {
//			  //if( (adc_val0 < 100) | (adc_val1 < 100) | (adc_val2 < 100) ) {
//				moveStop();
//				printf("Robot stops\r\n");
//				_delay_ms(0500);
//				
//				moveBackward(160,160);
//				printf("Robot moves backward\r\n");
//				_delay_ms(1000);
//
//				moveStop();
//				printf("Robot stops\r\n");
//				_delay_ms(0500);
//				
//				moveRight(160,160);
//				printf("Robot moves Right\r\n");
//				_delay_ms(0500);
//
//				moveStop();
//				printf("Robot stops\r\n");
//				_delay_ms(0500);
//			}else{
//				moveForward(160,160);
//				printf("Robot moves forward\r\n");
//			}
//		}
      }
}
void manual_mode() 
{
    int discardSensors = 1;
    while(1) {
	    _delay_ms(1);
	    //printf("\r\nPlease enter the input in the format : <dir> <l1><l0> <r1><r0>\r\n");
	    //char dir   = getchar();
  	      if( bit_is_set(UCSR0A, RXC0) != 0) {
	       	    char dir = UDR0;
	 	    printf("UDR0 = %c\r\n",dir);

		    if(dir == 'a') {
			printf("Auto mode enabled\r\n");
			auto_mode();
		    }
		    while( (dir !='d') & (dir !='h') & (dir !='f') & (dir !='b') & (dir !='l') & (dir !='r') & (dir !='s') ) {
			printf("Enter valid direction character(f,b,l,r,s,d,h)");
			char dir   = getchar();
		    }
		    if(dir=='d') {
			discardSensors = 1;		// Discard Sensor Values
		    }
		    if(dir=='h') {
			discardSensors = 0; 	// Hold    Sensor Values
		    }

		    if(dir=='f') {
			int  speedl1 = getchar();
			int  speedl0 = getchar();
			int  speedr1 = getchar();
			int  speedr0 = getchar();
			printf("You entered input as : %c %c%c %c%c\r\n", dir, speedl1,speedl0,speedr1,speedr0);
			int  speedl  = ((speedl1-48)*10  +  (speedl0-48))*255/99;
			int  speedr  = ((speedr1-48)*10  +  (speedr0-48))*255/99;

			moveForward(speedl,speedr);
			printf("Robot moves forward\r\n");
		    }
		    if(dir=='b') {
			int  speedl1 = getchar();
			int  speedl0 = getchar();
			int  speedr1 = getchar();
			int  speedr0 = getchar();
			printf("You entered input as : %c %c%c %c%c\r\n", dir, speedl1,speedl0,speedr1,speedr0);
			int  speedl  = ((speedl1-48)*10  +  (speedl0-48))*255/99;
			int  speedr  = ((speedr1-48)*10  +  (speedr0-48))*255/99;

			moveBackward(speedl,speedr);
			printf("Robot moves backward\r\n");
		    }
		    if(dir=='r') {
			int  speedl1 = getchar();
			int  speedl0 = getchar();
			int  speedr1 = getchar();
			int  speedr0 = getchar();
			printf("You entered input as : %c %c%c %c%c\r\n", dir, speedl1,speedl0,speedr1,speedr0);
			int  speedl  = ((speedl1-48)*10  +  (speedl0-48))*255/99;
			int  speedr  = ((speedr1-48)*10  +  (speedr0-48))*255/99;

			moveRight(speedl,speedr);
			printf("Robot moves right\r\n");
		    }
		    if(dir=='l') {
			int  speedl1 = getchar();
			int  speedl0 = getchar();
			int  speedr1 = getchar();
			int  speedr0 = getchar();
			printf("You entered input as : %c %c%c %c%c\r\n", dir, speedl1,speedl0,speedr1,speedr0);
			int  speedl  = ((speedl1-48)*10  +  (speedl0-48))*255/99;
			int  speedr  = ((speedr1-48)*10  +  (speedr0-48))*255/99;

			moveLeft(speedl,speedr);
			printf("Robot moves left\r\n");
		    }
		    if(dir=='s') {
			printf("You entered input as : %c\r\n", dir);

			moveStop();
			printf("Robot stops\r\n");
		    }

  	     }
	     else if( discardSensors!=1 ) {
			  _delay_ms(50);
			  int adc_val0=ReadADC(0);
			  _delay_ms(50);
			  int adc_val1=ReadADC(1);
			  _delay_ms(50);
			  int adc_val2=ReadADC(2);
			  printf("Sensor Readings :  ClifL = %d, ClifR = %d, Bump = %d\r\n",adc_val0,adc_val1,adc_val2);

			  if( (adc_val0 < 100) ) {
			  //if( (adc_val0 < 100) | (adc_val1 < 100) | (adc_val2 < 100) ) {
				moveStop();
				printf("Robot stops\r\n");
				_delay_ms(0500);
				
				moveBackward(160,160);
				printf("Robot moves backward\r\n");
				_delay_ms(1000);

				moveStop();
				printf("Robot stops\r\n");
				_delay_ms(0500);
				
				moveRight(160,160);
				printf("Robot moves Right\r\n");
				_delay_ms(0500);

				moveStop();
				printf("Robot stops\r\n");
				_delay_ms(0500);
			  }
            }
  }
}


int main(void)
{

  /* Setup serial port */
  uart_init();
  stdout = &uart_output;
  stdin  = &uart_input;
  printf("Hello world!\r\n");

  InitADC();
  moveStop();

  // Setup ports
  DDRB |= (1<<1) | (1<<0);
  PORTB |= (1<<0);
  PORTB &= ~(1<<1);

  /* Print hello and then echo serial
  ** port data while blinking LED */
  //printf("Hello world!\r\n");
  //PORTB ^= 0x01;


  while(1) {
	manual_mode();
  }

}
