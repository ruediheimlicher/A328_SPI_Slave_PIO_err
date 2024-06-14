//#include <Arduino.h>
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

//#include <avr/pgmspace.h>
//#include <avr/sleep.h>
#include <avr/eeprom.h>
#include <inttypes.h>
#include <avr/eeprom.h>
#include <inttypes.h>
#include <avr/wdt.h>
//#include "defines.h"
#include <stdint.h>
#include <util/twi.h>

#include "lcd.c"
#include "adc.c"

// 
#include "OLED_Driver.h"
#include "DEV_Config.h"

#include <U8x8lib.h>
#include "u8g2lib.h"

#define LOOPLEDPORT		PORTD
#define LOOPLEDDDR		DDRD
#define LOOPSTEP			0x0FFF

// Define fuer Slave:
#define LOOPLED			4

#define  Error   0x01
#define  Success 0x02
#define SLAVE_COMMAND 2
char     TransmitState = 0x00;
//char*    TextString    = "AVR communicating via the SPI"+0x00;
#define BUFSIZE 8

#define SPI_CONTROL_DDR			DDRB
#define SPI_CONTROL_PORT		PORTB
#define SPI_CONTROL_CS			PORTB2	//CS fuer HomeCentral Master
#define SPI_CONTROL_MOSI		PORTB3
#define SPI_CONTROL_MISO		PORTB4
#define SPI_CONTROL_SCK			PORTB5

#define waitspi() while(!(SPSR&(1<<SPIF)))

volatile unsigned char incoming[BUFSIZE];
volatile short int received=0;
volatile uint8_t spistatus = 0;
#define RECEIVED	0


uint16_t loopcount0=0;
    uint16_t loopcount1=0;
    uint16_t loopcount2=0;

// U8X8_SSD1327_WS_128X128_HW_I2C(A5,A6);




// Intialization RoutineSlave Mode (interrupt controlled)
void Init_Slave_IntContr (void)
{
	volatile char IOReg;
	// Set PB6(MISO) as output 
	SPI_CONTROL_DDR    |= (1<<SPI_CONTROL_MISO);
	SPI_CONTROL_PORT    |= (1<<SPI_CONTROL_MISO);	// MISO als Output
	
	SPI_CONTROL_DDR	&= ~(1<<SPI_CONTROL_CS);	// Chip Select als Eingang
	//SPI_CONTROL_PORT |=(1<<SPI_CONTROL_CS);		// HI
	SPI_CONTROL_DDR	&= ~(1<<SPI_CONTROL_SCK);	// SCK Eingang
	//SPI_CONTROL_PORT |=(1<<SPI_CONTROL_SCK);		// HI
	//SPI_CONTROL_DDR	&= ~(1<<SPI_CONTROL_MOSI);	// MOSI als Eingang
	//SPI_CONTROL_PORT |=(1<<SPI_CONTROL_MOSI);		// HI

	// Enable SPI Interrupt and SPI in Slave Mode with SCK = CK/4
	SPCR  = (1<<SPIE)|(1<<SPE);
	IOReg   = SPSR;                         // Clear SPIF bit in SPSR
	IOReg   = SPDR;

	//DDRD	= 0xFF;	
	// Set Port D as output
	sei(); // Enable global interrupts
}

unsigned char spi_tranceiver (unsigned char data)
{
    // Load data into the buffer
    SPDR = data;
 
    //Wait until transmission complete
    while(!(SPSR & (1<<SPIF)));   // Return received data

  return(SPDR);
}

uint8_t received_from_spi(uint8_t data)
{
  SPDR = data;
  return SPDR;
}

void parse_message()
{

 switch(incoming[0]) 
 {
 case SLAVE_COMMAND:
   //flash_led(incoming[1])
	;
   break;
 default:
   PORTD ^=(1<<1);//LED 1 toggeln
	;
 }

}
// Interrupt Routine Slave Mode (interrupt controlled)
 
// called by the SPI system when there is data ready.
// Just store the incoming data in a buffer, when we receive a
// terminating byte (0x00) call parse_message to process the data received
ISR( SPI_STC_vect )
{
	
	PORTD ^=(1<<0);//LED 0 toggeln

	
	//lcd_puthex(received);
  incoming[received++] = SPDR;//received_from_spi(0x00);
  spistatus |= (1<<RECEIVED);
	SPDR = loopcount2;
  if (received >= BUFSIZE ) //|| incoming[received-1] == 0x00) 
  {
    parse_message();
    received = 0;
  }

 

/*
        if (SPDR != *TextString)                // Check for transmission errors
        {
                TransmitState = Error;  
                SPCR &= ~(1<<SPIE);             // Disable SPI interrupts
        }
        else 
        {
                TextString++;                   // Set Pointer on next Character
                if (*TextString == 0)           // Check for end of String
                {
                        TransmitState = Success;
                        SPCR &= ~(1<<SPIE);     // Disable SPI interrupts
                }
        }
*/
}


void slaveinit(void)
{
	DDRD |= (1<<DDD0); // ISR
   /*
 			//Pin 0 von PORT D als Ausgang fuer Schalter: ON		
	DDRD |= (1<<DDD1);		//Pin 1 von PORT D als Ausgang fuer Schalter: OFF
	DDRD |= (1<<DDD2);		//Pin 2 von PORT D als Ausgang fuer Buzzer
 	DDRD |= (1<<DDD3);		//Pin 3 von PORT D als Ausgang fuer LED TWI
	DDRD |= (1<<DDD4);		//Pin 4 von PORT D als Ausgang fuer LED
    */
  LOOPLEDDDR |= (1<<LOOPLED);		//Pin 5 von PORT D als Ausgang fuer LED Loop
	LOOPLEDPORT |= (1<<LOOPLED);		//Pin 5 von PORT D als Ausgang fuer LED Loop

	//LCD
	LCD_DDR |= (1<<LCD_RSDS_PIN);	//Pin 4 von PORT B als Ausgang fuer LCD
 	LCD_DDR |= (1<<LCD_ENABLE_PIN);	//Pin 5 von PORT B als Ausgang fuer LCD
	LCD_DDR |= (1<<LCD_CLOCK_PIN);	//Pin 6 von PORT B als Ausgang fuer LCD
/*
	DDRC &= ~(1<<DDC0);	//Pin 0 von PORT C als Eingang fuer ADC
//	PORTC |= (1<<DDC0); //Pull-up
	DDRC &= ~(1<<DDC1);	//Pin 1 von PORT C als Eingang fuer ADC 	
//	PORTC |= (1<<DDC1); //Pull-up
	DDRC &= ~(1<<DDC2);	//Pin 2 von PORT C als Eingang fuer ADC 	
//	PORTC |= (1<<DDC3); //Pull-up
	DDRC &= ~(1<<DDC3);	//Pin 3 von PORT C als Eingang fuer Tastatur 	
//	PORTC |= (1<<DDC3); //Pull-up
*/

	
	
}


int main (void) 
{
	  slaveinit();
	   
	  //uint16_t ADC_Wert= readKanal(0);
		
	  // initialize the LCD 
	  lcd_initialize(LCD_FUNCTION_8x2, LCD_CMD_ENTRY_INC, LCD_CMD_ON);
	
		lcd_gotoxy(0,0);
		lcd_puts("SPI Slave Init \0");
 		//lcd_puts("Guten Tag");
	  _delay_ms(1000);
	 
	 Init_Slave_IntContr();
	 System_Init();
  //Serial.print(F("OLED_Init()...\r\n"));
	lcd_puts(" OLED");
  //OLED_1in5_Init();
 // _delay_ms(500); 
  //OLED_1in5_Clear();  

    uint8_t Tastenwert=0;
    uint8_t TastaturCount=0;
    
    uint16_t TastenStatus=0;
    uint16_t Tastencount=0;
    uint16_t Tastenprellen=0x01F;
    //timer0();
    
    //initADC(TASTATURPIN);
	
    

		uint8_t spi_input = 0;
    //uint16_t startdelay1=0;

    //uint8_t twierrcount=0;
    //LOOPLEDPORT |=(1<<LOOPLED);
			 System_Init();
  //Serial.print(F("OLED_Init()...\r\n"));
	//lcd_puts("OLED_Init");
 // OLED_1in5_Init();
 // _delay_ms(500); 
 // OLED_1in5_Clear();  

 
 
	   //timer2();

		 DDRB |= (1<<6);
		 DDRB |= (1<<7);
   
	while (1)
	{
      //PORTD ^= (1<<0);
      //_delay_ms(50);
     // PORTD &= ~(1<<0);
		//Blinkanzeige
      wdt_reset();


			if (spistatus |= (1<<RECEIVED))
			{
				spistatus &= ~(1<<RECEIVED);
				cli();
 				lcd_gotoxy(0,1);
        lcd_putint(received);
				lcd_putc(' ');
				lcd_putint(incoming[received]);
				sei();

			}
		loopcount0++;
		if (loopcount0>LOOPSTEP)
		{
			//PORTB ^= (1<<6);
			//PORTB ^= (1<<7);

			loopcount0=0;
			
      loopcount1++;
      if(loopcount1 > 0x1F)
         {
            LOOPLEDPORT ^=(1<<LOOPLED);
            loopcount1 = 0;
            loopcount2++;
            //lcd_gotoxy(10,1);
            //lcd_putint16(loopcount2);
						//lcd_putc(' ');
						//lcd_putint(incoming[received]);
						//lcd_putc(' ');
						//lcd_putint(received);
						//lcd_putc('*');

         }
		}
		

		
	}//while

 return 0;
}// main
