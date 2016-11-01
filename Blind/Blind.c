#define F_CPU 12000000
#include <avr/io.h>
#include <avr/wdt.h>
#include <avr/interrupt.h>     /* for sei() */
#include <util/delay.h>         /* for _delay_ms() */
#define F_CPU 12000000
#define INSTR_PER_US 12                   // instructions per microsecond (depends on MCU clock, 12MHz current)
#define INSTR_PER_MS 12000                // instructions per millisecond (depends on MCU clock, 12MHz current)
#define MAX_RESP_TIME_MS 200      // timeout - max time to wait for low voltage drop (higher value increases measuring distance at the price of slower sampling)
#define DELAY_BETWEEN_TESTS_MS 50 // echo cancelling time between sampling
#define rs PD0
//#define rw PD1
#define en PD1

void lcd_init();
void dis_cmd(char);
void dis_data(char);
void lcdcmd(char);
void lcddata(char);

int i=0;

// LCD module information
#define lcd_LineOne     0x00                    // start of line 1
#define lcd_LineTwo     0x40                    // start of line 2
// LCD instructions
#define lcd_Clear           0b00000001          // replace all characters with ASCII 'space'
#define lcd_Home            0b00000010          // return cursor to first position on first line
#define lcd_EntryMode       0b00000110          // shift cursor from left to right on read/write
#define lcd_DisplayOff      0b00001000          // turn display off
#define lcd_DisplayOn       0b00001100          // display on, cursor off, don't blink character
#define lcd_FunctionReset   0b00110000          // reset the LCD
#define lcd_FunctionSet4bit 0b00101000          // 4-bit data, 2-line display, 5 x 7 font
#define lcd_SetCursor       0b10000000          // set cursor position
volatile long result = 0;
volatile unsigned char up = 0;
volatile unsigned char running = 0;
volatile uint32_t timerCounter = 0;
volatile char res[5];

// timer overflow interrupt, each time when timer value passes 255 value
SIGNAL(TIMER0_OVF_vect)
{
        if (up) {       // voltage rise was detected previously
                timerCounter++;   // count the number of overflows
                // dont wait too long for the sonar end response, stop if time for measuring the distance exceeded limits
                uint32_t ticks = timerCounter * 256 + TCNT0;
                uint32_t max_ticks = (uint32_t)MAX_RESP_TIME_MS * INSTR_PER_MS; // this could be replaced with a value instead of multiplying
                if (ticks > max_ticks) {
                        // timeout
                        up = 0;          // stop counting timer values
                        running = 0; // ultrasound scan done
                        result = -1; // show that measurement failed with a timeout (could return max distance here if needed)
                }
        }
}
// interrupt for INT1 pin, to detect high/low voltage changes
/**
        We assume, that high voltage rise comes before low drop and not vice versa -
        however this should be implemented more correctly using both interrupts INT0/INT1,
        (i.e. INT0 configured for high rise, and INT1 - for low rise, thus the code must be separated also)
*/
SIGNAL(INT1_vect)
{
        if (running) { //accept interrupts only when sonar was started
                if (up == 0) { // voltage rise, start time measurement
                        up = 1;
                        timerCounter = 0;
                        TCNT0 = 0; // reset timer counter
                } else {
                        // voltage drop, stop time measurement
                        up = 0;
                         //real_time=(timeCounter)*(1 / F_CPU)
                        // convert from time to distance(millimeters): d = [ time_s * 340m/s ] / 2 = time_us/58
                        result = (timerCounter * 256 + TCNT0) / 58;
                    if(result<1000){
						res[0]=result/100 + '0';
                        res[1]=(result%100)/10 + '0';
                        res[2]=result%10 + '0';
                        res[3]='c';
                        res[4]='m';
						_delay_ms(100);
                   }
                else{
                res[0]='f';
                res[1]='a';
                res[2]='r';
                    }
						i=0;
                       while(res[i]!='\0')
                       {
	                       dis_data(res[i]);
	                       _delay_ms(200);
	                       i++;
					   }	                       
                        running = 0;
                }
        }
}

// generate an impulse for the Trig input (starts the sonar)
void sonar() {
        PORTC = 0x00; // clear to zero for 1 us
        _delay_us(1);
        PORTC = 0x01; // set high for 10us
        running = 1;  // sonar launched
        _delay_us(10);
        PORTC = 0x00; // clear
	    
	    DDRD=0xFF;
	    lcd_init();
}
int main(void)
{
        // ------------------- ultrasonic init code --------------------
        DDRC = 1; // PB0 output - connected to Trig
        PORTC = 0; // clear
        // turn on interrupts for INT1, connect Echo to INT1
        MCUCR |= (0 << ISC11) | (1 << ISC10); // enable interrupt on any(rising/droping) edge
        GICR |= (1 << INT1);      // Turns on INT1
        // setup 8 bit timer & enable interrupts, timer increments to 255 and interrupts on overflow
        TCCR0 = (0<<CS02)|(0<<CS01)|(1<<CS00); // select internal clock with no prescaling
        TCNT0 = 0; // reset counter to zero
        TIMSK = 1<<TOIE0; // enable timer interrupt
        sei(); // enable all(global) interrupts
    _delay_us(80);

    for(;;){  /* main program loop */
                // other code here...
                if (running == 0) { // launch only when next iteration can happen
                        // create a delay between tests, to compensate for old echoes
_delay_ms(DELAY_BETWEEN_TESTS_MS);
                        sonar(); // launch measurement!
                }

				}
				}

void lcd_init() // fuction for intialize
{
	dis_cmd(0x02); // to initialize LCD in 4-bit mode.
	dis_cmd(0x28); //to initialize LCD in 2 lines, 5X7 dots and 4bit mode.
	dis_cmd(0x0C);
	dis_cmd(0x06);
	dis_cmd(0x83);
}

void dis_cmd(char cmd_value)
{
	char cmd_value1;
	
	cmd_value1 = cmd_value & 0xF0; //mask lower nibble because PA4-PA7 pins are used.
	lcdcmd(cmd_value1); // send to LCD
	
	cmd_value1 = ((cmd_value<<4) & 0xF0); //shift 4-bit and mask
	lcdcmd(cmd_value1); // send to LCD
}


void dis_data(char data_value)
{
	char data_value1;
	
	data_value1=data_value&0xF0;
	lcddata(data_value1);
	
	data_value1=((data_value<<4)&0xF0);
	lcddata(data_value1);
}

void lcdcmd(char cmdout)
{
	PORTD=cmdout;
	PORTD&=~(1<<rs);
	//PORTA&=~(1<<rw);
	PORTD|=(1<<en);
	_delay_ms(1);
	PORTD&=~(1<<en);
}

void lcddata(char dataout)
{
	PORTD=dataout;
	PORTD|=(1<<rs);
	//PORTA&=~(1<<rw);
	PORTD|=(1<<en);
	_delay_ms(1);
	PORTD&=~(1<<en);
}


// There are three control lines on LCD module.

// EN- When it is set to 1,means that we are sending data to lcd. To send data to the LCD, your program should first set this line high (1) and then set the other two control lines and/or put data on the data bus. When the other lines are completely ready, bring EN low (0) again. The 1-0 transition tells the 44780 to take the data currently found on the other control lines and on the data bus and to treat it as a command.

// RS- “Register Select” line. When RS is low (0), the data is to be treated as a command or special instruction (such as clear screen, position cursor, etc.). When RS is high (1), the data being sent is text data which sould be displayed on the screen. For example, to display the letter “T” on the screen you would set RS high.

//The RW line is the “Read/Write” control line. When RW is low (0), the information on the data bus is being written to the LCD. When RW is high (1), the program is effectively querying (or reading) the LCD. Only one instruction (“Get LCD status”) is a read command. All others are write commands–so RW will almost always be low.
