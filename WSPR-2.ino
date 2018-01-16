
// Simple WSPR transmitter
// Bruce MacKinnon KC1FSZ
// 14 January 2018
//
#include <JTEncode.h>
#include <AD9850.h>    //http://github.com/F4GOJ/AD9850
#include <DebouncedSwitch.h>

// Mode defines
#define WSPR_TONE_SPACING       1.46           // ~1.46 Hz
#define WSPR_DELAY              683           // Delay value for WSPR
#define WSPR_FREQ               7040000.0      // 40m WSPR freq

// AD9850 Pins
#define W_CLK_PIN 6
#define FQ_UD_PIN 7
#define DATA_PIN 8
#define RESET_PIN 9
// Pushbutton
#define BUTTON_PIN 10
// LED
#define LED0_PIN 13

// ----------------------------------------------------------------------------------------
// DS1302 code adapted from code by arduino.cc user "Krodal"
//
#define DS1302_SCLK_PIN   2    // Arduino pin for the Serial Clock
#define DS1302_IO_PIN     3    // Arduino pin for the Data I/O
#define DS1302_CE_PIN     4    // Arduino pin for the Chip Enable

// Macros to convert the bcd values of the registers to normal
// integer variables.
// The code uses separate variables for the high byte and the low byte
// of the bcd, so these macros handle both bytes separately.
#define bcd2bin(h,l)    (((h)*10) + (l))
#define bin2bcd_h(x)   ((x)/10)
#define bin2bcd_l(x)    ((x)%10)

#define DS1302_ENABLE            0x8E
#define DS1302_TRICKLE           0x90
#define DS1302_CLOCK_BURST       0xBE
#define DS1302_CLOCK_BURST_WRITE 0xBE
#define DS1302_CLOCK_BURST_READ  0xBF
#define DS1302_RAMSTART          0xC0
#define DS1302_RAMEND            0xFC
#define DS1302_RAM_BURST         0xFE
#define DS1302_RAM_BURST_WRITE   0xFE
#define DS1302_RAM_BURST_READ    0xFF

// Defines for the bits, to be able to change 
// between bit number and binary definition.
// By using the bit number, using the DS1302 
// is like programming an AVR microcontroller.
// But instead of using "(1<<X)", or "_BV(X)", 
// the Arduino "bit(X)" is used.
#define DS1302_D0 0
#define DS1302_D1 1
#define DS1302_D2 2
#define DS1302_D3 3
#define DS1302_D4 4
#define DS1302_D5 5
#define DS1302_D6 6
#define DS1302_D7 7

// Bit for reading (bit in address)
#define DS1302_READBIT DS1302_D0 // READBIT=1: read instruction

// Bit for clock (0) or ram (1) area, 
// called R/C-bit (bit in address)
#define DS1302_RC DS1302_D6

// Seconds Register
#define DS1302_CH DS1302_D7   // 1 = Clock Halt, 0 = start

// Hour Register
#define DS1302_AM_PM DS1302_D5 // 0 = AM, 1 = PM
#define DS1302_12_24 DS1302_D7 // 0 = 24 hour, 1 = 12 hour

// Enable Register
#define DS1302_WP DS1302_D7   // 1 = Write Protect, 0 = enabled

// Trickle Register
#define DS1302_ROUT0 DS1302_D0
#define DS1302_ROUT1 DS1302_D1
#define DS1302_DS0   DS1302_D2
#define DS1302_DS1   DS1302_D2
#define DS1302_TCS0  DS1302_D4
#define DS1302_TCS1  DS1302_D5
#define DS1302_TCS2  DS1302_D6
#define DS1302_TCS3  DS1302_D7

struct ds1302_struct
{
  uint8_t Seconds:4;      // low decimal digit 0-9
  uint8_t Seconds10:3;    // high decimal digit 0-5
  uint8_t CH:1;           // CH = Clock Halt
  uint8_t Minutes:4;
  uint8_t Minutes10:3;
  uint8_t reserved1:1;
  union
  {
    struct
    {
      uint8_t Hour:4;
      uint8_t Hour10:2;
      uint8_t reserved2:1;
      uint8_t hour_12_24:1; // 0 for 24 hour format
    } h24;
    struct
    {
      uint8_t Hour:4;
      uint8_t Hour10:1;
      uint8_t AM_PM:1;      // 0 for AM, 1 for PM
      uint8_t reserved2:1;
      uint8_t hour_12_24:1; // 1 for 12 hour format
    } h12;
  };
  uint8_t Date:4;           // Day of month, 1 = first day
  uint8_t Date10:2;
  uint8_t reserved3:2;
  uint8_t Month:4;          // Month, 1 = January
  uint8_t Month10:1;
  uint8_t reserved4:3;
  uint8_t Day:3;            // Day of week, 1 = first day (any day)
  uint8_t reserved5:5;
  uint8_t Year:4;           // Year, 0 = year 2000
  uint8_t Year10:4;
  uint8_t reserved6:7;
  uint8_t WP:1;             // WP = Write Protect
};

JTEncode jtencode;
DebouncedSwitch debouncedButton(25);

const char call[] = "KC1FSZ";
const char loc[] = "FN42";  // Wellesley and Boston
//const char loc[] = "DM07";  // Wawona
const uint8_t dbm = 27; // 0.5W
// Interval
const long FOUR_MINUTES_MS = 4L * 60L * 1000L;

uint8_t tx_buffer[255];
double freq = WSPR_FREQ;
uint8_t symbol_count = WSPR_SYMBOL_COUNT;
uint16_t tone_delay = WSPR_DELAY;
double tone_spacing = WSPR_TONE_SPACING;

enum Mode { IDLE, INTRA_SYMBOL, SYMBOL };

// State (for state machine)
Mode mode = Mode::IDLE;
// The last time we sent a WSPR message (i.e. the start of the last two minute cycle)
long lastTransmitStamp = millis();
// The last time we started a symbol
long lastSymbolStartStamp = 0;
// Pointer through the sequence of symbols
long symbolPtr = 0;

void setup() {

  Serial.begin(9600);
  delay(100);
  Serial.println("KC1FSZ WSPR Beacon - Verson 2");
  Serial.print("Call: ");
  Serial.print(call);
  Serial.print(", Grid: ");
  Serial.println(loc);

  // Get the pins all configured
  pinMode(W_CLK_PIN,OUTPUT);
  pinMode(FQ_UD_PIN,OUTPUT);
  pinMode(DATA_PIN,OUTPUT);
  pinMode(RESET_PIN,OUTPUT);
  pinMode(BUTTON_PIN,INPUT_PULLUP);
  pinMode(LED0_PIN,OUTPUT);

  // Initialze the AD9850
  DDS.begin(W_CLK_PIN,FQ_UD_PIN,DATA_PIN,RESET_PIN);
  DDS.down();
  
  // Build the WSPR message and leave it in the transmit buffer
  jtencode.wspr_encode(call,loc,dbm,tx_buffer); 
  
  // Start by clearing the Write Protect bit
  // Otherwise the clock data cannot be written
  // The whole register is written, 
  // but the WP-bit is the only bit in that register.
  DS1302_write(DS1302_ENABLE, 0);

  // Disable Trickle Charger.
  DS1302_write(DS1302_TRICKLE, 0x00);

  // Read all clock data at once (burst mode).
  ds1302_struct rtc;
  DS1302_clock_burst_read( (uint8_t *) &rtc);

  // Load up the time
  int minute = bcd2bin(rtc.Minutes10,rtc.Minutes) % 2;
  int second = bcd2bin(rtc.Seconds10,rtc.Seconds);
  Serial.print("Time ");
  Serial.print(minute);
  Serial.print(":");
  Serial.println(second);
  // Set the last transmission stamp to to the top of the last even minute
  // so that we are properly synchronized.
  lastTransmitStamp = millis() - (minute * 60 * 1000) - (second * 1000);
}

void loop() {
  
  long now = millis();

  if (mode == Mode::IDLE) {
    // Transmission starts every 4 minutes
    if ((now - lastTransmitStamp) > FOUR_MINUTES_MS) {
      mode = Mode::INTRA_SYMBOL;
      symbolPtr = 0;
      lastTransmitStamp = now;
      Serial.println("Starting WSPR message");
      digitalWrite(LED0_PIN,1);
      DDS.up();
    }
  } 
  else if (mode == Mode::INTRA_SYMBOL) {
    mode = Mode::SYMBOL;
    lastSymbolStartStamp = now;
    //Serial.print(symbolPtr);
    //Serial.print(": ");
    double symbolFreq = freq + ((double)tx_buffer[symbolPtr] * tone_spacing);
    //Serial.print(tx_buffer[symbolPtr]);
    //Serial.print(" ");
    //Serial.print(symbolFreq);
    //Serial.print("\n");
    // This is where the AD9850 actually gets programmed
    DDS.setfreq(symbolFreq, 0);
  } 
  else if (mode == Mode::SYMBOL) {
    // Check to see if we have sent the symbol for long enough
    if (now - lastSymbolStartStamp > tone_delay) {
      mode = Mode::INTRA_SYMBOL;
      symbolPtr++;
      // Check to see if the entire message has bee=n sent
      if (symbolPtr >= symbol_count) {
        mode = Mode::IDLE;
        Serial.println("Completed WSPR message");
        digitalWrite(LED0_PIN,0);
        DDS.down();
      }
    }
  }

  // Look at pushbutton
  debouncedButton.loadSample(digitalRead(BUTTON_PIN) == LOW);

  // When pressed, reset the clock
  if (debouncedButton.getState()) {
    mode = Mode::IDLE;
    // Force the time to now minus four minutes
    lastTransmitStamp = now - FOUR_MINUTES_MS;
    digitalWrite(LED0_PIN,0);
    DDS.down();
  }
}

// --------------------------------------------------------
// DS1302_clock_burst_read
//
// This function reads 8 bytes clock data in burst mode
// from the DS1302.
//
// This function may be called as the first function, 
// also the pinMode is set.
//
void DS1302_clock_burst_read(uint8_t *p) {

  int i;
  
  _DS1302_start();

  // Instead of the address, 
  // the CLOCK_BURST_READ command is issued
  // the I/O-line is released for the data
  _DS1302_togglewrite(DS1302_CLOCK_BURST_READ, true);  

  for( i=0; i<8; i++)
  {
    *p++ = _DS1302_toggleread();
  }
  _DS1302_stop();
}

// --------------------------------------------------------
// DS1302_write
//
// This function writes a byte to the DS1302 (clock or ram).
//
// The address could be like "0x80" or "0x81", 
// the lowest bit is cleared anyway.
//
// This function may be called as the first function, 
// also the pinMode is set.
//
void DS1302_write( int address, uint8_t data)
{
  // clear lowest bit (read bit) in address
  bitClear( address, DS1302_READBIT);   

  _DS1302_start();
  // don't release the I/O-line
  _DS1302_togglewrite( address, false); 
  // don't release the I/O-line
  _DS1302_togglewrite( data, false); 
  _DS1302_stop();  
}

// --------------------------------------------------------
// _DS1302_start
//
// A helper function to setup the start condition.
//
// An 'init' function is not used.
// But now the pinMode is set every time.
// That's not a big deal, and it's valid.
// At startup, the pins of the Arduino are high impedance.
// Since the DS1302 has pull-down resistors, 
// the signals are low (inactive) until the DS1302 is used.

void _DS1302_start( void) {
  digitalWrite( DS1302_CE_PIN, LOW); // default, not enabled
  pinMode( DS1302_CE_PIN, OUTPUT);  
  digitalWrite( DS1302_SCLK_PIN, LOW); // default, clock low
  pinMode( DS1302_SCLK_PIN, OUTPUT);
  pinMode( DS1302_IO_PIN, OUTPUT);
  digitalWrite( DS1302_CE_PIN, HIGH); // start the session
  delayMicroseconds( 4);           // tCC = 4us
}

// --------------------------------------------------------
// _DS1302_stop
//
// A helper function to finish the communication.
//
void _DS1302_stop(void) {
  // Set CE low
  digitalWrite( DS1302_CE_PIN, LOW);
  delayMicroseconds( 4);           // tCWH = 4us
}

// --------------------------------------------------------
// _DS1302_toggleread
//
// A helper function for reading a byte with bit toggle
//
// This function assumes that the SCLK is still high.
//
uint8_t _DS1302_toggleread( void)
{
  uint8_t i, data;

  data = 0;
  for( i = 0; i <= 7; i++)
  {
    // Issue a clock pulse for the next databit.
    // If the 'togglewrite' function was used before 
    // this function, the SCLK is already high.
    digitalWrite( DS1302_SCLK_PIN, HIGH);
    delayMicroseconds( 1);

    // Clock down, data is ready after some time.
    digitalWrite( DS1302_SCLK_PIN, LOW);
    delayMicroseconds( 1);        // tCL=1000ns, tCDD=800ns

    // read bit, and set it in place in 'data' variable
    bitWrite( data, i, digitalRead( DS1302_IO_PIN)); 
  }
  return( data);
}

// --------------------------------------------------------
// _DS1302_togglewrite
//
// A helper function for writing a byte with bit toggle
//
// The 'release' parameter is for a read after this write.
// It will release the I/O-line and will keep the SCLK high.
//
void _DS1302_togglewrite( uint8_t data, uint8_t release)
{
  int i;

  for( i = 0; i <= 7; i++)
  { 
    // set a bit of the data on the I/O-line
    digitalWrite( DS1302_IO_PIN, bitRead(data, i));  
    delayMicroseconds( 1);     // tDC = 200ns

    // clock up, data is read by DS1302
    digitalWrite( DS1302_SCLK_PIN, HIGH);     
    delayMicroseconds( 1);     // tCH = 1000ns, tCDH = 800ns

    if (release && i == 7) {
      pinMode( DS1302_IO_PIN, INPUT);
    } else {
      digitalWrite( DS1302_SCLK_PIN, LOW);
      delayMicroseconds( 1);       // tCL=1000ns, tCDD=800ns
    }
  }
}

