// Simple WSPR transmitter
// Bruce MacKinnon KC1FSZ
// 9 January 2018
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

JTEncode jtencode;
DebouncedSwitch debouncedButton(25);

const char call[] = "KC1FSZ";
const char loc[] = "FN42";  // Wellesley and Boston
//const char loc[] = "DM07";  // Wawona
const uint8_t dbm = 27; // 0.5W
const long TWO_MINUTES_MS = 2L * 60L * 1000L;

uint8_t tx_buffer[255];
double freq = WSPR_FREQ;
uint8_t symbol_count = WSPR_SYMBOL_COUNT;
uint16_t tone_delay = WSPR_DELAY;
double tone_spacing = WSPR_TONE_SPACING;

enum Mode { IDLE, INTRA_SYMBOL, SYMBOL };

// State (for state machine)
Mode mode = Mode::IDLE;
// The last time we sent a WSPR message (i.e. the start of the last two minute cycle)
long lastTransmitStamp = millis() - TWO_MINUTES_MS;
// The last time we started a symbol
long lastSymbolStartStamp = 0;
// Pointer through the sequence of symbols
long symbolPtr = 0;

void setup() {

  Serial.begin(9600);
  delay(100);
  Serial.println("KC1FSZ WSPR Beacon - Verson 2");
  Serial.print("Location: ");
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
}

void loop() {
  
  long now = millis();

  if (mode == Mode::IDLE) {
    // Transmission starts every 2 minutes
    if ((now - lastTransmitStamp) > TWO_MINUTES_MS) {
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
    Serial.print(symbolPtr);
    Serial.print(": ");
    double symbolFreq = freq + ((double)tx_buffer[symbolPtr] * tone_spacing);
    Serial.print(tx_buffer[symbolPtr]);
    Serial.print(" ");
    Serial.print(symbolFreq);
    Serial.print("\n");
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
        Serial.println();
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
    // Force the time to now minus two minutes
    lastTransmitStamp = now - TWO_MINUTES_MS;
    digitalWrite(LED0_PIN,0);
    DDS.down();
  }
}

