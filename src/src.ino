/*  Energy monitor for 3-phase 

    Based on a single phase energy diverter by Martin Roberts 2/12/12, which itself was
    based on emonTx hardware from OpenEnergyMonitor http://openenergymonitor.org/emon/
    this version implements a phase-locked loop to synchronise to the mains supply and
    supports a single Dallas DS18B20 temperature sensor.
    
    Temp fault codes: 300 deg = Sensor has never been detected since power-up/reset. 
                      302 deg = Sensor returned an out-of-range value. 
                      304 deg = Faulty sensor, sensor broken or disconnected.
                      85  deg   although in range, might indicate a wiring fault.


                      85  deg   although in range, might indicate a wiring fault.

    Documented by Robert Wall http://openenergymonitor.org/emon/pvdiversion/pll
 
    Three-phase energy monitor
    V 1.0   10/12/17 The original extensively modified with diverter code removed
                     and extended for 3-phase operation. 
    V 1.1   20/02/18 Sleep (sleep_mode()) removed from rfm_sleep() in rfm.ino
    V 1.2   12/03/18 Temperature fault codes were
                      300 deg = Faulty sensor, sensor broken or disconnected.
                      301 deg = Sensor has never been detected since power-up/reset. 
                      302 deg = Sensor returned an out-of-range value. 
    V 1.3   30/07/18 TxShield functionality improved by DBates, including EEPROM signature checking.
    
                     
    History (single Phase energy diverter):
    2/12/12  first published version
    3/12/12  diverted power calculation & transmission added
    4/12/12  manual power input added for testing
    10/12/12 high & low energy thresholds added to reduce flicker
    12/10/13 PB added 3rd CT channel to determine diverted power
    09/09/14 EmonTx v3 option added by PB ( http://openenergymonitor.org/emon/node/5714 )


    emonhub.conf node decoder settings for this sketch:

    [[11]]
        nodename = emonTx_three_phase
        firmware = three_phase
        hardware = emonTx V3.2/V3.4/Shield
    [[[rx]]]
        names = powerL1, powerL2, powerL3, power4, Vrms, temp1, temp2, temp3, temp4, temp5, temp6, pulsecount
        datacodes = h, h, h, h, h, h, h, h, h, h, h, L
        scales = 1,1,1,1,0.01,0.01,0.01,0.01,0.01,0.01,0.01,1
        units =W,W,W,W,V,C,C,C,C,C,C,p
    
    [Note: Only one temperature sensor may be connected. All remaining temperatures will read "301.00"]

    For serial input, emonHub requires "datacode = 0" in place of "datacodes = ...." as above. ]
    
*/
const int version = 11;                          // The firmware version 1.10

#define EMONTX_SHIELD                            // Sets the I/O pin allocation. 
                                                 // use EMONTX_V2 or EMONTX_V32 or EMONTX_V34 or EMONTX_SHIELD as appropriate
                                                 // NOTE: You must still set the correct calibration coefficients and check values below.

//--------------------------------------------------------------------------------------------------
// #define DEBUGGING                             // enable this line to include debugging print statements
                                                 //  This is turned off when SERIALOUT or EMONESP (see below) is defined.

#define SERIALPRINT                              // include 'human-friendly' print statement for commissioning - comment this line to exclude.

// Pulse counting settings
//#define USEPULSECOUNT                            // include the ability to count pulses. Comment this line if pulse counting is not required.
#define PULSEINT 1                               // Interrupt no. for pulse counting: EmonTx V2 = 0, EmonTx V3 = 1, EmonTx Shield - see Wiki
#define PULSEPIN 3                               // Interrupt input pin: EmonTx V2 = 2, EmonTx V3 = 3, EmonTx Shield - see Wiki
#define PULSEMINPERIOD 110                       // minimum period between pulses (ms) - default pulse output meters = 100ms
                                                 //   Set to 0 for electronic sensor with solid-state output.
                                                 
// RFM settings                                  // THIS SKETCH WILL NOT WORK WITH THE RFM12B radio.
#define RFM69CW                                  // The type of Radio Module, or none.
//#define SERIALOUT                                // Can be RFM69CW 
//#define EMONESP                                  //   or SERIALOUT if a wired serial connection is used 
                                                 //   or EMONESP if an ESP WiFi module is used
                                                 //     (see http://openenergymonitor.org/emonnode/3872) 
                                                 //   or don't define anything if neither radio nor serial connection is required - in which case 
                                                 //      the IDE serial monitor output will be for information and debugging only.
                                                 // The sketch will hang if the wrong radio module is specified, or if one is specified and not fitted.
                                                 // For all serial output, the maximum is 9600 baud. The emonESP module must be set to suit.
                                                 
#undef RF12_433MHZ
#undef RF12_868MHZ
#undef RF12_915MHZ                               // Should not be present, but can cause problems if they are.

#define RF12_433MHZ                              // Frequency of RFM module can be 
                                                 //    RF12_433MHZ, RF12_868MHZ or RF12_915MHZ. 
                                                 //  You should use the one matching the module you have.
                                                 //  (Note: this is different from the normal OEM definition.)

#define RFPWR 0x99                               // Transmitter power: 0x80 = -18 dBm (min) - 0x9F = +13 dBm (max)
                                                 //    0x99 - RFM12B equivalent
                                                 //    A 5 V supply is required for the emonTx V3.4 versions prior to V3.4.4 if power is set 
                                                 //    significantly above the minimum.

int nodeID = 11;                                 //  node ID for this emonTx. Or nodeID-1 if DIP switch 1 is ON.
int networkGroup = 210;                          //  wireless network group
                                                 //  - needs to be same as emonBase and emonGLCD. OEM default is 210

#define LOOPTIME 5000     // time of outer loop in milliseconds, also time between data transmissions

//--------------------------------------------------------------------------------------------------

//--------------------------------------------------------------------------------------------------
// constants which must be set individually for each system


double vCal = 268.97;     // calculated value is 240:11.6 for UK transformer x 13:1 for resistor divider = 268.97
                          //   for the EU adapter use 250, for the USA adapter use 130.00

#define VCAL_EU 250.00;     // can use DIP switch 2 to set this as the starting value.


#if defined(EMONTX_V2) || defined(EMONTX_V32) || defined(EMONTX_V34)
double i1Cal = 90.91;     // calculated value is 100A:0.05A for transformer / 22 Ohms for resistor = 90.91
double i2Cal = 90.91;     // calculated value is 100A:0.05A for transformer / 22 Ohms for resistor = 90.91
double i3Cal = 90.91;     // calculated value is 100A:0.05A for transformer / 22 Ohms for resistor = 90.91
double i4Cal = 16.67;     // calculated value is 100A:0.05A for transformer / 120 Ohms for resistor
double i1Lead = 2.00;     // degrees that the v.t. phase error leads the c.t.1 phase error by
double i2Lead = 2.00;     // degrees that the v.t. phase error leads the c.t.2 phase error by
double i3Lead = 2.00;     // degrees that the v.t. phase error leads the c.t.3 phase error by
double i4Lead = 0.20;     // degrees that the v.t. phase error leads the c.t.4 phase error by
#endif

#if defined(EMONTX_SHIELD)
double i1Cal = 60.6;      // calculated value is 100A:0.05A for transformer / 33 Ohms for resistor = 60.6
double i2Cal = 60.6;      // calculated value is 100A:0.05A for transformer / 33 Ohms for resistor = 60.6
double i3Cal = 60.6;      // calculated value is 100A:0.05A for transformer / 33 Ohms for resistor = 60.6
double i4Cal = 60.6;      // calculated value is 100A:0.05A for transformer / 33 Ohms for resistor = 60.6
double i1Lead = 2.00;     // degrees that the v.t. phase error leads the c.t.1 phase error by
double i2Lead = 2.00;     // degrees that the v.t. phase error leads the c.t.2 phase error by
double i3Lead = 2.00;     // degrees that the v.t. phase error leads the c.t.3 phase error by
double i4Lead = 2.00;     // degrees that the v.t. phase error leads the c.t.4 phase error by
#endif

//#define CT4Phase PHASE1   // either PHASE1, PHASE2 or PHASE3 to attach c.t.4 to a phase, or comment this line 
                          //  if c.t.4 is not used (See also NUMSAMPLES below)
//#define LEDISLOCK         // comment this out for LED pulsed during transmission
                          //  otherwise LED occults, but that is not easily visible
//--------------------------------------------------------------------------------------------------

//--------------------------------------------------------------------------------------------------
// other system constants
#if defined(EMONTX_V2) || defined(EMONTX_V32) || defined(EMONTX_V34) //EMONTX_V2 or EMONTX_V32 or EMONTX_V34
#define SUPPLY_VOLTS 3.3   // used here because it's more accurate than the internal band-gap reference.
#endif
#ifdef EMONTX_SHIELD
#define SUPPLY_VOLTS 5.0  // Important note regarding the TxShield:
 // A measurement error will very likely derive from using different USB power supplies! This number in reality could be anything from 4.8 to 5.1V depending on the supply.
 // This will significantly alter all measured readings, as the shield uses the Vcc line as the ADC top reference, and sets voltage divider offsets for the input channels..
 // An approximate value of SUPPLY_VOLTS for an Uno + TxShield will be 5.00V minus 0.03V as there's a voltage drop accross the 8ohm current limiting input resistor.
#endif
#define SUPPLY_FREQUENCY 50
#define NUMSAMPLES 45     // number of times to sample each 50/60Hz cycle - must be a multiple of 3
                          // Permissible maximum values (serial only) 50 Hz, 3 c.t: 45         60 Hz, 3 c.t: 36
                          //                                          50 Hz, 4 c.t: 36         60 Hz, 4 c.t: 33
#define ADC_BITS 10       // ADC Resolution
#define ADC_RATE 64       // Time between successive ADC conversions in microseconds


#define PLLTIMERRANGE 100 // PLL timer range limit ~ +/-0.5Hz
#define PLLLOCKRANGE 40   // allowable ADC range to enter locked state
#define PLLUNLOCKRANGE 80 // allowable ADC range to remain locked
#define PLLLOCKCOUNT 100  // number of cycles to determine if PLL is locked

//--------------------------------------------------------------------------------------------------
//
//   Users should not need to change anything below here
//
//
//--------------------------------------------------------------------------------------------------

//--------------------------------------------------------------------------------------------------
// I/O pins for debugging
// Define these only for testing. On the emonTx V3.x, it may be necessary to comment out and
//  remove external connections to the interrupt pin, the One-wire or DS18B20 power pins.
//#define SYNCPIN 19  // this output will be a 50Hz square wave locked to the 50Hz input
//#define SAMPPIN 19  // this output goes high each time an ADC conversion starts or completes
//#define TXPIN 3     // this output goes high each time a radio transmission takes place
//--------------------------------------------------------------------------------------------------

// Arduino I/O pin usage
#if defined(EMONTX_V2)
// EmonTx v2 Pin references
#undef CT4Phase
#define VOLTSPIN 2
#define CT1PIN 3
#define CT2PIN 0
#define CT3PIN 1
#define LEDPIN 9
#define RFMSELPIN 10
#define RFMIRQPIN 2
#define SDOPIN 12
#define W1PIN 4         // 1-Wire pin for temperature

#elif defined(EMONTX_V32)
// EmonTx v3.2 Pin references
#define VOLTSPIN 0
#define CT1PIN 1
#define CT2PIN 2
#define CT3PIN 3
#define CT4PIN 4
#define LEDPIN 6
#define RFMSELPIN 4     // Pins for the RFM Radio module
#define RFMIRQPIN 3
#define SDOPIN 12
#define W1PIN 5         // 1-Wire pin for temperature
#define DS18B20_PWR 19  // Power for 1-wire temperature sensor

#elif defined EMONTX_SHIELD

// EmonTx Shield Pin references
#define VOLTSPIN 0
#define CT1PIN 1
#define CT2PIN 2
#define CT3PIN 3
#define CT4PIN 4
#define LEDPIN 9
#define RFMSELPIN 5   // See Wiki
#define RFMIRQPIN 2   // See Wiki
#define SDOPIN 12
#define W1PIN 4       // 1-Wire pin for temperature

#else
// EmonTx v3.4 Pin references
#define VOLTSPIN 0
#define CT1PIN 1
#define CT2PIN 2
#define CT3PIN 3
#define CT4PIN 4
#define LEDPIN 6
#define RFMSELPIN 10    // Pins for the RFM Radio module
#define RFMIRQPIN 2
#define SDOPIN 12
#define W1PIN 5         // 1-Wire pin for temperature
#define DS18B20_PWR 19  // Power for 1-wire temperature sensor
#define DIP_SWITCH1 8   // Voltage selection 230 / 110 V AC (switch off = 230V)  - with switch off, D8 is HIGH from internal pullup [Not used]
#define DIP_SWITCH2 9   // RF node ID (off = no change in node ID, switch on = nodeID -1) with switch off, D9 is HIGH from internal pullup

#endif
//--------------------------------------------------------------------------------------------------

//--------------------------------------------------------------------------------------------------
// constants calculated at compile time
#define PHASE1 0                                 // No delay for the Phase 1 voltage
#define PHASE2 (NUMSAMPLES/3)                    // Delay for the Phase 2 voltage 
#define PHASE3 (NUMSAMPLES*2/3)                  // Delay for the Phase 3 voltage
#define PHASE4 CT4Phase
#define BUFFERSIZE (PHASE3 + 2)                  // Store a little more than 240 degrees of voltage samples

#define ADC_COUNTS (1 << ADC_BITS)               // ADC Resolution in steps
#define SAMPLERATE (360.0 / NUMSAMPLES)          // Sample Rate in degrees


#define TIMERTOP (((1000000/SUPPLY_FREQUENCY/NUMSAMPLES)*16)-1) // terminal count for PLL timer
#define PLLTIMERMAX (TIMERTOP+PLLTIMERRANGE)
#define PLLTIMERMIN (TIMERTOP-PLLTIMERRANGE)
//--------------------------------------------------------------------------------------------------
  
//--------------------------------------------------------------------------------------------------

// Dallas DS18B20 commands
#define SKIP_ROM 0xCC 
#define CONVERT_TEMPERATURE 0x44
#define READ_SCRATCHPAD 0xBE
#define UNUSED_TEMPERATURE 30000                // this value (300C) is sent if no sensor has ever been detected
#define OUTOFRANGE_TEMPERATURE 30200            // this value (302C) is sent if the sensor reports < -55C or > +125C
#define BAD_TEMPERATURE 30400                   // this value (304C) is sent if no sensor is present or the checksum is bad (corrupted data)
#define TEMP_RANGE_LOW -5500
#define TEMP_RANGE_HIGH 12500
#define MAXONEWIRE 6                             // Max number of temperature sensors 
                                                 //  - 6 for compatibility, only one can be used.

//--------------------------------------------------------------------------------------------------

//--------------------------------------------------------------------------------------------------
// Pulse counting
volatile byte pulses = 0;
unsigned long pulseTime = 0;                     // Record time of interrupt pulse
const byte PulseMinPeriod = PULSEMINPERIOD;      // minimum period between pulses (ms)
//--------------------------------------------------------------------------------------------------

//--------------------------------------------------------------------------------------------------
#ifdef EMONESP
#undef SERIALPRINT                               // Must not corrupt serial output to emonHub with 'human-friendly' printout
#undef SERIALOUT
#undef DEBUGGING
#endif

#if defined SERIALOUT
#undef EMONESP
#undef SERIALPRINT                               // Must not corrupt serial output to emonHub with 'human-friendly' printout
#undef DEBUGGING
#endif
//--------------------------------------------------------------------------------------------------

#include <Wire.h>
#include <SPI.h>
#include <util/crc16.h>
#include <OneWire.h>

typedef struct { int power1, power2, power3, power4, Vrms, temp[MAXONEWIRE] = {30100,30100,30100,30100,30100,30100}; unsigned long pulseCount; } PayloadTx; 
PayloadTx emontx;


// Intermediate constants
double v_ratio, i1_ratio, i2_ratio, i3_ratio, i4_ratio;
double i1phaseshift, i2phaseshift, i3phaseshift, i4phaseshift;

bool firstCycle = true; 

// Accumulated values over 1 cycle - shared between ISR & main program
volatile unsigned long sumVsq, sumI1sq, sumI2sq, sumI3sq, sumI4sq; 
volatile long sumVavg, sumI1avg, sumI2avg, sumI3avg, sumI4avg;
volatile long sumPower1A, sumPower1B, sumPower2A, sumPower2B, sumPower3A, sumPower3B, sumPower4A, sumPower4B; 
volatile unsigned int sumSamples;

// Accumulated values over the reporting period
uint64_t sumPeriodVsq, sumPeriodI1sq, sumPeriodI2sq, sumPeriodI3sq, sumPeriodI4sq;
int64_t sumPeriodVavg, sumPeriodI1avg, sumPeriodI2avg, sumPeriodI3avg, sumPeriodI4avg;
int64_t sumPeriodPower1A, sumPeriodPower1B, sumPeriodPower2A, sumPeriodPower2B, sumPeriodPower3A, sumPeriodPower3B, sumPeriodPower4A, sumPeriodPower4B; 
unsigned long sumPeriodSamples;

double removeRMSOffset(uint64_t sumSquared, int64_t sum, unsigned long numSamples);
double removePowerOffset(uint64_t power, int64_t sumV, int64_t sumI, unsigned long numSamples);
double x1, x2, x3, x4, y1, y2, y3, y4;  // phase shift coefficients
double applyPhaseShift(double phaseShift, double sampleRate, double A, double B);
double deg_rad(double a);

float Vrms, I1rms, I2rms, I3rms, I4rms;
long sumTimerCount;
float realPower1,apparentPower1,powerFactor1;
float realPower2,apparentPower2,powerFactor2;
float realPower3,apparentPower3,powerFactor3;
float realPower4,apparentPower4,powerFactor4;
float frequency;
volatile word timerCount=TIMERTOP;
volatile word pllUnlocked=PLLLOCKCOUNT;
word sumCycleCount;
volatile boolean newsumCycle;
unsigned long nextTransmitTime;
bool rfmXmit = false;

OneWire oneWire(W1PIN);


void setup()
{
  #if defined(DS18B20_PWR)
  pinMode(DS18B20_PWR, OUTPUT);
  digitalWrite(DS18B20_PWR, HIGH);
  #endif
  pinMode(LEDPIN, OUTPUT);
  digitalWrite(LEDPIN, HIGH);
  #ifdef EMONTX_V34
  //READ DIP SWITCH 1 POSITION 
  pinMode(DIP_SWITCH1, INPUT_PULLUP);
  if (digitalRead(DIP_SWITCH1)==LOW) 
    nodeID++;  //If DIP switch 1 is switched on then add 1 to the nodeID
  //READ DIP SWITCH 2 POSITION 
  pinMode(DIP_SWITCH2, INPUT_PULLUP);
  if (digitalRead(DIP_SWITCH2)==LOW) 
    vCal = VCAL_EU;  //If DIP switch 2 is switched on then start with calibration for EU a.c. adapter
  #endif
  #ifdef SYNCPIN
  pinMode(SYNCPIN, OUTPUT);
  digitalWrite(SYNCPIN, LOW);
  #endif
  #ifdef SAMPPIN
  pinMode(SAMPPIN, OUTPUT);
  digitalWrite(SAMPPIN, LOW);
  #endif
  pinMode (RFMSELPIN, OUTPUT);
  digitalWrite(RFMSELPIN,HIGH);

  
  if (true) 
      for (byte i=0; i<4; i++)
          {
              digitalWrite(LEDPIN, LOW); delay(200); digitalWrite(LEDPIN, HIGH); delay(200);
          }
    
 // start the SPI library:
  SPI.begin();
  SPI.setBitOrder(MSBFIRST);
  SPI.setDataMode(0);
  SPI.setClockDivider(SPI_CLOCK_DIV8);
  // initialise RFM12 / RFM69
  delay(200); // wait for RFM12 POR
  #if (defined RFM12B || defined RFM69CW)
    rfm_init();
  #endif
  
  #ifdef USEPULSECOUNT
  pinMode(PULSEPIN, INPUT_PULLUP);               // Set interrupt pulse counting pin as input
  attachInterrupt(PULSEINT, onPulse, RISING);    // Attach pulse counting interrupt pulse counting
  #endif
  emontx.pulseCount=0;                           // Make sure pulse count starts at zero
    
  Serial.begin(9600);                            // Do NOT set greater than 9600
   
  Serial.println(F("OpenEnergyMonitor.org"));
  #if !defined SERIALOUT && !defined EMONESP
   #ifdef EMONTX_V2
     Serial.print(F("emonTx V2"));
   #endif
   #ifdef EMONTX_V32
     Serial.print(F("emonTx V3.2"));
   #endif
   #ifdef EMONTX_V34
     Serial.print(F("emonTx V3.4"));
   #endif
   #ifdef EMONTX_SHIELD    
     Serial.print(F("emonTx Shield"));
   #endif    
   Serial.print(F(" CT1234 Voltage 3 Phase PLL example - Firmware version "));
   Serial.println(version/10.0);


   #ifdef RFM69CW
     Serial.println(F("Using RFM69CW Radio"));
   #endif
   #ifdef SERIALOUT
     Serial.println(F("Using wired serial output"));
   #endif
   #ifdef EMONESP
     Serial.println(F("Using ESP8266 serial output"));
   #endif
   load_config(true);                                 // Load RF config from EEPROM (if any exists)
  #else   // #if !defined SERIALOUT && !defined EMONESP 
   load_config(false);   
  #endif  // #if !defined SERIALOUT && !defined EMONESP 
  
  #if !defined SERIALOUT && !defined EMONESP
   Serial.print(F("Network: ")); 
   Serial.println(networkGroup);
  
   Serial.print(F("Node: ")); 
   Serial.print(nodeID); 

   Serial.print(F(" Freq: ")); 
   #ifdef RF12_868MHZ
     Serial.println(F("868MHz"));
   #elif defined RF12_915MHZ    
     Serial.println(F("915MHz"))
   #else // default to 433 MHz
     Serial.println(F("433MHz"));
   #endif

   readInput();                                   // Read new RF config and send to EEPROM (if desired)

  #endif  // #if !defined SERIALOUT && !defined EMONESP 
  
  calculateTiming();
  calculateConstants();

  nextTransmitTime=millis();
 
  convertTemperature(); // start initial temperature conversion

  
  #ifdef DEBUGGING
    Serial.println(F("Phase shift coefficients:"));
    Serial.print(F("x1 = "));Serial.print(x1);Serial.print(F("  y1 = "));Serial.println(y1);
    Serial.print(F("x2 = "));Serial.print(x2);Serial.print(F("  y2 = "));Serial.println(y2);
    Serial.print(F("x3 = "));Serial.print(x3);Serial.print(F("  y3 = "));Serial.println(y3);
    Serial.print(F("x4 = "));Serial.print(x4);Serial.print(F("  y4 = "));Serial.println(y4);
      
  #endif
  
  
  
  // change ADC prescaler to /64 = 250kHz clock
  // slightly out of spec of 200kHz but should be OK
  ADCSRA &= 0xf8;  // remove bits set by Arduino library
  ADCSRA |= 0x06; 

  //set timer 1 interrupt for required sumPeriod
  noInterrupts();
  TCCR1A = 0; // clear control registers
  TCCR1B = 0;
  TCNT1  = 0; // clear counter
  OCR1A = TIMERTOP; // set compare reg for timer sumPeriod
  bitSet(TCCR1B,WGM12); // CTC mode
  bitSet(TCCR1B,CS10); // no prescaling
  bitSet(TIMSK1,OCIE1A); // enable timer 1 compare interrupt
  bitSet(ADCSRA,ADIE); // enable ADC interrupt
  interrupts();
}

void loop()
{
  getCalibration();
  
  if(newsumCycle && !firstCycle)
    addsumCycle(); // a new mains sumCycle has been sampled
  firstCycle = false;
  
  if((millis()>=nextTransmitTime) && ((millis()-nextTransmitTime)<0x80000000L)) // check for overflow
  {
    #ifndef LEDISLOCK
      digitalWrite(LEDPIN,HIGH);
    #else
      digitalWrite(LEDPIN,LOW);
    #endif
    calculateVIPF();
    emontx.temp[0]=readTemperature();
    if (pulses)                                      // if the ISR has counted some pulses, update the total count
    {
        cli();                                       // Disable interrupt just in case a pulse comes in while we are updating the count
        emontx.pulseCount += pulses;
        pulses = 0;
        sei();                                       // Re-enable interrupts
    }

    sendResults();
    convertTemperature(); // start next conversion
    nextTransmitTime+=LOOPTIME;
    #ifndef LEDISLOCK
      digitalWrite(LEDPIN,LOW);
    #else
      digitalWrite(LEDPIN,HIGH);
    #endif
  }
}

// timer 1 interrupt handler
ISR(TIMER1_COMPA_vect)
{
  #ifdef SAMPPIN
  digitalWrite(SAMPPIN,HIGH);
  #endif
  ADMUX = _BV(REFS0) | CT1PIN; // start ADC conversion for first current
  ADCSRA |= _BV(ADSC);
  #ifdef SAMPPIN
  digitalWrite(SAMPPIN,LOW);
  #endif
}

// ADC interrupt handler
ISR(ADC_vect)
{
  static int newV, lastV, sampleI1, sampleI2, sampleI3, sampleI4;
  static int storedV[BUFFERSIZE];  // Array to store >240 degrees of voltage samples
  int result;
  static int Vindex = 0;
  
  #ifdef SAMPPIN
  digitalWrite(SAMPPIN,HIGH);
  #endif
  result = ADCL;
  result |= ADCH<<8;
  // remove the nominal offset 
  result -=(ADC_COUNTS >> 1);
  // determine which conversion just completed
  switch(ADMUX & 0x0f)
  {
    case CT1PIN:
      ADMUX = _BV(REFS0) | CT2PIN; // start CT2 conversion
      ADCSRA |= _BV(ADSC);
      sampleI1 = result;
      sumI1sq += (long)sampleI1 * sampleI1;
      sumI1avg += sampleI1;
      break;
    case CT2PIN:
      ADMUX = _BV(REFS0) | CT3PIN; // start CT3 conversion
      ADCSRA |= _BV(ADSC);
      sampleI2 = result;
      sumI2sq += (long)sampleI2 * sampleI2;
      sumI2avg += sampleI2; 
     break;
    case CT3PIN:
    #ifdef CT4Phase
      ADMUX = _BV(REFS0) | CT4PIN; // start CT4 conversion
    #else
      ADMUX = _BV(REFS0) | VOLTSPIN; // start Voltage conversion        
    #endif
      ADCSRA |= _BV(ADSC);
      sampleI3 = result;
      sumI3sq += (long)sampleI3 * sampleI3;
      sumI3avg += sampleI3; 
      break;
    #ifdef CT4Phase
    case CT4PIN:
      ADMUX = _BV(REFS0) | VOLTSPIN; // start Voltage conversion
      ADCSRA |= _BV(ADSC);
      sampleI4 = result;
      sumI4sq += (long)sampleI4 * sampleI4;
      sumI4avg += sampleI4; 
      break;
    #endif
    case VOLTSPIN:
      lastV=newV;
      newV = result;
      storedV[Vindex] = newV;        // store this voltage sample in circular buffer
      sumVsq += ((long)newV * newV);
      sumVavg += newV; 
      sumPower1A += (long)newV * sampleI1;
      sumPower1B += (long)lastV * sampleI1;
      sumPower2A += (long)storedV[(Vindex+BUFFERSIZE-PHASE2)%BUFFERSIZE] * sampleI2;  // Use stored & delayed voltage for power calculation phase 2
      sumPower2B += (long)storedV[(Vindex+BUFFERSIZE-PHASE2-1)%BUFFERSIZE] * sampleI2;
      sumPower3A += (long)storedV[(Vindex+BUFFERSIZE-PHASE3)%BUFFERSIZE] * sampleI3;  // Use stored & delayed voltage for power calculation phase 3
      sumPower3B += (long)storedV[(Vindex+BUFFERSIZE-PHASE3-1)%BUFFERSIZE] * sampleI3;
#ifdef CT4Phase
      sumPower4A += (long)storedV[(Vindex+BUFFERSIZE-PHASE4)%BUFFERSIZE] * sampleI4;  // Align c.t. 4 to any phase as required
      sumPower4B += (long)storedV[(Vindex+BUFFERSIZE-PHASE4-1)%BUFFERSIZE] * sampleI4;
#endif      
      sumSamples++;
      updatePLL(newV,lastV);
      ++Vindex %= BUFFERSIZE;    
      break;
      
  }
  #ifdef SAMPPIN
  digitalWrite(SAMPPIN,LOW);
  #endif

}

void updatePLL(int newV, int lastV)
{
  static byte samples=0;
  static int oldV;
  boolean rising;

  rising=(newV>lastV); // synchronise to rising zero crossing
  
  samples++;
  if(samples>=NUMSAMPLES) // end of one 50Hz sumCycle
  {
    #ifdef SYNCPIN
    digitalWrite(SYNCPIN,HIGH);
    #endif
    samples=0;
    if(rising)
    {
      // if we're in the rising part of the 50Hz sumCycle adjust the final timer count
      // to move newV towards 0, only adjust if we're moving in the wrong direction
      if(((newV<0)&&(newV<=oldV))||((newV>0)&&(newV>=oldV))) timerCount-=newV;
      // limit range of PLL frequency
      timerCount=constrain(timerCount,PLLTIMERMIN,PLLTIMERMAX);
      OCR1A=timerCount;
      if(abs(newV)>PLLUNLOCKRANGE) pllUnlocked=PLLLOCKCOUNT; // we're unlocked
      else if(pllUnlocked) pllUnlocked--;
      #ifdef LEDISLOCK
        digitalWrite(LEDPIN,pllUnlocked?LOW:HIGH);
      #endif
    }
    else // in the falling part of the sumCycle, we shouldn't be here
    {
      OCR1A=PLLTIMERMAX; // shift out of this region fast
      pllUnlocked=PLLLOCKCOUNT; // and we can't be locked
    }
    
    oldV=newV;
    
    newsumCycle=true; // flag new sumCycle to outer loop
  }
  else if(samples==(NUMSAMPLES/2))
  {
    // negative zero crossing
    #ifdef SYNCPIN
    digitalWrite(SYNCPIN,LOW);
    #endif
  }
  #ifdef SAMPPIN
  digitalWrite(SAMPPIN,LOW);
  #endif
}

// add data for new 50Hz sumCycle to total for the period (called from loop() )
void addsumCycle()
{
  // save results for outer loop
  noInterrupts();
  sumPeriodVsq      += sumVsq;
  sumPeriodVavg     += sumVavg;
  sumPeriodI1sq     += sumI1sq;
  sumPeriodI1avg    += sumI1avg; 
  sumPeriodI2sq     += sumI2sq;
  sumPeriodI2avg    += sumI2avg; 
  sumPeriodI3sq     += sumI3sq;
  sumPeriodI3avg    += sumI3avg; 
  sumPeriodI4sq     += sumI4sq;
  sumPeriodI4avg    += sumI4avg; 

  sumPeriodPower1A  += sumPower1A;
  sumPeriodPower1B  += sumPower1B;
  sumPeriodPower2A  += sumPower2A;
  sumPeriodPower2B  += sumPower2B;
  sumPeriodPower3A  += sumPower3A;
  sumPeriodPower3B  += sumPower3B;
  sumPeriodPower4A  += sumPower4A;
  sumPeriodPower4B  += sumPower4B;
  sumPeriodSamples  += sumSamples;

  sumVsq	    = 0;
  sumVavg	    = 0;
  sumI1sq	    = 0;
  sumI1avg	    = 0; 
  sumI2sq	    = 0;
  sumI2avg	    = 0; 
  sumI3sq	    = 0;
  sumI3avg	    = 0; 
  sumI4sq       = 0;
  sumI4avg	    = 0; 
      
  sumPower1A	= 0;
  sumPower1B	= 0;
  sumPower2A	= 0;
  sumPower2B	= 0;
  sumPower3A	= 0;
  sumPower3B	= 0;
  sumPower4A	= 0;
  sumPower4B	= 0;
  sumSamples	= 0;

  sumTimerCount+=(timerCount+1); // for average frequency calculation
  sumCycleCount++;
  newsumCycle=false;
  interrupts();
}


double removeRMSOffset(uint64_t sumSquared, int64_t sum, unsigned long numSamples)
{
    double x = ((double)sumSquared / numSamples) - ((double)sum * sum / numSamples / numSamples);
    return (x<0.0 ? 0.0 : sqrt(x));
}

double removePowerOffset(int64_t power, int64_t sumV, int64_t sumI, unsigned long numSamples)
{
    return (((double)power / numSamples) - ((double)sumV * sumI / numSamples / numSamples));
}

double deg_rad(double a)
{
    return (0.01745329*a);
}

double applyPhaseShift(double phaseShift, double sampleRate, double A, double B)
{
  double y = sin(deg_rad(phaseShift)) / sin(deg_rad(sampleRate));
  double x = cos(deg_rad(phaseShift)) - y * cos(deg_rad(sampleRate));
  return (A * x + B * y); 
}


// calculate voltage, current, power and frequency
void calculateVIPF()
{  
  if(sumPeriodSamples==0) return; // just in case
  
  frequency=((float)sumCycleCount*16000000)/(((float)sumTimerCount)*NUMSAMPLES);

  // rms values - voltage & current
  
  // Vrms still contains the fine voltage offset. Correct this by subtracting the "Offset V^2" before the sq. root.
  Vrms = v_ratio * removeRMSOffset(sumPeriodVsq, sumPeriodVavg, sumPeriodSamples);
   
  // Similarly the 4 currents
  I1rms = i1_ratio * removeRMSOffset(sumPeriodI1sq, sumPeriodI1avg, sumPeriodSamples);
  I2rms = i2_ratio * removeRMSOffset(sumPeriodI2sq, sumPeriodI2avg, sumPeriodSamples);
  I3rms = i3_ratio * removeRMSOffset(sumPeriodI3sq, sumPeriodI3avg, sumPeriodSamples);
  #ifdef CT4Phase
  I4rms = i4_ratio * removeRMSOffset(sumPeriodI4sq, sumPeriodI4avg, sumPeriodSamples);
  #endif

  // Power contains both voltage & current offsets. Correct this by subtracting the "Offset Power": Vavg * Iavg.
  // Apply timing/phase compensation to obtain real power.
  //  real power = Ical * Vcal * (powerA * PHASECAL - powerB * (PHASECAL - 1));
  // or more accurately:
  //  y = sin(phase_shift) / sin(sampleRate);
  //  x = cos(phase_shift) - y * cos(sampleRate);
  // realPower = Ical * Vcal * (powerA * x + powerB * y);
  // [sampleRate] is the angle between sample sets in radians - and is different for 
  //   50 Hz and 60 Hz systems
  // [phase_shift] will vary according to the time delay between the current and voltage samples
  //   as well as the difference in phase leads of the two transformers.
  // x & y have been calculated in setup() as they won't change.

  realPower1 = v_ratio * i1_ratio * (x1 * removePowerOffset(sumPeriodPower1A, sumPeriodVavg, sumPeriodI1avg, sumPeriodSamples) 
                                   + y1 * removePowerOffset(sumPeriodPower1B, sumPeriodVavg, sumPeriodI1avg, sumPeriodSamples));
  apparentPower1 = I1rms * Vrms; 
  if (apparentPower1 > 0.1)        // suppress "nan" values
    powerFactor1 = realPower1 / apparentPower1;
  else
    powerFactor1 = 0.0;
  
  realPower2 = v_ratio * i2_ratio * (x2 * removePowerOffset(sumPeriodPower2A, sumPeriodVavg, sumPeriodI2avg, sumPeriodSamples)
                                   + y2 * removePowerOffset(sumPeriodPower2B, sumPeriodVavg, sumPeriodI2avg, sumPeriodSamples));
  apparentPower2 = I2rms * Vrms; 
  if (apparentPower2 > 0.1) 
    powerFactor2 = realPower2 / apparentPower2;
  else
    powerFactor2 = 0.0;
  realPower3 = v_ratio * i3_ratio * (x3 * removePowerOffset(sumPeriodPower3A, sumPeriodVavg, sumPeriodI3avg, sumPeriodSamples)
                                   + y3 * removePowerOffset(sumPeriodPower3B, sumPeriodVavg, sumPeriodI3avg, sumPeriodSamples));
  apparentPower3 = I3rms * Vrms; 
  if (apparentPower3 > 0.1) 
    powerFactor3 = realPower3 / apparentPower3;
  else
    powerFactor3 = 0.0;
  #ifdef CT4Phase
  realPower4 = v_ratio * i4_ratio * (x4 * removePowerOffset(sumPeriodPower4A, sumPeriodVavg, sumPeriodI4avg, sumPeriodSamples)
                                    +y4 * removePowerOffset(sumPeriodPower4B, sumPeriodVavg, sumPeriodI4avg, sumPeriodSamples));
  apparentPower4 = I4rms * Vrms; 
  if (apparentPower4 > 0.1) 
    powerFactor4 = realPower4 / apparentPower4;
  else
    powerFactor4 = 0.0;
  #endif
  

  emontx.power1=(int)(realPower1+0.5);
  emontx.power2=(int)(realPower2+0.5);
  emontx.power3=(int)(realPower3+0.5);
  emontx.power4=(int)(realPower4+0.5);
  emontx.Vrms=(int)(Vrms*100+0.5);

  sumPeriodVsq      = 0;
  sumPeriodVavg     = 0;
  sumPeriodI1sq     = 0;
  sumPeriodI1avg    = 0;
  sumPeriodI2sq     = 0;
  sumPeriodI2avg    = 0;
  sumPeriodI3sq     = 0;
  sumPeriodI3avg    = 0;
  sumPeriodI4sq     = 0;
  sumPeriodI4avg    = 0;

  sumPeriodPower1A  = 0;
  sumPeriodPower1B  = 0;
  sumPeriodPower2A  = 0;
  sumPeriodPower2B  = 0;
  sumPeriodPower3A  = 0;
  sumPeriodPower3B  = 0;
  sumPeriodPower4A  = 0;
  sumPeriodPower4B  = 0;
  sumPeriodSamples  = 0;

  sumCycleCount=0;
  sumTimerCount=0;

}

void calculateConstants(void)
{
  // Intermediate calculations

  v_ratio = vCal  * SUPPLY_VOLTS / 1024; 
  i1_ratio = i1Cal  * SUPPLY_VOLTS / 1024;
  i2_ratio = i2Cal  * SUPPLY_VOLTS / 1024;
  i3_ratio = i3Cal  * SUPPLY_VOLTS / 1024; 
  i4_ratio = i4Cal  * SUPPLY_VOLTS / 1024; 

  #ifdef CT4Phase
    i1phaseshift = (4 * ADC_RATE * 3.6e-4 * SUPPLY_FREQUENCY - i1Lead); // in degrees
    i2phaseshift = (3 * ADC_RATE * 3.6e-4 * SUPPLY_FREQUENCY - i2Lead);
    i3phaseshift = (2 * ADC_RATE * 3.6e-4 * SUPPLY_FREQUENCY - i3Lead);
    i4phaseshift = (1 * ADC_RATE * 3.6e-4 * SUPPLY_FREQUENCY - i4Lead);
  #else
    i1phaseshift = (3 * ADC_RATE * 3.6e-4 * SUPPLY_FREQUENCY - i1Lead); // in degrees
    i2phaseshift = (2 * ADC_RATE * 3.6e-4 * SUPPLY_FREQUENCY - i2Lead);
    i3phaseshift = (1 * ADC_RATE * 3.6e-4 * SUPPLY_FREQUENCY - i3Lead);
  #endif    

}

void calculateTiming(void)
{
  // Pre-calculate the constants for phase/timing correction
  y1 = sin(deg_rad(i1phaseshift)) / sin(deg_rad(SAMPLERATE));
  x1 = cos(deg_rad(i1phaseshift)) - y1 * cos(deg_rad(SAMPLERATE));
  y2 = sin(deg_rad(i2phaseshift)) / sin(deg_rad(SAMPLERATE));
  x2 = cos(deg_rad(i2phaseshift)) - y2 * cos(deg_rad(SAMPLERATE));
  y3 = sin(deg_rad(i3phaseshift)) / sin(deg_rad(SAMPLERATE));
  x3 = cos(deg_rad(i3phaseshift)) - y3 * cos(deg_rad(SAMPLERATE));
  #ifdef CT4Phase
    y4 = sin(deg_rad(i4phaseshift)) / sin(deg_rad(SAMPLERATE));
    x4 = cos(deg_rad(i4phaseshift)) - y4 * cos(deg_rad(SAMPLERATE));
  #endif
}


void sendResults()
{
  #ifdef RFM69CW    
    rfm_send((byte *)&emontx, sizeof(emontx), networkGroup, nodeID);      // *SEND RF DATA*
  #else

    #ifdef TXPIN
      digitalWrite(TXPIN,HIGH);
      delay(5);
      digitalWrite(TXPIN,LOW);
    #endif
  #endif

  #if defined SERIALOUT && !defined EMONESP
    Serial.print(nodeID);     Serial.print(' ');
    Serial.print((int)(realPower1+0.5)); Serial.print(F(" "));   // These for compatibility, but whatever you need if emonHub is configured to suit. 
    Serial.print((int)(realPower2+0.5)); Serial.print(F(" "));
    Serial.print((int)(realPower3+0.5)); Serial.print(F(" "));
    Serial.print((int)(realPower4+0.5)); Serial.print(F(" "));
    Serial.print((int)(Vrms*100));
    Serial.print(F(" "));
   
    for(byte j=0;j<MAXONEWIRE;j++)
    {
    Serial.print(emontx.temp[j]);
    Serial.print(F(" "));
    }
    Serial.println(emontx.pulseCount);

  #endif  // if defined SERIALOUT && !defined EMONESP

  #if defined EMONESP && !defined SERIALOUT
    Serial.print(F("ct1:")); Serial.print(realPower1);            // These for compatibility, but whatever you need if the receiver is configured to suit. 
    Serial.print(F(",ct2:")); Serial.print(realPower2);
    Serial.print(F(",ct3:")); Serial.print(realPower3);
    Serial.print(F(",ct4:")); Serial.print(realPower4);
    Serial.print(F(",vrms:")); Serial.print(Vrms);

    
    for(byte j=0;j<MAXONEWIRE;j++)
    {
      Serial.print(F(",t")); Serial.print(j+1); Serial.print(F(":"));
      Serial.print(emontx.temp[j]/100.0);
    }
    Serial.print(F(",pulses:"));Serial.print(emontx.pulseCount);
    Serial.println();
    delay(50);
  #endif

  
  #if defined SERIALPRINT && !defined EMONESP
    Serial.print(Vrms);
    Serial.print(F(" "));
    Serial.print(I1rms,3);
    Serial.print(F(" "));
    Serial.print(I2rms,3);
    Serial.print(F(" "));
    Serial.print(I3rms,3);
    Serial.print(F(" "));
    Serial.print(I4rms,3);
    Serial.print(F(" "));
    Serial.print(realPower1);
    Serial.print(F(" "));
    Serial.print(realPower2);
    Serial.print(F(" "));
    Serial.print(realPower3);
    Serial.print(F(" "));
    Serial.print(realPower4);
    Serial.print(F(" "));

    Serial.print(frequency,3);
    Serial.print(F(" "));

    Serial.print(powerFactor1,4);
    Serial.print(F(" "));
    Serial.print(powerFactor2,4);
    Serial.print(F(" "));
    Serial.print(powerFactor3,4);
    Serial.print(F(" "));
    Serial.print(powerFactor4,4);
    Serial.print(F(" "));
    Serial.print((float)emontx.temp[0]/100);

    #ifdef USEPULSECOUNT
      Serial.print(F(" Pulses=")); Serial.print(emontx.pulseCount);
    #endif
    
    Serial.print(F(" "));
    if(pllUnlocked) Serial.print(F(" PLL is unlocked "));
    else Serial.print(F(" PLL is locked "));
    Serial.println();
  #endif
  
}

void convertTemperature()
{
  oneWire.reset();
  oneWire.write(SKIP_ROM);
  oneWire.write(CONVERT_TEMPERATURE);
}

int readTemperature()
{
  byte buf[9];
  int result;
  
  oneWire.reset();
  oneWire.write(SKIP_ROM);
  oneWire.write(READ_SCRATCHPAD);
  for(int i=0; i<9; i++) buf[i]=oneWire.read();
  if(oneWire.crc8(buf,8)==buf[8])
  {
    result=(buf[1]<<8)|buf[0];
    // result is temperature x16, multiply by 6.25 to convert to temperature x100
    result=(result*6)+(result>>2);
  }
  else result=BAD_TEMPERATURE;
  if (result <= TEMP_RANGE_LOW || result >= TEMP_RANGE_HIGH)
    return OUTOFRANGE_TEMPERATURE;     // return value ('Out of range')
  return result;
}
/*
    Temp fault codes: BAD_TEMPERATURE          = Faulty sensor, sensor broken or disconnected.
                      UNUSED_TEMPERATURE       = Sensor has never been detected since power-up/reset. 
                      OUTOFRANGE_TEMPERATURE   = Sensor returned an out-of-range value. 
                      85  deg                    although in range, might indicate a wiring fault.
*/
                      


#ifdef USEPULSECOUNT
//-------------------------------------------------------------------------------------------------------------------------------------------
// The Interrupt Service Routine - runs each time a falling edge of a pulse is detected
//-------------------------------------------------------------------------------------------------------------------------------------------
void onPulse()                  
{
  if (PulseMinPeriod)
  {
    if ((millis() - pulseTime) > PulseMinPeriod) {              // Check that contact bounce has finished
      pulses++;
    }
    pulseTime=millis();                                         // No 'debounce' required - electronic switch presumed 	
  }
  else
    pulses++;					
}

#endif
