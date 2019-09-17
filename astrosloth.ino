/*******************************************************************************
  Title: Astrosloth Reflow Controller
  Version: 1.00
  Date: 08-19-2019
  Author: Astrosloth

  Brief
  =====
  The AstroSloth Reflow Oven Controller is an ATMEGA328 microcontroller 
  based system that automates your toaster oven to reflow your DIY SMT PCB projects. 
  With two profiles stored, you can choose between lead-based and SnAgCu based solder.
  The firmware was modified from the original Rocketscream Electronics Tiny Reflow controller code written by Lim Phang Moh.

  Disclaimer
  ==========
  Please note that while the board works with DC voltages, mains power voltages are involved during oven use. 
  High voltages can be lethal and in the worst case may lead to electrocution and possible death. 
  If you are unsure of how to connect the device to mains voltage, please first contact and speak to a licensed electrician who can help you.
  Use of the Astrosloth Reflow Controller is entirely at your own risk, for which we shall not be liable.

  Licences
  ========
  This Tiny Reflow Controller hardware and firmware are released under the
  Creative Commons Share Alike v3.0 license
  http://creativecommons.org/licenses/by-sa/3.0/
  You are free to take this piece of code, use it and modify it.
  All we ask is attribution including the supporting libraries used in this
  firmware.

  Required Libraries
  ==================
  - Arduino PID Library:
    >> https://github.com/br3ttb/Arduino-PID-Library
  - Adafruit MAX31855 Library:
    >> https://github.com/adafruit/Adafruit_MAX31855
  - Adafruit SSD1306 Library:
    >> https://github.com/adafruit/Adafruit_SSD1306
  - Adafruit GFX Library:
    >> https://github.com/adafruit/Adafruit-GFX-Library

  Revision  Description
  ========  ===========
  1.00      Initial public release:
            - Based on ATMega328 5V @ 16MHz
            - Uses SSD1306 128x64 Monochrome display

*******************************************************************************/

// ***** INCLUDES *****
#include <SPI.h>
#include <Wire.h>
#include <EEPROM.h>
#include <LiquidCrystal.h>
#include <Adafruit_GFX.h>      
#include <Adafruit_SSD1306.h>  
#include <Adafruit_MAX31855.h> 
#include <PID_v1.h>


// ***** STATE TYPE DEFINITIONS *****
typedef enum REFLOW_STATE
{
  REFLOW_STATE_IDLE, //orange
  REFLOW_STATE_PREHEAT, //yellow
  REFLOW_STATE_SOAK, //green
  REFLOW_STATE_REFLOW, // bright green
  REFLOW_STATE_COOL, //blue
  REFLOW_STATE_COMPLETE, //indigo
  REFLOW_STATE_TOO_HOT, //violet
  REFLOW_STATE_ERROR //red
} reflowState_t;

typedef enum REFLOW_STATUS
{
  REFLOW_STATUS_OFF,
  REFLOW_STATUS_ON
} reflowStatus_t;

typedef enum SWITCH
{
  SWITCH_NONE,
  SWITCH_1,
  SWITCH_2
} switch_t;

typedef enum DEBOUNCE_STATE
{
  DEBOUNCE_STATE_IDLE,
  DEBOUNCE_STATE_CHECK,
  DEBOUNCE_STATE_RELEASE
} debounceState_t;

typedef enum REFLOW_PROFILE
{
  REFLOW_PROFILE_LEADFREE,
  REFLOW_PROFILE_LEADED
} reflowProfile_t;

// ***** CONSTANTS *****

// ***** GENERAL PROFILE CONSTANTS *****
#define PROFILE_TYPE_ADDRESS 0
#define TEMPERATURE_ROOM 50
#define TEMPERATURE_SOAK_MIN 150
#define TEMPERATURE_COOL_MIN 100
#define SENSOR_SAMPLING_TIME 1000
#define SOAK_TEMPERATURE_STEP 5

// ***** LEAD FREE PROFILE CONSTANTS *****
#define TEMPERATURE_SOAK_MAX_LF 200
#define TEMPERATURE_REFLOW_MAX_LF 250
#define SOAK_MICRO_PERIOD_LF 9000

// ***** LEADED PROFILE CONSTANTS *****
#define TEMPERATURE_SOAK_MAX_PB 180
#define TEMPERATURE_REFLOW_MAX_PB 224
#define SOAK_MICRO_PERIOD_PB 10000

// ***** SWITCH SPECIFIC CONSTANTS *****
#define DEBOUNCE_PERIOD_MIN 100

// ***** DISPLAY SPECIFIC CONSTANTS *****
#define UPDATE_RATE 100
#define SCREEN_WIDTH 128 // display display width, in pixels
#define SCREEN_HEIGHT 64 // display display height, in pixels
#define X_AXIS_START 18 // X-axis starting position

// ***** PID PARAMETERS *****
// ***** PRE-HEAT STAGE *****
#define PID_KP_PREHEAT 100
#define PID_KI_PREHEAT 0.025
#define PID_KD_PREHEAT 20
// ***** SOAKING STAGE *****
#define PID_KP_SOAK 300
#define PID_KI_SOAK 0.05
#define PID_KD_SOAK 250
// ***** REFLOW STAGE *****
#define PID_KP_REFLOW 300
#define PID_KI_REFLOW 0.05
#define PID_KD_REFLOW 350
#define PID_SAMPLE_TIME 1000


// ***** LCD MESSAGES *****
const char* lcdMessagesReflowStatus[] = {
  "Ready",
  "PreHeat",
  "Soak",
  "Reflow",
  "Cool",
  "Done!",
  "Hot!",
  "Error"
};

// ***** DEGREE SYMBOL FOR LCD *****
unsigned char degree[8]  = {
  140, 146, 146, 140, 128, 128, 128, 128
};

// ***** PIN ASSIGNMENT *****

unsigned char ssrPin = 5; 
unsigned char fanPin = A1;
unsigned char buzzerPin = 6; 
unsigned char switchStartStopPin = A0; 
unsigned char switchLfPbPin = A2;
#define ledPin A6 
#define display_MOSI   11
#define display_CLK   13
#define display_DC    10
#define display_CS    8
#define display_RESET 7
#define MAXDO   4
#define MAXCS   2
#define MAXCLK  3


// ***** PID CONTROL VARIABLES *****
double setpoint;
double input;
double output;
double kp = PID_KP_PREHEAT;
double ki = PID_KI_PREHEAT;
double kd = PID_KD_PREHEAT;
int windowSize;
unsigned long windowStartTime;
unsigned long nextCheck;
unsigned long nextRead;
unsigned long updateLcd;
unsigned long timerSoak;
unsigned long buzzerPeriod;
unsigned char soakTemperatureMax;
unsigned char reflowTemperatureMax;
unsigned long soakMicroPeriod;

// Reflow oven controller state machine state variable
reflowState_t reflowState;

// Reflow oven controller status
reflowStatus_t reflowStatus;

// Reflow profile type
reflowProfile_t reflowProfile;

// Switch debounce state machine state variable
debounceState_t debounceState;

// Switch debounce timer
long lastDebounceTime;
// Switch press status
switch_t switchStatus;
switch_t switchValue;

switch_t switchMask;
// Seconds timer
unsigned int timerSeconds;

// Thermocouple fault status
unsigned char fault;
unsigned int timerUpdate;
unsigned char temperature[SCREEN_WIDTH - X_AXIS_START];
unsigned char x;


// PID control interface
PID reflowOvenPID(&input, &output, &setpoint, kp, ki, kd, DIRECT);

//OLED DISPLAY SETUP
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, display_RESET);

// MAX31855 thermocouple interface
Adafruit_MAX31855 thermocouple(MAXCLK, MAXCS, MAXDO);

void setup()
{
  // Check current selected reflow profile
  unsigned char value = EEPROM.read(PROFILE_TYPE_ADDRESS);
  if ((value == 0) || (value == 1))
  {
    // Valid reflow profile value
    reflowProfile = value;
  }
  else
  {
    // Default to lead-free profile
    EEPROM.write(PROFILE_TYPE_ADDRESS, 0);
    reflowProfile = REFLOW_PROFILE_LEADFREE;
  }

  // SSR pin initialization to ensure reflow oven is off
  digitalWrite(ssrPin, LOW);
  pinMode(ssrPin, OUTPUT);

  // Buzzer pin initialization to ensure annoying buzzer is off
  digitalWrite(buzzerPin, LOW);
  pinMode(buzzerPin, OUTPUT);

  // LED pins initialization and turn on upon start-up (active high)

  //pinMode(ledPin, OUTPUT);
  //digitalWrite(ledPin, HIGH);

  // Start-up splash
  digitalWrite(buzzerPin, HIGH);
  display.begin(SSD1306_SWITCHCAPVCC, 0x3D);
  display.display();
  digitalWrite(buzzerPin, LOW);
  delay(2000);
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0, 0);
  display.println(F("     AstroSloth"));
  display.println(F("     Reflow"));
  display.println(F("     Controller"));
  display.println();
  display.println(F("       v1.00"));
  display.println();
  display.println(F("      08-19-19"));
  display.display();
  delay(3000);
  display.clearDisplay();


  // Serial communication at 115200 bps
  Serial.begin(115200);

  // Turn off LED (active high)
  //digitalWrite(ledPin, LOW);
  // Set window size
  windowSize = 2000;
  // Initialize time keeping variable
  nextCheck = millis();
  // Initialize thermocouple reading variable
  nextRead = millis();
  // Initialize LCD update timer
  updateLcd = millis();
}

void loop()
{
  // Current time
  unsigned long currentTime;

  // Time to read thermocouple?
  if (millis() > nextRead)
  {
    // Read thermocouple next sampling period
    nextRead += SENSOR_SAMPLING_TIME;
    // Read current temperature
    input = thermocouple.readCelsius();

    // If any thermocouple fault is detected
    if (isnan(input))
    {
      // Illegal operation
      reflowState = REFLOW_STATE_ERROR;
      reflowStatus = REFLOW_STATUS_OFF;
      Serial.println(F("Error"));
    }
  }

  if (millis() > nextCheck)
  {
    // Check input in the next seconds
    nextCheck += SENSOR_SAMPLING_TIME;
    // If reflow process is on going
    if (reflowStatus == REFLOW_STATUS_ON)
    {
      // Toggle red LED as system heart beat
      //digitalWrite(ledPin, !(digitalRead(ledPin)));
      // Increase seconds timer for reflow curve plot
      timerSeconds++;
      // Send temperature and time stamp to serial
      Serial.print(timerSeconds);
      Serial.print(F(","));
      Serial.print(setpoint);
      Serial.print(F(","));
      Serial.print(input);
      Serial.print(F(","));
      Serial.println(output);
    }
    else
    {
      // Turn off red LED
      //digitalWrite(ledPin, LOW);
    }
  }

  if (millis() > updateLcd)
  {
    // Update LCD in the next 100 ms
    updateLcd += UPDATE_RATE;

    display.clearDisplay();
    display.setTextSize(2);
    display.setCursor(0, 0);
    display.print(lcdMessagesReflowStatus[reflowState]);
    display.setTextSize(1);
    display.setCursor(115, 0);

    if (reflowProfile == REFLOW_PROFILE_LEADFREE)
    {
      display.print(F("LF"));
    }
    else
    {
      display.print(F("PB"));
    }
    
    // Temperature markers
    display.setCursor(0, 18);
    display.print(F("250"));
    display.setCursor(0, 36);
    display.print(F("150"));
    display.setCursor(0, 54);
    display.print(F("50"));
    // Draw temperature and time axis on monochrome display
    display.drawLine(18, 18, 18, 63, WHITE);
    display.drawLine(18, 63, 127, 63, WHITE);
    display.setCursor(115, 0);

    // If currently in error state
    if (reflowState == REFLOW_STATE_ERROR)
    {
      display.setCursor(80, 9);
      display.print(F("TC Error"));
    }
    else
    {
      // Right align temperature reading
      if (input < 10) display.setCursor(91, 9);
      else if (input < 100) display.setCursor(85,9);
      else display.setCursor(80, 9);
      // Display current temperature
      display.print(input);
      display.print((char)247);
      display.print(F("C"));
    }
    
    if (reflowStatus == REFLOW_STATUS_ON)
    {
      // We are updating the display faster than sensor reading
      if (timerSeconds > timerUpdate)
      {
        // Store temperature reading every 3 s
        if ((timerSeconds % 3) == 0)
        {
          timerUpdate = timerSeconds;
          unsigned char averageReading = map(input, 0, 250, 63, 19);
          if (x < (SCREEN_WIDTH - X_AXIS_START))
          {
            temperature[x++] = averageReading;
          }
        }
      }
    }
    
    unsigned char timeAxis;
    for (timeAxis = 0; timeAxis < x; timeAxis++)
    {
      display.drawPixel(timeAxis + X_AXIS_START, temperature[timeAxis], WHITE);
    }
    
    // Update screen
    display.display();
  }

  // Reflow oven controller state machine
  switch (reflowState)
  {
    case REFLOW_STATE_IDLE:
      // If oven temperature is still above room temperature
      if (input >= TEMPERATURE_ROOM)
      {
        reflowState = REFLOW_STATE_TOO_HOT;
      }
      else
      {
        // If switch is pressed to start reflow process
        if (switchStatus == SWITCH_1)
        {
          // Send header for CSV file
          Serial.println(F("Time,Setpoint,Input,Output"));
          // Intialize seconds timer for serial debug information
          timerSeconds = 0;
          
          
          // Initialize reflow plot update timer
          timerUpdate = 0;
          
          for (x = 0; x < (SCREEN_WIDTH - X_AXIS_START); x++)
          {
            temperature[x] = 0;
          }
          // Initialize index for average temperature array used for reflow plot
          x = 0;
         
          
          // Initialize PID control window starting time
          windowStartTime = millis();
          // Ramp up to minimum soaking temperature
          setpoint = TEMPERATURE_SOAK_MIN;
          // Load profile specific constant
          if (reflowProfile == REFLOW_PROFILE_LEADFREE)
          {
            soakTemperatureMax = TEMPERATURE_SOAK_MAX_LF;
            reflowTemperatureMax = TEMPERATURE_REFLOW_MAX_LF;
            soakMicroPeriod = SOAK_MICRO_PERIOD_LF;
          }
          else
          {
            soakTemperatureMax = TEMPERATURE_SOAK_MAX_PB;
            reflowTemperatureMax = TEMPERATURE_REFLOW_MAX_PB;
            soakMicroPeriod = SOAK_MICRO_PERIOD_PB;
          }
          // Tell the PID to range between 0 and the full window size
          reflowOvenPID.SetOutputLimits(0, windowSize);
          reflowOvenPID.SetSampleTime(PID_SAMPLE_TIME);
          // Turn the PID on
          reflowOvenPID.SetMode(AUTOMATIC);
          // Proceed to preheat stage
          reflowState = REFLOW_STATE_PREHEAT;
        }
      }
      break;

    case REFLOW_STATE_PREHEAT:
      reflowStatus = REFLOW_STATUS_ON;
      // If minimum soak temperature is achieve
      if (input >= TEMPERATURE_SOAK_MIN)
      {
        // Chop soaking period into smaller sub-period
        timerSoak = millis() + soakMicroPeriod;
        // Set less agressive PID parameters for soaking ramp
        reflowOvenPID.SetTunings(PID_KP_SOAK, PID_KI_SOAK, PID_KD_SOAK);
        // Ramp up to first section of soaking temperature
        setpoint = TEMPERATURE_SOAK_MIN + SOAK_TEMPERATURE_STEP;
        // Proceed to soaking state
        reflowState = REFLOW_STATE_SOAK;
      }
      break;

    case REFLOW_STATE_SOAK:
      // If micro soak temperature is achieved
      if (millis() > timerSoak)
      {
        timerSoak = millis() + soakMicroPeriod;
        // Increment micro setpoint
        setpoint += SOAK_TEMPERATURE_STEP;
        if (setpoint > soakTemperatureMax)
        {
          // Set agressive PID parameters for reflow ramp
          reflowOvenPID.SetTunings(PID_KP_REFLOW, PID_KI_REFLOW, PID_KD_REFLOW);
          // Ramp up to first section of soaking temperature
          setpoint = reflowTemperatureMax;
          // Proceed to reflowing state
          reflowState = REFLOW_STATE_REFLOW;
        }
      }
      break;

    case REFLOW_STATE_REFLOW:
      // We need to avoid hovering at peak temperature for too long
      // Crude method that works like a charm and safe for the components
      if (input >= (reflowTemperatureMax - 5))
      {
        // Set PID parameters for cooling ramp
        reflowOvenPID.SetTunings(PID_KP_REFLOW, PID_KI_REFLOW, PID_KD_REFLOW);
        // Ramp down to minimum cooling temperature
        setpoint = TEMPERATURE_COOL_MIN;
        // Proceed to cooling state
        reflowState = REFLOW_STATE_COOL;
      }
      break;

    case REFLOW_STATE_COOL:
      // If minimum cool temperature is achieved
      if (input <= TEMPERATURE_COOL_MIN)
      {
        // Retrieve current time for buzzer usage
        buzzerPeriod = millis() + 1000;
        // Turn on buzzer to indicate completion
        digitalWrite(buzzerPin, HIGH);
        // Turn off reflow process
        reflowStatus = REFLOW_STATUS_OFF;
        // Proceed to reflow Completion state
        reflowState = REFLOW_STATE_COMPLETE;
      }
      break;

    case REFLOW_STATE_COMPLETE:
    //play buzzer for buzzerperiod
      if (millis() > buzzerPeriod)
      {
        // Turn off buzzer
        digitalWrite(buzzerPin, LOW);
        // Reflow process ended
        reflowState = REFLOW_STATE_IDLE;
      }
      break;

    case REFLOW_STATE_TOO_HOT:
      // If oven temperature drops below room temperature
      if (input < TEMPERATURE_ROOM)
      {
        // Ready to reflow
        reflowState = REFLOW_STATE_IDLE;
      }
      break;

    case REFLOW_STATE_ERROR:
      // Check for thermocouple fault
      double c = thermocouple.readCelsius();

      // If thermocouple problem is still present - kinked, or broken, or not sitting in the terminal block correctly.
      if(isnan(c))
      {
        // Wait until thermocouple wire is re-connected
        reflowState = REFLOW_STATE_ERROR;
      }
      else
      {
        // Clear to perform reflow process
        reflowState = REFLOW_STATE_IDLE;
      }
      break;
  }

  // If switch 1 is pressed
  if (switchStatus == SWITCH_1)
  {
    // If currently reflow process is ongoing
    if (reflowStatus == REFLOW_STATUS_ON)
    {
      // Button press is for cancelling
      // Turn off reflow process NOW
      reflowStatus = REFLOW_STATUS_OFF;
      // Reinitialize state machine
      reflowState = REFLOW_STATE_IDLE;
    }
  }
  // Switch 2 is pressed
  else if (switchStatus == SWITCH_2)
  {
    // Only can switch reflow profile during idle
    if (reflowState == REFLOW_STATE_IDLE)
    {
      // Currently using lead-free reflow profile
      if (reflowProfile == REFLOW_PROFILE_LEADFREE)
      {
        // Switch to leaded reflow profile
        reflowProfile = REFLOW_PROFILE_LEADED;
        EEPROM.write(PROFILE_TYPE_ADDRESS, 1);
      }
      // Currently using leaded reflow profile
      else
      {
        // Switch to lead-free profile
        reflowProfile = REFLOW_PROFILE_LEADFREE;
        EEPROM.write(PROFILE_TYPE_ADDRESS, 0);
      }
    }
  }
  // Switch status has been read
  switchStatus = SWITCH_NONE;

  // Switch debounce state machine (analog switch)
  switch (debounceState)
  {
    case DEBOUNCE_STATE_IDLE:
      // No valid switch press
      switchStatus = SWITCH_NONE;

      switchValue = readSwitch();

      // If either switch is pressed
      if (switchValue != SWITCH_NONE)
      {
        // Note the pressed switch
        switchMask = switchValue;
        // Intialize debounce counter
        lastDebounceTime = millis();
        // Proceed to check validity of button press
        debounceState = DEBOUNCE_STATE_CHECK;
      }
      break;

    case DEBOUNCE_STATE_CHECK:
      switchValue = readSwitch();
      if (switchValue == switchMask)
      {
        // If minimum debounce period is complete
        if ((millis() - lastDebounceTime) > DEBOUNCE_PERIOD_MIN)
        {
          // Valid switch press
          switchStatus = switchMask;
          // Wait for button release
          debounceState = DEBOUNCE_STATE_RELEASE;
        }
      }
      // A False trigger Dettected
      else
      {
        // Reinitialize button debounce state machine
        debounceState = DEBOUNCE_STATE_IDLE;
      }
      break;

    case DEBOUNCE_STATE_RELEASE:
      switchValue = readSwitch();
      if (switchValue == SWITCH_NONE)
      {
        // Reinitialize button debounce state machine when no switches are pressed
        debounceState = DEBOUNCE_STATE_IDLE;
      }
      break;
  }

  // PID computation and appropriate SSR control
  if (reflowStatus == REFLOW_STATUS_ON)
  {
    currentTime = millis();

    reflowOvenPID.Compute();

    if ((currentTime - windowStartTime) > windowSize)
    {
      // Time to shift the Relay Window
      windowStartTime += windowSize;
    }
    if (output > (currentTime - windowStartTime)) digitalWrite(ssrPin, HIGH);
    else digitalWrite(ssrPin, LOW);
  }
  // Reflow oven process is off, ensure oven is off
  else
  {
    digitalWrite(ssrPin, LOW);
  }
}

switch_t readSwitch(void)
{
  // Checking if either the START/STOP or CHANGE PROFILE Pins were selected
  if (digitalRead(switchStartStopPin) == LOW) return SWITCH_1;
  if (digitalRead(switchLfPbPin) == LOW) return SWITCH_2;
  if (!(digitalRead(switchLfPbPin) == LOW) || !(digitalRead(switchLfPbPin) == LOW)) return SWITCH_NONE;
}
