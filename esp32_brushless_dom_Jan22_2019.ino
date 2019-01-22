// Based on https://github.com/airzone-sama/Brushless_Dominator
//
// EEPROM Functionailty is not in this release... Want it?  Fork it and try to make it work

//
#include <ESP32_Servo.h>  //   https://github.com/jkb-git/ESP32Servo
#include <Bounce2.h>      //  https://github.com/thomasfredericks/Bounce2
#include <SSD1306Ascii.h>
#include <SSD1306AsciiWire.h>
#include <Wire.h>
#include <SerialDebug.h>    // https://github.com/JoaoLopesF/SerialDebug/archive/master.zip  and: https://randomnerdtutorials.com/serialdebug-library-arduino-ide/


// Pin Definitions
// I do not use 3 motors sets so these are commented out for B and C -jenp 1-13-19
#define PIN_MOTOR_A 18
//#define PIN_MOTOR_B 34
//#define PIN_MOTOR_C 27

// Fet definitions
#define PIN_ESC_FET 33
#define PIN_PUSHER_FET 25

// things rleated to triggers
#define PIN_TRIGGER_FULL 14
#define PIN_TRIGGER_HALF 13
#define PIN_TRIGGER_REV 12
#define PIN_ALT_MODE 26




// E stop gets own section because hell yes
#define PIN_E_STOP 27

// Profiles
#define PIN_PROFILE_A 19
#define PIN_PROFILE_B 23

// Select fire modes
#define PIN_SELECT_FIRE_A 37
#define PIN_SELECT_FIRE_B 32

 // Config BT and Debug mode section
#define PIN_CONFIG_MODE 32
#define PIN_ENABLE_BT 1
#define PIN_ENABLE_DEBUG 35

// The fancy stuff that could be optional in some builds and made modular
#define PIN_MAG_SENSOR 16
#define PIN_DART_IN_MAG_SENSOR 17
#define PIN_PUSHER_RETURN_SENSOR 36
#define PIN_CHRONO_A 38
#define PIN_CHRONO_B 39
#define PIN_BATTERY_MONITOR 4

// not sure why I tuck the LED well iR led down here
#define PIN_LED 36



// Configuration Options
byte BurstSize = 3; // Configured
byte TargetDPS = 99; // 99 means full rate
bool EStopOnPusherJam = true; // Controls whether to stop on a pusher jam or not
bool EStopOnDartJam = true; // Controls whether to stop on a dart jam or not
bool FireOnEmptyMag = false; // Controls whether to fire on an empty mag
int MotorSpeedFull = 50; // For full-pull
int MotorSpeedHalf = 30; // For half-pull
int MagSize = 18; // Magazine size
#define ENABLE_CHRONO false // Disable chrono functionality if you only have one barrel sensor (which should be PIN_CHRONO_A)
bool SwapDartsInMagForChrono = false;

// Profile Management
#define PROFILE_ALT_PROFILE_A 0
#define PROFILE_ALT_PROFILE_B 1
#define PROFILE_ALT_PROFILE_C 2
#define PROFILE_ALT_PROFILE_D 3
#define PROFILE_ALT_SWAP_DIM_FPS 4
#define PROFILE_ALT_NOTHING 5
struct ProfileDefinition
{
  byte FullPower = 100;
  byte HalfPower = 30;
  byte ROF = 99;
  byte BurstSize = 3;
  byte MagSize = 18;
  bool FireOnEmptyMag = false;
  byte AltAction = PROFILE_ALT_NOTHING;
};
ProfileDefinition Profiles[4];
byte CurrentProfile = 0;
byte ProfileToConfigure = 0; // For the menu system
byte AltAction = PROFILE_ALT_NOTHING;


// Config Screen Menu Paging
#define CONFIG_MENU_MAIN 0
#define CONFIG_MENU_PROFILE_A 1
#define CONFIG_MENU_PROFILE_A_ALT 2
#define CONFIG_MENU_PROFILE_B 3
#define CONFIG_MENU_PROFILE_B_ALT 4
#define CONFIG_MENU_PROFILE_C 5
#define CONFIG_MENU_PROFILE_C_ALT 6
#define CONFIG_MENU_PROFILE_D 7
#define CONFIG_MENU_PROFILE_D_ALT 8
#define CONFIG_MENU_SYSTEM 9
byte CurrentConfigMenuPage = CONFIG_MENU_MAIN;


// Pusher Controls
// Pusher 3S
#define PULSE_ON_TIME_3S 45
#define PULSE_RETRACT_TIME_3S 75
#define PULSE_DUTY_CYCLE_3S 255 
// Pusher 4S
#define PULSE_ON_TIME_4S 25
#define PULSE_RETRACT_TIME_4S 75
#define PULSE_DUTY_CYCLE_4S 192 
int PulseOnTime;
int PulseRetractTime;
int PulseDutyCycle;
#define SOLENOID_CYCLE_IDLE 0
#define SOLENOID_CYCLE_PULSE 1
#define SOLENOID_CYCLE_RETRACT 2
#define SOLENOID_CYCLE_COOLDOWN 3
int CurrentSolenoidCyclePosition = SOLENOID_CYCLE_IDLE;
unsigned long LastSolenoidCycleStarted = 0;
bool PusherHome = true;


// Firing Controls
#define FIRE_MODE_SINGLE 0
#define FIRE_MODE_BURST 1
#define FIRE_MODE_AUTO 2
#define FIRE_MODE_AUTO_LASTSHOT 3
#define FIRE_MODE_IDLE 4
#define SOLENOID_JAM_DURATION 1500 // How long before a Solenoid Jam occurs
#define PUSHER_SENSOR_ANALOG_THRESHOLD 3500
#define DART_JAM_DURATION 1000
#define CHRONO_SENSOR_ANALOG_THRESHOLD 3500
int CurrentFireMode = FIRE_MODE_SINGLE; // This is the user request based on the button state
int ProcessingFireMode = FIRE_MODE_IDLE; // This is what will actually be fired.
bool ExecuteFiring = false; // Set to true when the Solenoid is supposed to move
int TimeBetweenShots = 0; // Calculated to lower ROF
int ShotsToFire = 0; // Number of shots in the queue
unsigned long LastShot = 0; // When the last shot took place.
unsigned long SolenoidLastHome = 0; // Register the last known time the Solenoid was in the home position
bool ShotInTravel = false; // A shot is in travel.
unsigned long ChronoATriggerTime = 0;
unsigned long ChronoBTriggerTime = 0;
bool ChronoATriggered = false;
bool ChronoBTriggered = false;
bool ChamberEmptied = false; // Keep track of this to see if the chamber was full and then reported empty after a firing command


// Magazine status
#define DART_IN_MAG_SENSOR_ANALOG_THRESHOLD 2000
#define MAG_SENSOR_ANALOG_THRESHOLD 3500
bool DartLoaded = false;
bool MagLoaded = false;
int DartsInMag = 0;
long TotalDartsFired = 0;
long LifetimeDartsFired = 0; 
bool InsertingMag = false; // Cater for the time between the mag insert, and the dart detector registering no darts.


// Physical Switch Status
bool RevTriggerPressed = false; // Rev Trigger is Depressed
bool FireFullTriggerPressed = false; // Fire Trigger is Depressed
bool FireHalfTriggerPressed = false; // Fire Trigger is Depressed
bool ConfigModePressed = false; // Fire Trigger is Depressed


// OLED Stuff
SSD1306AsciiWire Display;
#define OLED_ADDR 0x3C


// Motor Controls
#define MOTOR_SPINUP_LAG 100 // How long we give the motors before we know that have spun up.
#define MOTOR_SPINDOWN_3S 4000
#define MOTOR_SPINDOWN_4S 6000
#define MOTOR_SPINUP_3S 0
#define MOTOR_SPINUP_4S 0
int MaxMotorSpeed = 2000;
int DecelerateTime = 0;
int AccelerateTime = 0;
int MotorRampUpPerMS = 0;
int MotorRampDownPerMS = 0;
int MinMotorSpeed = 1000;
int CurrentMotorSpeed = MinMotorSpeed;
int TargetMotorSpeed = MinMotorSpeed;
bool MotorsEnabled = false;
byte SetMaxSpeed = 100; // in percent.
unsigned long TimeLastMotorSpeedChanged = 0;
#define COMMAND_REV_NONE 0
#define COMMAND_REV_HALF 1
#define COMMAND_REV_FULL 2
int CommandRev = COMMAND_REV_NONE;
int PrevCommandRev = COMMAND_REV_NONE;
bool AutoRev = false; // True when the computer is managing the rev process.
Servo MotorA;
//Servo MotorB;
//Servo MotorC;


// System Controls
#define MODE_NORMAL 0
#define MODE_CONFIG 1
#define MODE_MAG_OUT 2
#define MODE_DARTS_OUT 3
#define MODE_LOW_BATT 4
#define MODE_E_STOP 6
int SystemMode;
bool EStopActive = false;
bool InConfigMode = false;


// Inputs
#define DebounceWindow 5 // Debounce Window = 5ms
Bounce RevTriggerBounce = Bounce();
Bounce FireHalfTriggerBounce = Bounce();
Bounce FireFullTriggerBounce = Bounce();
Bounce PusherResetBounce = Bounce();
Bounce ModeSelectABounce = Bounce();
Bounce ModeSelectBBounce = Bounce();
Bounce EStopBounce = Bounce();
Bounce ConfigModeBounce = Bounce();
Bounce AltModeBounce = Bounce();
Bounce ProfileABounce = Bounce();
Bounce ProfileBBounce = Bounce();


// Misc Stuff
bool DebugEnabled = true; // Set by STM pin
bool BTEnabled = true; // Set by STM Pin


// Serial Comms
#define SERIAL_INPUT_BUFFER_MAX 25
char SerialInputBuffer[SERIAL_INPUT_BUFFER_MAX];
byte SavedMode = FIRE_MODE_SINGLE;
byte SavedBurstLength = 0;
bool HasSavedMode = false;


// Battery Controls
int BatteryS = 3;
#define BATTERY_3S_MIN 9.6
#define BATTERY_3S_MAX 12.6
#define BATTERY_4S_MIN 12.8
#define BATTERY_4S_MAX 16.8
#define BATTERY_CALFACTOR 0.0 // Adjustment for calibration.
float BatteryMaxVoltage;
float BatteryMinVoltage;
float BatteryCurrentVoltage = 99.0;
bool BatteryFlat = false;


void setup() {
  // put your setup code here, to run once:

  Wire.begin();
  Wire.setClock(400000L);
  Display.begin(&Adafruit128x64, OLED_ADDR);

  Display.clear();
  Display.setCursor(0, 0);
  Display.setFont(ZevvPeep8x16);
  Display.print( F("Initialising") );

  pinMode( PIN_ENABLE_DEBUG, INPUT_PULLUP );
  if( digitalRead( PIN_ENABLE_DEBUG ) == HIGH )
    DebugEnabled = false;
  else
    DebugEnabled = true;

  pinMode( PIN_ENABLE_BT, INPUT_PULLUP );
  if( digitalRead( PIN_ENABLE_BT ) == HIGH )
    BTEnabled = false;
  else
    BTEnabled = true;

  Display.clear();
  Display.setCursor(0, 0);
  Display.print( F("Initialising\n") );
  if( DebugEnabled ) 
    Display.print( F("Debug Mode\n") );
  if( BTEnabled ) 
    Display.print( F("BT Enabled\n") );

  delay( 250 );

  DebugInit(); 
  DebugPrintln( F("Booting.. ") );  

  // Connect Bluetooth
  DebugPrintln( F("Initialising Display") );
  

  // Set up debouncing
  DebugPrintln( F("Configuring Debouncing") );

  pinMode(PIN_TRIGGER_REV, INPUT_PULLUP);
  RevTriggerBounce.attach( PIN_TRIGGER_REV );
  RevTriggerBounce.interval( DebounceWindow );
  
  pinMode(PIN_TRIGGER_FULL, INPUT_PULLUP);
  FireFullTriggerBounce.attach( PIN_TRIGGER_FULL );
  FireFullTriggerBounce.interval( DebounceWindow );

  pinMode(PIN_TRIGGER_HALF, INPUT_PULLUP);
  FireHalfTriggerBounce.attach( PIN_TRIGGER_HALF );
  FireHalfTriggerBounce.interval( DebounceWindow );
  
  pinMode(PIN_PUSHER_RETURN_SENSOR, INPUT_PULLUP);
  PusherResetBounce.attach( PIN_PUSHER_RETURN_SENSOR );
  PusherResetBounce.interval( DebounceWindow );  

  pinMode(PIN_SELECT_FIRE_A, INPUT_PULLUP);
  ModeSelectABounce.attach( PIN_SELECT_FIRE_A );
  ModeSelectABounce.interval( DebounceWindow );
  
  pinMode(PIN_SELECT_FIRE_B, INPUT_PULLUP);
  ModeSelectBBounce.attach( PIN_SELECT_FIRE_B );
  ModeSelectBBounce.interval( DebounceWindow );  

  pinMode(PIN_E_STOP, INPUT_PULLUP);
  EStopBounce.attach( PIN_E_STOP );
  EStopBounce.interval( DebounceWindow );  

  pinMode(PIN_CONFIG_MODE, INPUT_PULLUP);
  ConfigModeBounce.attach( PIN_CONFIG_MODE );
  ConfigModeBounce.interval( DebounceWindow );    

  pinMode(PIN_ALT_MODE, INPUT_PULLUP);
  AltModeBounce.attach( PIN_ALT_MODE );
  AltModeBounce.interval( DebounceWindow );    
    
  pinMode(PIN_PROFILE_A, INPUT_PULLUP);
  ProfileABounce.attach( PIN_PROFILE_A );
  ProfileABounce.interval( DebounceWindow );    
  
  pinMode(PIN_PROFILE_B, INPUT_PULLUP);
  ProfileBBounce.attach( PIN_PROFILE_B );
  ProfileBBounce.interval( DebounceWindow );      
  
  DebugPrintln( F("Debouncing Configured") );

  // Set up the sensor pins.
  pinMode( PIN_DART_IN_MAG_SENSOR, INPUT_PULLUP );
  pinMode( PIN_MAG_SENSOR, INPUT_PULLUP );
  pinMode( PIN_CHRONO_A, INPUT_PULLUP );
  pinMode( PIN_CHRONO_A, INPUT_PULLUP );

  DebugPrintln( F("Initialising ESC") );
  // Turn on ESC's
  pinMode(PIN_ESC_FET, OUTPUT);
  digitalWrite( PIN_ESC_FET, HIGH );  

  // Set up motors
  MotorA.attach(PIN_MOTOR_A);
//  MotorB.attach(PIN_MOTOR_B);
//  MotorC.attach(PIN_MOTOR_C);
  // Arm ESC's
  MotorA.writeMicroseconds(MinMotorSpeed);
//  MotorB.writeMicroseconds(MinMotorSpeed);
//  MotorC.writeMicroseconds(MinMotorSpeed);

  delay(1000);   // Wait for ESC to initialise (9 seconds)
  DebugPrintln( F("ESC Initialised") );

  // Initialise the pusher
  pinMode(PIN_PUSHER_FET, OUTPUT);
  digitalWrite( PIN_PUSHER_FET, LOW );    

 

 

  // Battery Configuration
  SetupSelectBattery();
  DebugPrintln( F("Battery Selected") );
  
  if( BatteryS == 3 )
  {
    PulseOnTime = PULSE_ON_TIME_3S;
    PulseRetractTime = PULSE_RETRACT_TIME_3S;
    PulseDutyCycle = PULSE_DUTY_CYCLE_3S;

    BatteryMaxVoltage = BATTERY_3S_MAX;
    BatteryMinVoltage = BATTERY_3S_MIN;

    DecelerateTime = MOTOR_SPINDOWN_3S;
    AccelerateTime = MOTOR_SPINUP_3S;
  }
  else
  {
    PulseOnTime = PULSE_ON_TIME_4S;
    PulseRetractTime = PULSE_RETRACT_TIME_4S;
    PulseDutyCycle = PULSE_DUTY_CYCLE_4S;    

    BatteryMaxVoltage = BATTERY_4S_MAX;
    BatteryMinVoltage = BATTERY_4S_MIN;

    DecelerateTime = MOTOR_SPINDOWN_4S;
    AccelerateTime = MOTOR_SPINUP_4S;
  }
  
  SystemMode = MODE_NORMAL;
  CalculateRampRates(); 
  ChangeProfile( CurrentProfile ); // Restore settings
  
  // Now wait until the trigger is high
  FireHalfTriggerBounce.update();
  while( FireHalfTriggerBounce.read() == HIGH )
  {
    delay(10);
    FireHalfTriggerBounce.update();
  }
  delay(10);
}



void PrintProfile( int ProfileNum )
{
  if( !DebugEnabled )
    return; // Don't waste time

  DebugPrint( "\nProfile Configuration: Profile = " );
  DebugPrintln( ProfileNum );

  DebugPrint( "FullPower = " );
  DebugPrintln( Profiles[ProfileNum].FullPower );
  DebugPrint( "HalfPower = " );
  DebugPrintln( Profiles[ProfileNum].HalfPower );
  DebugPrint( "ROF = " );
  DebugPrintln( Profiles[ProfileNum].ROF );
  DebugPrint( "BurstSize = " );
  DebugPrintln( Profiles[ProfileNum].BurstSize );
  DebugPrint( "MagSize = " );
  DebugPrintln( Profiles[ProfileNum].MagSize );
  DebugPrint( "FireOnEmptyMag = " );
  DebugPrintln( Profiles[ProfileNum].FireOnEmptyMag );
  DebugPrint( "AltAction = " );
  DebugPrintln( Profiles[ProfileNum].AltAction ); 
}

bool ValidateProfile( int ProfileNum )
{
  if( Profiles[ProfileNum].FullPower < 30 || Profiles[ProfileNum].FullPower > 100 ) return false;
  if( Profiles[ProfileNum].HalfPower < 30 || Profiles[ProfileNum].HalfPower > 100 ) return false;
  if( Profiles[ProfileNum].ROF < 1 || Profiles[ProfileNum].ROF > 99 ) return false;
  if( Profiles[ProfileNum].BurstSize < 1 || Profiles[ProfileNum].BurstSize > 99 ) return false;
  if( Profiles[ProfileNum].MagSize < 6 || Profiles[ProfileNum].MagSize > 48 ) return false;
  if( Profiles[ProfileNum].AltAction < PROFILE_ALT_PROFILE_A || Profiles[ProfileNum].AltAction > PROFILE_ALT_NOTHING ) return false;
  return true;
}

void DefaultProfile( int ProfileNum )
{
  Profiles[ProfileNum].FullPower = 100;
  Profiles[ProfileNum].HalfPower = 30;
  Profiles[ProfileNum].ROF = 99;
  Profiles[ProfileNum].BurstSize = 3;
  Profiles[ProfileNum].MagSize = 18;
  Profiles[ProfileNum].FireOnEmptyMag = false;
  Profiles[ProfileNum].AltAction = PROFILE_ALT_NOTHING; 
}

/*
 * Displays the startup screen to select the battery type
 */
void SetupSelectBattery()
{
  unsigned long StartTime = millis();
  int BatSel = 0;
  int LastBatSel = 99;
  bool Processed = false;

  while( !Processed )
  {
    RevTriggerBounce.update(); // Update the pin bounce state
    FireHalfTriggerBounce.update();

    if( RevTriggerBounce.fell() )
    {
      if( BatSel == 0 )
        BatSel = 1;
      else
        BatSel = 0;
    }
    if( FireHalfTriggerBounce.fell() )
    {
      Processed = true;
    }
  
    if( BatSel != LastBatSel )
    {
      Display.clear();
      Display.setCursor(0, 0);
      Display.setFont(ZevvPeep8x16);
      Display.print( F("Confirm Battery\n") );
      if( BatSel == 0 )
      {
        Display.print( F("> 3s\n") );
        Display.print( F("  4s") );
      }
      else
      {
        Display.print( F("  3s\n") );
        Display.print( F("> 4s") );      
      }
    }
    LastBatSel = BatSel;
  }

  if( BatSel == 0 )
    BatteryS = 3;
  else
    BatteryS = 4;

  Display.clear();
  Display.setCursor(0, 0);
  Display.print( F("Initialising") );
  
  while( millis() - StartTime < 3000 )
  {
    delay( 10 );
  }
}

/*
 * This is a boot time init sub to calcualte the Acceleration and 
 * deceleration ramp rates of the motors.
 * 
 */
void CalculateRampRates()
{
  int SpeedRange = (MaxMotorSpeed - MinMotorSpeed) * 1000; // Multiply by 1000 to give some resolution due to integer calculations
  if( AccelerateTime == 0 )
  {
    MotorRampUpPerMS = SpeedRange;  // For instant acceleration
  }
  else
  {
    MotorRampUpPerMS = SpeedRange / AccelerateTime;  // Use when Accelerating
  }

  if( DecelerateTime == 0 )
  {
    MotorRampDownPerMS = SpeedRange;  // For instant acceleration
  }
  else
  {
    MotorRampDownPerMS = SpeedRange / DecelerateTime;  // Use when Decelerating
  }  


  DebugPrint( "Ramp Up per MS = " );
  DebugPrintln( MotorRampUpPerMS );

  DebugPrint( "Ramp Dowbn per MS = " );
  DebugPrintln( MotorRampDownPerMS );
}

void loop() {
  // put your main code here, to run repeatedly:

  ProcessButtons(); // Get User and Sensor input
  ProcessMagRelease(); // Update any conditions based on magazine status
  ProcessBatteryMonitor(); // Check battery voltage
  ProcessSystemMode(); // Find out what the system should be doing
  ProcessAltButton(); // Handle a press of the Alt Mode button
  ProcessProfileButtons(); // Handle the profile button press
  ProcessEStop(); // Process the EStop Condition

  // Process Serial input
  if( ProcessSerialInput() )
  {
    ProcessSerialCommand();
  }
  
  ProcessRevCommand(); // Handle motor intentions
  
  // Detected a change to the command. Reset the last speed change timer.
  if( PrevCommandRev != CommandRev )
  {
    TimeLastMotorSpeedChanged = millis();
    PrevCommandRev = CommandRev;
  }

  // Process speed control  
  ProcessSpeedControl();
  // Calcualte the new motor speed
  ProcessMotorSpeed();
  // Send the speed to the ESC
  ProcessMainMotors();

  // Process Firing Controls
  ProcessFiring();
  ProcessSolenoid();

  // Process the chrony / dart jam sensor stuff now.
  ProcessChrony();

  // Let's fix up the DartsInMag now... 
  // We might think that the mag is empty when there is really a dart still loaded
  if( (DartsInMag <= 0) && DartLoaded && !InsertingMag )
  {
    DartsInMag = 1; // We don't know how many, but at least one..
  }
  // Or we might think we have mire than we really do
  if( (DartsInMag > 0) && !DartLoaded && !InsertingMag )
  {
    DartsInMag = 0;
  }
  // We detected a dart after the mag load event. We are now no longer in the process of loading a magazine.
  if( InsertingMag && DartLoaded )
  {
    InsertingMag = false;
  }

  // Update the OLED
  ProcessDisplay();

  //DebugPrintln( "." );


}

// Update the flash memory with our configuration



bool ProcessSerialInput()
{
  // Neither BT or Debug mose is active
  if( !(BTEnabled || DebugEnabled) )
    return false;

  bool SerialDataAvailable = false;
  if( DebugEnabled )
  {
    if( Serial.available() != 0 )
      SerialDataAvailable = true;
  }

  if( BTEnabled )
  {
    if( Serial1.available() != 0 )
      SerialDataAvailable = true;
  }
    
  if( !SerialDataAvailable ) return false; // Ignore when there is no serial input
  
  static byte CurrentBufferPosition = 0;

  while( (Serial.available() > 0) || (Serial1.available() > 0) )
  {
    char NextByte = 0;
    if( DebugEnabled )
    {
      if( Serial.available() != 0 )
        NextByte = Serial.read();
    } 
    else if( BTEnabled )
    {
      if( Serial1.available() != 0 )
        NextByte = Serial1.read();
    }
    else
    {
      NextByte = 0; //WTF is this happening??
    }

    switch( NextByte )
    {
      case '#': // Starting new command
        CurrentBufferPosition = 0;
        break;
      case '$': // Ending command
        return true; // Jump out.. There's more data in the buffer, but we can read that next time around.
        break;
      default: // Just some stuff coming through
        SerialInputBuffer[ CurrentBufferPosition ] = NextByte; // Insert into the buffer
        CurrentBufferPosition ++; // Move the place to the right
        if( CurrentBufferPosition >= SERIAL_INPUT_BUFFER_MAX ) CurrentBufferPosition = (SERIAL_INPUT_BUFFER_MAX - 1);  // Capture Overflows.
    }
  }

  return false;
}


/*
 * Protocol:
 * Start Character: #
 * Termination Character: $
 * Command Codes: 2 characters exactly
 * Commands:
 * Run Motor Full:  FM
 * Run Motor Half:  HM
 * Stop Motor:      SM
 * Set Full Speed:  FS-XXX where XXX is the percentage of max speed
 * Set Hall Speed:  HS-XXX where XXX is the percentage of max speed
 * Query Device:    QD
 * Get Info:        QS
 * Single Shot:     SS
 * Burst Shot:      BS-XX-Y where XX is the number of shots to fire and Y is 0 for Full Speed, 1 for Half Speed
 * Rate of Fire:    RF-XX where X is 01 to 98 for requested speeds, or 99 for Max
 * Mag Size:        DM-XX where XX is the mag size (Min 6, Max 50)
 * 
 */
void ProcessSerialCommand()
{
  char CommandHeader[3]; // Place the header into this buffer
  // Copy it using a lazy way
  CommandHeader[0] = SerialInputBuffer[0];
  CommandHeader[1] = SerialInputBuffer[1];
  CommandHeader[2] = 0;

  // Run Motor Full Command - FM
  if( (strcmp( CommandHeader, "FM" ) == 0) && (SystemMode == MODE_NORMAL) )
  {
    AutoRev = true;
    CommandRev = COMMAND_REV_FULL;
  }

  // Run Motor Half Command - HM
  if( (strcmp( CommandHeader, "HM" ) == 0) && (SystemMode == MODE_NORMAL) )
  {
    AutoRev = true;
    CommandRev = COMMAND_REV_HALF;
  }

  // Stop Motor Command - SM
  if( (strcmp( CommandHeader, "SM" ) == 0) && (SystemMode == MODE_NORMAL) )
  {
    AutoRev = true;
    CommandRev = COMMAND_REV_NONE;
  }

  // Query Device Command - QD
  if( strcmp( CommandHeader, "QD" ) == 0 )
  {
    DebugPrintln( F("#WD-OK$") );
    if( BTEnabled )
      Serial1.println( F("#WD-OK$") );

  }

  // Single Shot Command - SS
  if( (strcmp( CommandHeader, "SS" ) == 0) && (SystemMode == MODE_NORMAL) )
  {
    AutoRev = true;
    CommandRev = COMMAND_REV_NONE;
  }
  
    
}

void ProcessChrony()
{
  //DebugPrintULln( millis() );
  if( ChamberEmptied && ShotInTravel && ((millis() - LastShot) > DART_JAM_DURATION) ) // The Dart is jammed
  {
    //DebugPrintln( "Jammed Dart" );
    if( EStopOnDartJam )
    {
      EStopActive = true;
      return;
    }
  }
  if( ShotInTravel )
  {
    DebugPrintULln( LastShot );
  }

  if( ShotInTravel && !ENABLE_CHRONO ) // Chrono is not enabled, so just check for the dart exiting the barrel
  {
    if( ChronoATriggered ) // A shot has crossed the beam
    {
      DebugPrintln( "Chrono Sensor A Triggered" );
      ShotInTravel = false; // The dart has left the building..
    }
    return;
  }

  if( ShotInTravel && ENABLE_CHRONO ) // Chrono is not enabled, so just check for the dart exiting the barrel
  {
    if( ChronoBTriggered ) // A shot has crossed the beam final
    {
      ShotInTravel = false; // The dart has left the building..

      // Calculate FPS here.
    }
  }
}

void ProcessProfileButtons()
{
  if( (SystemMode == MODE_CONFIG) || (SystemMode == MODE_LOW_BATT) || (SystemMode == MODE_E_STOP) )
  {
    return; //Ignore it if the system is not in the right mode.
  }

  if( ProfileABounce.rose() )
    ChangeProfile( 0 );

  if( ProfileBBounce.rose() )
    ChangeProfile( 1 );

}

void ProcessAltButton()
{
  if( (SystemMode == MODE_CONFIG) || (SystemMode == MODE_LOW_BATT) || (SystemMode == MODE_E_STOP) )
  {
    return; //Ignore it if the system is not in the right mode.
  }

  if( AltModeBounce.rose() )
  {
    switch( AltAction )
    {
      case PROFILE_ALT_PROFILE_A:
        ChangeProfile( 0 );
        break;
      case PROFILE_ALT_PROFILE_B:
        ChangeProfile( 1 );
        break;      
      case PROFILE_ALT_PROFILE_C:
        ChangeProfile( 2 );
        break;
      case PROFILE_ALT_PROFILE_D:
        ChangeProfile( 3 );
        break;
      case PROFILE_ALT_SWAP_DIM_FPS:
        SwapDartsInMagForChrono = !SwapDartsInMagForChrono;
        break;
      default: // This is the nothing mode
        break;
    }

    AltModeBounce.update();
    DebugPrintln( "." );
  }
  
}

void ChangeProfile( int ProfileNum )
{
  // Check bounds
  if( ProfileNum < 0 ) return;
  if( ProfileNum > 3 ) return;

  DebugPrint( "Changed to profile: " );
  DebugPrintln( ProfileNum );

  CurrentProfile = ProfileNum;
  MotorSpeedFull = Profiles[CurrentProfile].FullPower;
  MotorSpeedHalf = Profiles[CurrentProfile].HalfPower;
  TargetDPS = Profiles[CurrentProfile].ROF;
  BurstSize = Profiles[CurrentProfile].BurstSize;
  MagSize = Profiles[CurrentProfile].MagSize;
  FireOnEmptyMag = Profiles[CurrentProfile].FireOnEmptyMag;
  AltAction = Profiles[CurrentProfile].AltAction;

}

void ProcessMagRelease()
{
  static bool LastMagLoaded = false;
  if( LastMagLoaded != MagLoaded )  // Detect a change in the status quo
  {
    if( MagLoaded )  // A mag has been inserted
    {
      InsertingMag = true;
      DartsInMag = MagSize;
    }
    else // Mag has been dropped
    {
      InsertingMag = false;
      DartsInMag = 0;
      LifetimeDartsFired += TotalDartsFired;
    }
  }
  LastMagLoaded = MagLoaded;
}

void ProcessSolenoid()
{
  if( !PusherHome && (millis() - SolenoidLastHome > SOLENOID_JAM_DURATION) ) // The pusher is jammed
  {
    if( EStopOnPusherJam )
    {
      EStopActive = true;
      return;
    }
  }

  if( !ExecuteFiring ) // Just skip if there is no firing to execute
  {
    return;
  }

  // Calculate duty cycle whenever the target changes.
  static int PrevTargetDPS = 0;
  if( PrevTargetDPS != TargetDPS )
  {
    PrevTargetDPS = TargetDPS;
    if( TargetDPS == 99 ) // Full rate
    {
      TimeBetweenShots = 0;
    }
    else
    {
      int PulseOverhead = PulseOnTime + PulseRetractTime;
      int TotalPulseOverhead = PulseOverhead * TargetDPS;
      int FreeMS = 1000 - TotalPulseOverhead;
      if( FreeMS <= 0 )
      {
        TimeBetweenShots = 0; // Pusher won't achieve this rate
      }
      else
      {
        TimeBetweenShots = FreeMS / TargetDPS;
      }
    }
  }

  if( ProcessingFireMode == FIRE_MODE_IDLE )
  {
    return; // Solenoid is idling.
  }

  if( ShotsToFire == 0 && ProcessingFireMode != FIRE_MODE_IDLE )
  {
    ProcessingFireMode = FIRE_MODE_IDLE;
    CurrentSolenoidCyclePosition = SOLENOID_CYCLE_IDLE;
    digitalWrite( PIN_PUSHER_FET, LOW );
    digitalWrite( PIN_LED, HIGH );
    DebugPrintln( "Finished shooting" );
    ExecuteFiring = false;
    return;    
  }


  if( CommandRev == COMMAND_REV_NONE )
  {
    ProcessingFireMode = FIRE_MODE_IDLE;
    CurrentSolenoidCyclePosition = SOLENOID_CYCLE_IDLE;
    digitalWrite( PIN_PUSHER_FET, LOW );
    digitalWrite( PIN_LED, HIGH );
    DebugPrintln( "Shooting Aborted - Motors not running" );
    ExecuteFiring = false;
    return;        
  }

  if( (millis() - LastSolenoidCycleStarted) < PulseOnTime )
  {
    if( CurrentSolenoidCyclePosition != SOLENOID_CYCLE_PULSE )
    {
      //DebugPrintln( "Start Pulse" );
      if( !PusherHome && EStopOnPusherJam ) // The pusher is not in the home position - so don't fire yet, but don't cancel either.
      {
        LastSolenoidCycleStarted = millis(); // We don't want to come in a bit late and not give enough time to push / retract. So reset the Cycle time.
        return;
      }
      if( (SystemMode == MODE_MAG_OUT) || ( SystemMode == MODE_CONFIG )) // Don't fire with the mag out or with config open
      {
        ShotsToFire = 0;
        DebugPrintln( "Mag Out!!" );
        return;
      }
      if( !DartLoaded && !FireOnEmptyMag ) // Mag is empty, and we are configured not to fire.
      {
        ShotsToFire = 0;
        DebugPrintln( "Mag Empty!!" );
        return;
      } 
      if( ShotInTravel && EStopOnDartJam ) // There is still a dart in travel through the cages, and we are configured to stop on jam.
      {
        ShotsToFire = 0;
        DebugPrintln( "Dart Still In Travel!!" );
        return;        
      }
    }
    CurrentSolenoidCyclePosition = SOLENOID_CYCLE_PULSE;
    digitalWrite( PIN_PUSHER_FET, HIGH );
    digitalWrite( PIN_LED, LOW );
    return;
  }

  if( (millis() - LastSolenoidCycleStarted) < (PulseOnTime + PulseRetractTime) )
  {
    if( CurrentSolenoidCyclePosition != SOLENOID_CYCLE_RETRACT )
    {
      //DebugPrintln( "End Pulse" );
    }
    CurrentSolenoidCyclePosition = SOLENOID_CYCLE_RETRACT;
    digitalWrite( PIN_PUSHER_FET, LOW );
    digitalWrite( PIN_LED, HIGH );
    return;      
  }  

  if((millis() - LastSolenoidCycleStarted) < (PulseOnTime + PulseRetractTime + TimeBetweenShots))
  {
    if( CurrentSolenoidCyclePosition != SOLENOID_CYCLE_COOLDOWN )
    {
      //DebugPrintln( "Cooling Down" );
    }
    CurrentSolenoidCyclePosition = SOLENOID_CYCLE_COOLDOWN;
    digitalWrite( PIN_PUSHER_FET, LOW );
    digitalWrite( PIN_LED, HIGH );
    return;      
  }

  CurrentSolenoidCyclePosition = SOLENOID_CYCLE_IDLE;
  ShotsToFire -= 1;
  TotalDartsFired ++;
  if( DartsInMag > 0 )
  {
    DartsInMag --;
    ChamberEmptied = false; // We assume that there was a dart in the chamber. We are now waiting to see if this was reported.
  }
  //if( DartLoaded )
    ShotInTravel = true; // A shot is in travel. But only if there was one loaded.
  ChronoATriggerTime = 0;
  ChronoBTriggerTime = 0;
  ChronoATriggered = false;
  ChronoBTriggered = false;
  LastShot = millis();
  LastSolenoidCycleStarted = millis();
  DebugPrintln( "Bang!!" );  

  if( !DartLoaded && !FireOnEmptyMag ) // Mag is empty, and we are configured not to fire.
  {
    ShotsToFire = 0;
    DebugPrintln( "Mag Empty!!" );
  }  
}

// Process the firing request and queue up some darts to fire.
void ProcessFiring()
{
  if( !((SystemMode == MODE_NORMAL) || 
        (SystemMode == MODE_DARTS_OUT) || 
        (SystemMode == MODE_MAG_OUT) ||
        (SystemMode == MODE_CONFIG)) ) // Finish off the stroke unless in low batt or e-stop.
  {
    ShotsToFire = 0;
    if( ProcessingFireMode == FIRE_MODE_AUTO_LASTSHOT )
      ProcessingFireMode = FIRE_MODE_AUTO;
    return;
  }

  if( CommandRev == COMMAND_REV_NONE ) // Don't try and push a dart into stationary flywheels..
  {
    if( ProcessingFireMode == FIRE_MODE_AUTO )
    {
      ProcessingFireMode = FIRE_MODE_AUTO_LASTSHOT;
      LastSolenoidCycleStarted = millis();
      ShotsToFire = 0;
      ExecuteFiring = true;
    }
    return;
  }
  
  if( FireFullTriggerBounce.fell() && ProcessingFireMode ==  FIRE_MODE_IDLE )
  {
    ProcessingFireMode = CurrentFireMode;
    switch( ProcessingFireMode )
    {
      case FIRE_MODE_SINGLE:
        ShotsToFire = 1; // Add another shot to the queue
        LastSolenoidCycleStarted = millis();
        ExecuteFiring = true;
        break;
      case FIRE_MODE_BURST:
        ShotsToFire = BurstSize; // Set the burst size
        LastSolenoidCycleStarted = millis();
        ExecuteFiring = true;
        break;        
      case FIRE_MODE_AUTO:
        ShotsToFire = 9999; // Set to something unreasonably high
        LastSolenoidCycleStarted = millis();
        ExecuteFiring = true;
        break;        
    }
  }
  else if( FireFullTriggerBounce.rose() && (ProcessingFireMode == FIRE_MODE_AUTO) ) 
  {
    ProcessingFireMode = FIRE_MODE_AUTO_LASTSHOT;
    LastSolenoidCycleStarted = millis();
    ExecuteFiring = true;

    if( CurrentSolenoidCyclePosition == SOLENOID_CYCLE_PULSE )
    {
      ShotsToFire = 1;
    }
    else
    {
      ShotsToFire = 0;
    }
  }
}


// We are toggline between different system states here..
void ProcessSystemMode()
{
  static int LastSystemMode = MODE_NORMAL;
  //static bool LastInConfigMode = true;

  if( ConfigModeBounce.fell() )
  {
    InConfigMode = !InConfigMode;    
    DebugPrintln( "CFG BTN Fell");
    DebugPrint( "InConfigMode = " );
    DebugPrintln( InConfigMode );
  }

  
  if( BatteryFlat ) // Battery low
  {
    SystemMode = MODE_LOW_BATT;
  }
  else
  {
    if( EStopActive )
    {
      SystemMode = MODE_E_STOP;
    }
    else
    {
      if( InConfigMode )
      {       
        SystemMode = MODE_CONFIG;
        if( LastSystemMode != MODE_CONFIG ) // Reset back to the menu start on a when pressing the config button
          CurrentConfigMenuPage = CONFIG_MENU_MAIN;
        //return;       
      }
      else if( !MagLoaded )
      {
        SystemMode = MODE_MAG_OUT;
        //return;
      }      
      else if( !DartLoaded && !FireOnEmptyMag )
      {
        SystemMode = MODE_DARTS_OUT;
        //return;
      }
      else
        SystemMode = MODE_NORMAL;
    }
  }

  
  if( LastSystemMode != SystemMode )
  {
    DebugPrint( "New System Mode = " );
    DebugPrintln( SystemMode );
    LastSystemMode = SystemMode;
  }
}

// Handle the E-Stop condition
void ProcessEStop()
{
  static bool PrevEStopActive = false;
  if( PrevEStopActive != EStopActive ) // Only process when a change happens.
  {
    PrevEStopActive = EStopActive;
    if( EStopActive ) 
    {
      digitalWrite( PIN_ESC_FET, LOW ); // Turn off cages
      digitalWrite( PIN_PUSHER_FET, LOW ); // Turn off the pusher
    }
    else
    {
      digitalWrite( PIN_ESC_FET, HIGH ); // Turn on cages
    }
  }
}

// We need to set the Target Motor Speed here.
void ProcessSpeedControl()
{
  static byte LastSetMaxSpeed = 100;

  if( CommandRev == COMMAND_REV_HALF ) SetMaxSpeed = MotorSpeedHalf;
  if( CommandRev == COMMAND_REV_FULL ) SetMaxSpeed = MotorSpeedFull;
  if( CommandRev == COMMAND_REV_NONE ) SetMaxSpeed = 0;

  if( LastSetMaxSpeed == SetMaxSpeed ) return; // Speed hasn't changed

  if( CommandRev > COMMAND_REV_NONE ) 
  {
    SetMaxSpeed = constrain( SetMaxSpeed, 30, 100 ); // Constrain between 30% and 100%
  }
  
  TargetMotorSpeed = map( SetMaxSpeed, 0, 100, MinMotorSpeed, MaxMotorSpeed );  // Find out our new target speed.

  LastSetMaxSpeed = SetMaxSpeed;

  DebugPrint( F("New max speed % = ") );
  DebugPrintln( SetMaxSpeed );

  DebugPrint( F("New target speed = ") );
  DebugPrintln( TargetMotorSpeed );
  
}

/*
 * Process the manual commands leading to motor reving
 * 
 * Logic:
 * If AutoRev is being performed, disconnect it when the half trigger is pulled.
 * We are looking for the following events: 
 * If the Half Trigger is pressed, Rev to Speed A
 * If the Rev Trigger is pressed, and the Half Trigger is also pressed, Rev to Speed B
 * If the Rev Trigger is pressed, but the Half Trigger is not, then ignore the command.
 * 
 */
void ProcessRevCommand()
{
  
  static bool PreviousFireHalfTriggerPressed = false; // Keep track of the human input

  if( !((SystemMode == MODE_NORMAL) || (SystemMode == MODE_DARTS_OUT)) ) // Spin the motors down when something out of the ordinary happens.
  {
     CommandRev = COMMAND_REV_NONE;
     AutoRev = false;
     return;
  }

  if( PreviousFireHalfTriggerPressed != FireHalfTriggerPressed )
  {
    // Human has taken control - disengage autopilot
    PreviousFireHalfTriggerPressed = FireHalfTriggerPressed;
    AutoRev = false;
  }

  if( !AutoRev )
  {
    if( FireHalfTriggerPressed )
    {
      if( RevTriggerPressed )
      {
          CommandRev = COMMAND_REV_FULL;
      }
      else
      {
        CommandRev = COMMAND_REV_HALF;
      }
    }
    else
    {
      CommandRev = COMMAND_REV_NONE;
    }
  }
  // Else the computer is controlling, and the current rev trigger state is ignored. Autopilot will adjust CommandRev
  
}

void ProcessMainMotors()
{
  static long PreviousMotorSpeed = MinMotorSpeed;

  if( PreviousMotorSpeed != CurrentMotorSpeed ) 
  { 
    // Debugging output
    DebugPrintln(CurrentMotorSpeed);

    // Use this for Servo Library
    MotorA.writeMicroseconds( CurrentMotorSpeed );
//    MotorB.writeMicroseconds( CurrentMotorSpeed );
//    MotorC.writeMicroseconds( CurrentMotorSpeed );

    PreviousMotorSpeed = CurrentMotorSpeed;
  }
}

/*
 * Process input from Buttons and Sensors.
 */
void ProcessButtons()
{
  RevTriggerBounce.update(); // Update the pin bounce state
  RevTriggerPressed = !(RevTriggerBounce.read());

  ConfigModeBounce.update(); // Update the pin bounce state
  ConfigModePressed = !(ConfigModeBounce.read());

  // These are onbly watching for the rising edge (which is when the button is released)
  AltModeBounce.update(); // Update the pin bounce state
  ProfileABounce.update(); // Update the pin bounce state
  ProfileBBounce.update(); // Update the pin bounce state

  FireFullTriggerBounce.update(); // Update the pin bounce state
  FireFullTriggerPressed = !(FireFullTriggerBounce.read()); // We don't really care about this, because firing is based on the fall to ground potential

  FireHalfTriggerBounce.update(); // Update the pin bounce state
  FireHalfTriggerPressed = (FireHalfTriggerBounce.read()); // n.b. This switch is NC.

  PusherResetBounce.update(); // Update the pin bounce state
  PusherHome = (PusherResetBounce.read() == LOW);
  if( PusherHome )
  {
    SolenoidLastHome = millis(); // Keep track of the last known time
  }
  if( !PusherHome )
  {
    int PusherSensorValue = analogRead( PIN_PUSHER_RETURN_SENSOR );
    if( PusherSensorValue <= PUSHER_SENSOR_ANALOG_THRESHOLD ) // Just in case something is dirty
      PusherHome = true; 
    //else
    //  DebugPrintln( PusherSensorValue );
  }

  bool CheckDartLoaded = false;
  CheckDartLoaded = (digitalRead(PIN_DART_IN_MAG_SENSOR) == HIGH);
  if( !CheckDartLoaded )
  {
    int DartInMagSensorValue = analogRead( PIN_DART_IN_MAG_SENSOR );
    if( DartInMagSensorValue >= DART_IN_MAG_SENSOR_ANALOG_THRESHOLD ) // Just in case something is dirty
      CheckDartLoaded = true; 
    //else
      //DebugPrintln( DartInMagSensorValue );
  }
  // We need to wait for the new dart to be pushed into the chamber
  if( CheckDartLoaded )
  {
    DartLoaded = true;
  }
  else // Dart is not chambered yet. If the pusher has returned home and is ready for the next shot, and there is a mag loaded, assume that the mag is empty
  {
    if( !ChamberEmptied ) // We know that the system has set this to true when the solenoid was pushed. Now there is no dart seen. So let's signify this.
      ChamberEmptied = true;
    if( PusherHome && MagLoaded )
    {
      DartLoaded = false;
    }
    // Otherwise just keep the prior value
  }

  MagLoaded = (digitalRead(PIN_MAG_SENSOR) == LOW);
  if( !MagLoaded )
  {
    int MagSensorValue = analogRead( PIN_MAG_SENSOR );
    if( MagSensorValue <= MAG_SENSOR_ANALOG_THRESHOLD ) // Just in case something is dirty
      MagLoaded = true; 
    //else
      //DebugPrintln( MagSensorValue );
  }  

  bool PreviousChronoATriggered = ChronoATriggered;
  ChronoATriggered = (digitalRead(PIN_CHRONO_A) == LOW);
  if( !ChronoATriggered )
  {
    int ChronoSensorValue = analogRead( PIN_CHRONO_A );
    if( ChronoSensorValue >= CHRONO_SENSOR_ANALOG_THRESHOLD ) // Just in case something is dirty
      ChronoATriggered = true; 
    //else
      //DebugPrintln( ChronoSensorValue );
  } 
   
  //DebugPrintln( ChronoATriggered );
  if( ChronoATriggered && !PreviousChronoATriggered ) // Sensor has just risen
  {
    ChronoATriggerTime = micros(); // Capture the time that the change occurred.
  }
  

  bool PreviousChronoBTriggered = ChronoBTriggered;
  ChronoBTriggered = (digitalRead(PIN_CHRONO_B) == LOW);
  if( !ChronoBTriggered )
  {
    int ChronoSensorValue = analogRead( PIN_CHRONO_B );
    if( ChronoSensorValue >= CHRONO_SENSOR_ANALOG_THRESHOLD ) // Just in case something is dirty
      ChronoBTriggered = true; 
    //else
      //DebugPrintln( DartInMagSensorValue );
  }  
  //DebugPrintln( ChronoBTriggered );
  if( ChronoBTriggered && !PreviousChronoBTriggered ) // Sensor has just risen
  {
    ChronoBTriggerTime = micros(); // Capture the time that the change occurred.
  }


  // Determine the current firing mode
  ModeSelectABounce.update();
  ModeSelectBBounce.update();
  if( ModeSelectABounce.read() == LOW && ModeSelectBBounce.read() == HIGH && CurrentFireMode != FIRE_MODE_AUTO_LASTSHOT )
    CurrentFireMode = FIRE_MODE_AUTO;
  else if( ModeSelectABounce.read() == HIGH && ModeSelectBBounce.read() == HIGH )
    CurrentFireMode = FIRE_MODE_BURST;
  else if( ModeSelectABounce.read() == HIGH && ModeSelectBBounce.read() == LOW )
    CurrentFireMode = FIRE_MODE_SINGLE;

  EStopBounce.update(); // Update the e-stop state.
  if( EStopBounce.fell() )
  {
    EStopActive = !EStopActive;
  }
}

/*
 * Run the main motors.
 */
void ProcessMotorSpeed()
{
  // Don't do anything if the motor is already running at the desired speed.
  if( CurrentMotorSpeed == TargetMotorSpeed )
  {
    return;
  }

  unsigned long CurrentTime = millis(); // Need a base time to calcualte from
  unsigned long MSElapsed = CurrentTime - TimeLastMotorSpeedChanged;
  if( MSElapsed == 0 ) // No meaningful time has elapsed, so speed will not change
  {
    return;
  }
  if( CurrentMotorSpeed < TargetMotorSpeed )
  {
    int SpeedDelta = (MSElapsed * MotorRampUpPerMS / 1000);
    if( SpeedDelta < 1 ) return; // Not enough cycles have passed to make an appreciable difference to speed.    
    int NewMotorSpeed = CurrentMotorSpeed + SpeedDelta; // Calclate the new motor speed..

    // If it's within 1% (which is 10) of target, then just set it
    if( NewMotorSpeed + 10 >= TargetMotorSpeed )
    {
      NewMotorSpeed = TargetMotorSpeed;
    }

    TimeLastMotorSpeedChanged = CurrentTime;
    CurrentMotorSpeed = NewMotorSpeed;
  }
  if( CurrentMotorSpeed > TargetMotorSpeed )
  {
    int SpeedDelta = (MSElapsed * MotorRampDownPerMS / 1000);
    if( SpeedDelta < 1 ) return; // Not enough cycles have passed to make an appreciable difference to speed.
    int NewMotorSpeed = CurrentMotorSpeed - SpeedDelta; // Calclate the new motor speed..

    // If it's within 1% (which is 10) of target, then just set it
    if( NewMotorSpeed - 10 <= TargetMotorSpeed )
    {
      NewMotorSpeed = TargetMotorSpeed;
    }

    TimeLastMotorSpeedChanged = CurrentTime;
    CurrentMotorSpeed = NewMotorSpeed;
  }
}
void ProcessBatteryMonitor()
{
  
  // Only count one in 10 run-through cycles
  static int RunNumber = 0;
  RunNumber++;
  if( RunNumber <= 300 ) // Only read once every 300 cycles.. For performance reasons.
    return;
  RunNumber = 0;
  
  #define NUM_SAMPLES 100
  static int CollectedSamples = 0;
  static float SampleAverage = 0;
  float SensorValue = analogRead( PIN_BATTERY_MONITOR );
  if( CollectedSamples < NUM_SAMPLES )
  {
    CollectedSamples ++;
    SampleAverage += SensorValue;
  }
  else
  {
    BatteryCurrentVoltage = (((float)SampleAverage / (float)CollectedSamples * 3.3)  / 4096.0 * (float)((47.0 + 10.0) / 10.0)) + BATTERY_CALFACTOR;  // Voltage dividor - 47k and 10k
    if( BatteryCurrentVoltage < BatteryMinVoltage )
    {
      if( BatteryCurrentVoltage > 1.5 ) // If the current voltage is 0, we are probably debugging
      {
        BatteryFlat = true;
      }
      else
      {
        BatteryFlat = false;
      }
    }
    else
    {
      BatteryFlat = false;
    }  
    //DebugPrint( "BatteryVoltage = " );
    //DebugPrintln( BatteryCurrentVoltage );
    CollectedSamples = 0;
    SampleAverage = 0;
  }
}




/*
 * OLED Display Stuff
 */

void ProcessDisplay()
{
/*
#define MODE_NORMAL 0
#define MODE_CONFIG 1
#define MODE_MAG_OUT 2
#define MODE_DARTS_OUT 3
#define MODE_LOW_BATT 4
#define MODE_E_STOP 6
*/
  
  static int LastSystemMode = 99;
  static int LastConfigMenuPage = 99;
  bool ClearScreen = false;
  
  if( LastSystemMode != SystemMode )
  {
    ClearScreen = true;
    LastSystemMode = SystemMode;
  }
  if( SystemMode == MODE_CONFIG )
  {
    if( LastConfigMenuPage != CurrentConfigMenuPage )
    {
      ClearScreen = true;
      LastConfigMenuPage = CurrentConfigMenuPage;
      DebugPrintln( "Config page change" );
    }
  }

  Display_ScreenHeader( ClearScreen );
  switch( SystemMode )
  {
    case MODE_MAG_OUT:
      Display_MagOut( ClearScreen );
      break;
    case MODE_CONFIG:
      Display_Config( ClearScreen );
      break;
    case MODE_LOW_BATT:
      Display_LowBatt( ClearScreen );
      break;
    case MODE_DARTS_OUT:
      Display_Darts_Out( ClearScreen );
      break;
    case MODE_E_STOP:
      Display_EStop( ClearScreen );
      break;
    default:
      Display_Normal( ClearScreen );
      break;
  }
} 

void Display_Config( bool ClearScreen )
{

  /*
#define CONFIG_MENU_MAIN 0
#define CONFIG_MENU_PROFILE_A 1
#define CONFIG_MENU_PROFILE_A_ALT 2
#define CONFIG_MENU_PROFILE_B 3
#define CONFIG_MENU_PROFILE_B_ALT 4
#define CONFIG_MENU_PROFILE_C 5
#define CONFIG_MENU_PROFILE_C_ALT 6
#define CONFIG_MENU_PROFILE_D 7
#define CONFIG_MENU_PROFILE_D_ALT 8
#define CONFIG_MENU_SYSTEM 9
   */
  switch( CurrentConfigMenuPage )
  {
    case CONFIG_MENU_PROFILE_A:
      Display_ConfigProfile( ClearScreen );
      break;
    case CONFIG_MENU_PROFILE_A_ALT:
      Display_ConfigProfileAlt( ClearScreen );
      break;
    case CONFIG_MENU_PROFILE_B:
      Display_ConfigProfile( ClearScreen );
      break;    
    case CONFIG_MENU_PROFILE_B_ALT:
      Display_ConfigProfileAlt( ClearScreen );
      break;    
    case CONFIG_MENU_PROFILE_C:
      Display_ConfigProfile( ClearScreen );
      break;    
    case CONFIG_MENU_PROFILE_C_ALT:
      Display_ConfigProfileAlt( ClearScreen );
      break;    
    case CONFIG_MENU_PROFILE_D:
      Display_ConfigProfile( ClearScreen );
      break;    
    case CONFIG_MENU_PROFILE_D_ALT:
      Display_ConfigProfileAlt( ClearScreen );
      break;   
    case CONFIG_MENU_SYSTEM:
      Display_ConfigSystem( ClearScreen );
      break; 
    default: // Main menu
      Display_ConfigMainMenu( ClearScreen );
      break;    
  }
}

void Display_ConfigMainMenu( bool ClearScreen )
{
  static byte MenuItem = 0;
  static byte LastMenuItem = 99;
  if( ClearScreen || (MenuItem != LastMenuItem) )
  {
    if( ClearScreen )
      MenuItem = 0;

    DebugPrintln( F("CONFIG SCREEN - MAIN MENU") );

    Display.setFont(ZevvPeep8x16);
    Display.setCursor(0, 2);

    switch( MenuItem )
    {
      case 0:
        Display.print( F("> Profile A\n") );
        Display.print( F("  Profile B\n") );
        Display.print( F("  Profile C") );
        break;
     case 1:   
        Display.print( F("  Profile A\n") );
        Display.print( F("> Profile B\n") );
        Display.print( F("  Profile C") );
        break;
     case 2:   
        Display.print( F("  Profile B\n") );
        Display.print( F("> Profile C\n") );
        Display.print( F("  Profile D") );
        break;
     case 3:   
        Display.print( F("  Profile C\n") );
        Display.print( F("> Profile D\n") );
        Display.print( F("  System   ") );
        break;
     default:   
        Display.print( F("  Profile C\n") );
        Display.print( F("  Profile D\n") );
        Display.print( F("> System   ") );
        break;
    }

    LastMenuItem = MenuItem;
  }

  if( RevTriggerBounce.rose() )
  {
    MenuItem++;
    if( MenuItem > 4 )
      MenuItem = 0;
  }

  if( FireHalfTriggerBounce.rose() )
  {
    switch( MenuItem )
    {
      case 0:
        ProfileToConfigure = 0;
        CurrentConfigMenuPage = CONFIG_MENU_PROFILE_A;
        break;
      case 1:
        ProfileToConfigure = 1;
        CurrentConfigMenuPage = CONFIG_MENU_PROFILE_B;
        break;
      case 2:
        ProfileToConfigure = 2;
        CurrentConfigMenuPage = CONFIG_MENU_PROFILE_C;
        break;
      case 3:
        ProfileToConfigure = 3;
        CurrentConfigMenuPage = CONFIG_MENU_PROFILE_D;
        break;
      default:
        CurrentConfigMenuPage = CONFIG_MENU_SYSTEM;
        break;
    }
  }
}


void Display_ConfigProfileAlt( bool ClearScreen )
{
  static byte MenuItem = 0;
  static byte LastMenuItem = 99;
  if( ClearScreen || (MenuItem != LastMenuItem) )
  {
    if( ClearScreen )
      MenuItem = 0;

    DebugPrint( F("CONFIG SCREEN - PROFILE: ") );
    DebugPrintln( ProfileToConfigure );

    Display.setFont(ZevvPeep8x16);
    Display.setCursor(0, 2);

    char CurrentAltOption[20];
    switch( Profiles[ProfileToConfigure].AltAction )
    {
      case PROFILE_ALT_PROFILE_A:
        strcpy(CurrentAltOption, "   Set Profile A\n");
        break;
      case PROFILE_ALT_PROFILE_B:
        strcpy(CurrentAltOption, "   Set Profile B\n");
        break;
      case PROFILE_ALT_PROFILE_C:
        strcpy(CurrentAltOption, "   Set Profile C\n");
        break;
      case PROFILE_ALT_PROFILE_D:
        strcpy(CurrentAltOption, "   Set Profile D\n");
        break;
      case PROFILE_ALT_SWAP_DIM_FPS:
        strcpy(CurrentAltOption, "   Chrony Mode  \n");
        break;
      default:
        strcpy(CurrentAltOption, "   Nothing      \n");
        break;      
    }

    switch( MenuItem )
    {
      case 0:
        Display.print( F("> Alt Btn Action\n") );
        Display.print( CurrentAltOption );
        Display.print( F("  Back          ") );
        break;
     default:   
        Display.print( F("  Alt Btn Action\n") );
        Display.print( CurrentAltOption );
        Display.print( F("> Back          ") );
        break;
    }

    LastMenuItem = MenuItem;
  }

  if( RevTriggerBounce.rose() )
  {
    MenuItem++;
    if( MenuItem > 1 )
      MenuItem = 0;
  }

  if( FireHalfTriggerBounce.rose() )
  {
    switch( MenuItem )
    {
      case 0:
        Profiles[ProfileToConfigure].AltAction ++;
        if( Profiles[ProfileToConfigure].AltAction > PROFILE_ALT_NOTHING )
          Profiles[ProfileToConfigure].AltAction = PROFILE_ALT_PROFILE_A;
        if( ProfileToConfigure == CurrentProfile )
          AltAction = Profiles[ProfileToConfigure].AltAction;
        break;
      default:
      {
        switch( ProfileToConfigure )
        {
          case 0:
          CurrentConfigMenuPage = CONFIG_MENU_PROFILE_A;
          break;
        case 1:
          CurrentConfigMenuPage = CONFIG_MENU_PROFILE_B;
          break;
        case 2:
          CurrentConfigMenuPage = CONFIG_MENU_PROFILE_C;
          break;
        default:
          CurrentConfigMenuPage = CONFIG_MENU_PROFILE_D;
          break;
        }
        break;
      }
    }

    LastMenuItem = 99;
  } 
}


void Display_ConfigProfile( bool ClearScreen )
{
  static byte MenuItem = 0;
  static byte LastMenuItem = 99;
  if( ClearScreen || (MenuItem != LastMenuItem) )
  {
    if( ClearScreen )
      MenuItem = 0;

    DebugPrint( F("CONFIG SCREEN - PROFILE: ") );
    DebugPrintln( ProfileToConfigure );

    char BurstBuffer[4];
    sprintf( BurstBuffer, "%2d", Profiles[ProfileToConfigure].BurstSize );
    char MagBuffer[4];
    sprintf( MagBuffer, "%2d", Profiles[ProfileToConfigure].MagSize ); 
    char ROFBuffer[4];
    sprintf( ROFBuffer, "%2d", Profiles[ProfileToConfigure].ROF ); 
    char FullPowerBuffer[5];
    sprintf( FullPowerBuffer, "%3d", Profiles[ProfileToConfigure].FullPower ); 
    char HalfPowerBuffer[5];
    sprintf( HalfPowerBuffer, "%3d", Profiles[ProfileToConfigure].HalfPower ); 
    
    Display.setFont(ZevvPeep8x16);
    Display.setCursor(0, 2);

    switch( MenuItem )
    {
      case 0:
        Display.print( F("> Full Pwr: ") );
        Display.print( FullPowerBuffer );
        Display.print( F(" \n") );
        Display.print( F("  Half Pwr: ") );
        Display.print( HalfPowerBuffer );
        Display.print( F(" \n") );
        Display.print( F("  DPS: ") );
        if( Profiles[ProfileToConfigure].ROF == 99 )
          Display.print( "Max" );
        else
          Display.print( ROFBuffer );
        Display.print( F("      ") );
        break;
     case 1:   
        Display.print( F("  Full Pwr: ") );
        Display.print( FullPowerBuffer );
        Display.print( F(" \n") );
        Display.print( F("> Half Pwr: ") );
        Display.print( HalfPowerBuffer );
        Display.print( F(" \n") );
        Display.print( F("  DPS: ") );
        if( Profiles[ProfileToConfigure].ROF == 99 )
          Display.print( "Max" );
        else
          Display.print( ROFBuffer );
        Display.print( F("      ") );
        break;
     case 2:   
        Display.print( F("  Half Pwr: ") );
        Display.print( HalfPowerBuffer );
        Display.print( F(" \n") );
        Display.print( F("> DPS: ") );
        if( Profiles[ProfileToConfigure].ROF == 99 )
          Display.print( "Max" );
        else
          Display.print( ROFBuffer );
        Display.print( F("      \n") );
        Display.print( F("  Burst: ") );
        Display.print( BurstBuffer );
        Display.print( F("     ") );
        break;
     case 3:   
        Display.print( F("  DPS: ") );
        if( Profiles[ProfileToConfigure].ROF == 99 )
          Display.print( "Max" );
        else
          Display.print( ROFBuffer );
        Display.print( F("      \n") );
        Display.print( F("> Burst: ") );
        Display.print( BurstBuffer );
        Display.print( F("     \n") );
        Display.print( F("  Mag Size: ") );
        Display.print( MagBuffer );
        Display.print( F(" ") );
        break;
     case 4:
        Display.print( F("  Burst: ") );
        Display.print( BurstBuffer );
        Display.print( F("     \n") );
        Display.print( F("> Mag Size: ") );
        Display.print( MagBuffer );
        Display.print( F(" \n") );
        Display.print( F("  Dry Fire: ") );
        if( Profiles[ProfileToConfigure].FireOnEmptyMag )
          Display.print( "Y" );
        else
          Display.print( "N" );
        Display.print( F("   ") );
        break;
     case 5:
        Display.print( F("  Mag Size: ") );
        Display.print( MagBuffer );
        Display.print( F(" \n") );
        Display.print( F("> Dry Fire: ") );
        if( Profiles[ProfileToConfigure].FireOnEmptyMag )
          Display.print( "Y" );
        else
          Display.print( "N" );
        Display.print( F("   \n") );
        Display.print( F("  Assign Alt Btn") );
        break;
     case 6:        
        Display.print( F("  Dry Fire: ") );
        if( Profiles[ProfileToConfigure].FireOnEmptyMag )
          Display.print( "Y" );
        else
          Display.print( "N" );
        Display.print( F("   \n") );
        Display.print( F("> Assign Alt Btn\n") );
        Display.print( F("  Back          ") );
        break;
     default:   
        Display.print( F("  Dry Fire: ") );
        if( Profiles[ProfileToConfigure].FireOnEmptyMag )
          Display.print( "Y" );
        else
          Display.print( "N" );
        Display.print( F("   \n") );
        Display.print( F("  Assign Alt Btn\n") );
        Display.print( F("> Back          ") );
        break;
    }

    LastMenuItem = MenuItem;
  }

  if( RevTriggerBounce.rose() )
  {
    MenuItem++;
    if( MenuItem > 7 )
      MenuItem = 0;
  }

  if( FireHalfTriggerBounce.rose() )
  {
    switch( MenuItem )
    {
      case 0:
        Profiles[ProfileToConfigure].FullPower -= 5;
        if( Profiles[ProfileToConfigure].FullPower < 30 )
          Profiles[ProfileToConfigure].FullPower = 100;
        if( ProfileToConfigure == CurrentProfile )
          MotorSpeedFull = Profiles[ProfileToConfigure].FullPower;
        break;
      case 1:
        Profiles[ProfileToConfigure].HalfPower -= 5;
        if( Profiles[ProfileToConfigure].HalfPower < 30 )
          Profiles[ProfileToConfigure].HalfPower = 100;
        if( ProfileToConfigure == CurrentProfile )
          MotorSpeedHalf = Profiles[ProfileToConfigure].HalfPower;
      
        break;
      case 2:
      {
        int MaxWholeDPS = 1000 / (PulseOnTime + PulseRetractTime);
        if( Profiles[ProfileToConfigure].ROF == 99 )
          Profiles[ProfileToConfigure].ROF = MaxWholeDPS;
        else if( Profiles[ProfileToConfigure].ROF == 1 )
          Profiles[ProfileToConfigure].ROF = 99;
        else
          Profiles[ProfileToConfigure].ROF --;
        if( ProfileToConfigure == CurrentProfile )
          TargetDPS = Profiles[ProfileToConfigure].ROF;
        
        break;
      }
      case 3:
        Profiles[ProfileToConfigure].BurstSize --;
        if( (Profiles[ProfileToConfigure].BurstSize <= 0) || (Profiles[ProfileToConfigure].BurstSize > Profiles[ProfileToConfigure].MagSize) )
        {
          Profiles[ProfileToConfigure].BurstSize = Profiles[ProfileToConfigure].MagSize;
        }
        if( ProfileToConfigure == CurrentProfile )
          BurstSize = Profiles[ProfileToConfigure].BurstSize;
       
        break;
      case 4:
        Profiles[ProfileToConfigure].MagSize --;
        if( Profiles[ProfileToConfigure].MagSize <= 0 )
        {
          Profiles[ProfileToConfigure].MagSize = 48;
        }
        if( ProfileToConfigure == CurrentProfile )
          MagSize = Profiles[ProfileToConfigure].MagSize;
        
        break;
      case 5:
        Profiles[ProfileToConfigure].FireOnEmptyMag = !Profiles[ProfileToConfigure].FireOnEmptyMag;
        if( ProfileToConfigure == CurrentProfile )
          FireOnEmptyMag = Profiles[ProfileToConfigure].FireOnEmptyMag;
       
        break;
      case 6:
      {
        switch( ProfileToConfigure )
        {
          case 0:
          CurrentConfigMenuPage = CONFIG_MENU_PROFILE_A_ALT;
          break;
        case 1:
          CurrentConfigMenuPage = CONFIG_MENU_PROFILE_B_ALT;
          break;
        case 2:
          CurrentConfigMenuPage = CONFIG_MENU_PROFILE_C_ALT;
          break;
        default:
          CurrentConfigMenuPage = CONFIG_MENU_PROFILE_D_ALT;
          break;
        }
        break;
      default:
        CurrentConfigMenuPage = CONFIG_MENU_MAIN;
        break;
      }
    }
    
    LastMenuItem = 99;
  } 

  if( AltModeBounce.rose() ) // Since we have some pretty heavy options, the Alt Mode button will cycle the other way. Ignore the sub menu stuff
  {
    switch( MenuItem )
    {
      case 0:
        Profiles[ProfileToConfigure].FullPower += 5;
        if( Profiles[ProfileToConfigure].FullPower > 100 )
          Profiles[ProfileToConfigure].FullPower = 30;
        if( ProfileToConfigure == CurrentProfile )
          MotorSpeedFull = Profiles[ProfileToConfigure].FullPower;
        
        break;
      case 1:
        Profiles[ProfileToConfigure].HalfPower += 5;
        if( Profiles[ProfileToConfigure].HalfPower > 100 )
          Profiles[ProfileToConfigure].HalfPower = 30;
        if( ProfileToConfigure == CurrentProfile )
          MotorSpeedHalf = Profiles[ProfileToConfigure].HalfPower;
        break;
      case 2:
      {
        int MaxWholeDPS = 1000 / (PulseOnTime + PulseRetractTime);
        if( Profiles[ProfileToConfigure].ROF == 99 )
          Profiles[ProfileToConfigure].ROF = 1;
        else if( Profiles[ProfileToConfigure].ROF == MaxWholeDPS )
          Profiles[ProfileToConfigure].ROF = 99;
        else
          Profiles[ProfileToConfigure].ROF ++;
        if( ProfileToConfigure == CurrentProfile )
          TargetDPS = Profiles[ProfileToConfigure].ROF;  
        break;
      }
      case 3:
        Profiles[ProfileToConfigure].BurstSize ++;
        if( Profiles[ProfileToConfigure].BurstSize > Profiles[ProfileToConfigure].MagSize )
        {
          Profiles[ProfileToConfigure].BurstSize = 1;
        }
        if( ProfileToConfigure == CurrentProfile )
          BurstSize = Profiles[ProfileToConfigure].BurstSize;
        break;
      case 4:
        Profiles[ProfileToConfigure].MagSize ++;
        if( Profiles[ProfileToConfigure].MagSize >= 48 )
        {
          Profiles[ProfileToConfigure].MagSize = 6;
        }
        if( ProfileToConfigure == CurrentProfile )
          MagSize = Profiles[ProfileToConfigure].MagSize;
        break;
      case 5:
        Profiles[ProfileToConfigure].FireOnEmptyMag = !Profiles[ProfileToConfigure].FireOnEmptyMag;
        if( ProfileToConfigure == CurrentProfile )
          FireOnEmptyMag = Profiles[ProfileToConfigure].FireOnEmptyMag;
      
        break;
      default:
        break;
    } 

    LastMenuItem = 99;
  }
}

void Display_ConfigSystem( bool ClearScreen )
{
  static byte MenuItem = 0;
  static byte LastMenuItem = 99;
  if( ClearScreen || (MenuItem != LastMenuItem) )
  {
    if( ClearScreen )
      MenuItem = 0;
    DebugPrintln( F("CONFIG SCREEN - SYSTEM") );

    Display.setFont(ZevvPeep8x16);
    Display.setCursor(0, 2);

    char TDFBuffer[7];
    sprintf( TDFBuffer, "%6d", TotalDartsFired );    
    char LDFBuffer[7];
    sprintf( LDFBuffer, "%6d", LifetimeDartsFired );  

    switch( MenuItem )
    {
      case 0:
        Display.print( F("> Unused ") );

        Display.print( "\n" );
        Display.print( F("  P Jam Stop: ") );
         if( EStopOnPusherJam )
          Display.print( "Y" );
        else
          Display.print( "N" );
        Display.print( " \n" );       
        Display.print( F("  D Jam Stop: ") );
         if( EStopOnDartJam )
          Display.print( "Y" );
        else
          Display.print( "N" );  
        break;
     case 1:   
        Display.print( F("  Unused ") );

        Display.print( "\n" );
        Display.print( F("> P Jam Stop: ") );
         if( EStopOnPusherJam )
          Display.print( "Y" );
        else
          Display.print( "N" );
        Display.print( " \n" ); 
        Display.print( F("  D Jam Stop: ") );
         if( EStopOnDartJam )
          Display.print( "Y" );
        else
          Display.print( "N" );     
        break;
     case 2:   
        Display.print( F("  P Jam Stop: ") );
         if( EStopOnPusherJam )
          Display.print( "Y" );
        else
          Display.print( "N" );
        Display.print( " \n" );       
        Display.print( F("> D Jam Stop: ") );
         if( EStopOnDartJam )
          Display.print( "Y" );
        else
          Display.print( "N" );     
        Display.print( " \n" );  
        Display.print( F("  TDF: ") );
        Display.print( TDFBuffer );
        Display.print( F("  ") );
        break;
     case 3:   
        Display.print( F("  D Jam Stop: ") );
         if( EStopOnDartJam )
          Display.print( "Y" );
        else
          Display.print( "N" );    
        Display.print( " \n" );    
        Display.print( F("> TDF: ") );
        Display.print( TDFBuffer );
        Display.print( F("  ") );
        Display.print( " \n" );        
        Display.print( F("  LDF: ") );
        Display.print( LDFBuffer );
        Display.print( F("  ") );
        break;
     case 4:
        Display.print( F("  TDF: ") );
        Display.print( TDFBuffer );
        Display.print( F("  ") );
        Display.print( " \n" );
        Display.print( F("> LDF: ") );
        Display.print( LDFBuffer );
        Display.print( F("  ") );
        Display.print( " \n" );     
        Display.print( F("  Back         ") );
        break;     
     default:   
        Display.print( F("  TDF: ") );
        Display.print( TDFBuffer );
        Display.print( F(" ") );
        Display.print( " \n" );
        Display.print( F("  LDF: ") );
        Display.print( LDFBuffer );
        Display.print( F(" ") );
        Display.print( " \n" );     
        Display.print( F("> Back         ") );
        break;
    }

    LastMenuItem = MenuItem;
  }

  if( RevTriggerBounce.rose() )
  {
    MenuItem++;
    if( MenuItem > 5 )
      MenuItem = 0;
  }

  if( FireHalfTriggerBounce.rose() )
  {
    switch( MenuItem )
    {
      case 0:

        break;
      case 1:
        EStopOnPusherJam = !EStopOnPusherJam;
        break;
      case 2:
        EStopOnDartJam = !EStopOnDartJam;
        break;
      case 3:
        break;
      case 4:
      default:
        CurrentConfigMenuPage = CONFIG_MENU_MAIN;
        break;
    }
    LastMenuItem = 99;
  }  
}

void Display_ScreenHeader( bool ClearScreen )
{
  static float LastBatteryVoltage = 99.0;
  static unsigned int LastProfile = 9999;
  char Buffer[6];
  
  if( ClearScreen )
  {
    Display.clear();
  }
  // Do not clear screen, voltage has changed
  if( ClearScreen || ( (int)(LastBatteryVoltage*10) != (int)(BatteryCurrentVoltage*10) ) )
  {
    Display.setFont(ZevvPeep8x16);
    Display.setCursor(0, 0);
    sprintf( Buffer, "%3d", (int)(BatteryCurrentVoltage * 10) );
    Buffer[4] = 0;
    Buffer[3] = Buffer[2];
    Buffer[2] = '.';
    //sprintf( Buffer, "%2.1f", BatteryCurrentVoltage );
    Display.print( Buffer );
    Display.print( F("v ") );
    Display.print( BatteryS );
    Display.print( F("s") );
    //DebugPrintln( BatteryCurrentVoltage );
    //DebugPrintln( Buffer );
  }

  
  if( ClearScreen || (LastProfile != CurrentProfile ) )
  {
    Display.setCursor( 70, 1 );
    Display.setFont(font5x7);
    //display.setRow( 1 );
    switch( CurrentProfile )
    {
      case 0:
        Display.print( "Profile A" );
        break;
      case 1:
        Display.print( "Profile B" );
        break;
      case 2:
        Display.print( "Profile C" );
        break;
      default:
        Display.print( "Profile D" );
        break;                 
    }
  }


  LastBatteryVoltage = BatteryCurrentVoltage;
  LastProfile = CurrentProfile;
}

void Display_Normal( bool ClearScreen )
{
  char Buffer[4];
  static int LastDartsInMag = 99;
  static byte LastCurrentFireMode = 99;
  static byte LastTargetDPS = 255;
  static int LastMotorSpeedFull = 999;
  static int LastMotorSpeedHalf = 999;
  static byte LastBurstSize = 99;
  if( ClearScreen )
  {
    DebugPrintln( F("NORMAL MODE!!") );
  }
  if( ClearScreen || (CurrentFireMode != LastCurrentFireMode) || (BurstSize != LastBurstSize) )
  {  
    Display.setFont(ZevvPeep8x16);
    Display.setCursor(0, 2);
    if( CurrentFireMode == FIRE_MODE_SINGLE )
    {
      Display.print( F("Single    ") );
    }
    else if( CurrentFireMode == FIRE_MODE_AUTO )
    {
      Display.print( F("Full Auto ") );
    }
    else
    {
      sprintf( Buffer, "%2d", BurstSize );
      Display.print( F("Burst: ") );
      Display.print( Buffer );
    }
  }
  
  if( ClearScreen || (TargetDPS != LastTargetDPS) )
  {
    Display.setFont(ZevvPeep8x16);
    Display.setCursor(0, 4);
    Display.print( F("DPS: ") );
    if( TargetDPS == 99 )
      Display.print( F("MAX") );
    else 
    {
      sprintf( Buffer, "%2d", TargetDPS );
      Display.print( Buffer );     
    }
  }
  
  if( ClearScreen || (MotorSpeedFull != LastMotorSpeedFull) || (MotorSpeedHalf != LastMotorSpeedHalf) )
  {
    Display.setFont(ZevvPeep8x16);
    Display.setCursor(0, 6);
    Display.print( F("Pwr:") );
    sprintf( Buffer, "%3d", MotorSpeedFull );
    Display.print( Buffer );
    Display.print( "/" );
    sprintf( Buffer, "%3d", MotorSpeedHalf );
    Display.print( Buffer );
  }

  if( ClearScreen || (DartsInMag != LastDartsInMag) )
  {
    Display.setFont(ZevvPeep8x16);
    Display.setCursor(90, 3);
    Display.set2X();
    sprintf( Buffer, "%2d", DartsInMag );
    Display.print( Buffer );
    Display.set1X();  
  }
  LastDartsInMag = DartsInMag;
  LastBurstSize = BurstSize;
  LastTargetDPS = TargetDPS;
  LastMotorSpeedHalf = MotorSpeedHalf;
  LastMotorSpeedFull = MotorSpeedFull;
  LastCurrentFireMode = CurrentFireMode;
}

void Display_LowBatt( bool ClearScreen )
{
  if( ClearScreen )
  {
    DebugPrintln( F("LOW BATTERY!!") );
    DebugPrintln( BatteryCurrentVoltage );

    Display.setFont(ZevvPeep8x16);
    Display.setCursor(0, 2);
    Display.print( F("################\n") );
    Display.print( F("# LOW BATTERY! #\n") );
    Display.print( F("################") ); 
  }
}

void Display_Darts_Out( bool ClearScreen )
{
  if( ClearScreen )
  {
    DebugPrintln( F("DARTS OUT!!") );

    Display.setFont(ZevvPeep8x16);
    Display.setCursor(0, 2);
    Display.print( F("################\n") );
    Display.print( F("#  MAG EMPTY!  #\n") );
    Display.print( F("################") ); 
  }
}

void Display_MagOut( bool ClearScreen )
{
  if( ClearScreen )
  {
    DebugPrintln( F("MAG OUT!!") );

    Display.setFont(ZevvPeep8x16);
    Display.setCursor(0, 2);
    Display.print( F("################\n") );
    Display.print( F("# MAG DROPPED! #\n") );
    Display.print( F("################") ); 
  }
}

void Display_EStop( bool ClearScreen )
{
  if( ClearScreen )
  {
    DebugPrintln( F("E STOP!!") );

    Display.setFont(ZevvPeep8x16);
    Display.setCursor(0, 2);
    Display.print( F("################\n") );
    Display.print( F("#EMERGENCY STOP#\n") );
    Display.print( F("################") ); 
  }
}


/*
 * Debug Stuff
 * 
 * Just a bunch of overloaded functions to print to serial if DebugEnabled is true
 * 
 */

void DebugInit()
{
  if( !DebugEnabled ) return;
  Serial.begin( 57600 );
  //while( !Serial );
}

void DebugPrint( int Val )
{
  if( !DebugEnabled ) return;
  if( Serial.availableForWrite() >= sizeof( Val ) )
    Serial.print( Val );  
}
void DebugPrint( float Val )
{
  if( !DebugEnabled ) return;
  if( Serial.availableForWrite() >= sizeof( Val ) )
    Serial.print( Val );  
}
void DebugPrint( bool Val )
{
  if( !DebugEnabled ) return;
  if( Serial.availableForWrite() >= sizeof( Val ) )
    Serial.print( Val );  
}
void DebugPrint( String Val )
{
  if( !DebugEnabled ) return;
  if( Serial.availableForWrite() >= sizeof( Val ) )
    Serial.print( Val );  
}
void DebugPrintUL( unsigned long Val )
{
  if( !DebugEnabled ) return;
  if( Serial.availableForWrite() >= sizeof( Val ) )
    Serial.print( Val );  
}
void DebugPrint( char *Val )
{
  if( !DebugEnabled ) return;
  char CurrentChar;
  byte Index = 0;
  CurrentChar = *(Val + Index);
  while( (Index < 200) && (CurrentChar != 0) )
  {
    if( Serial.availableForWrite() >= sizeof( CurrentChar ) )
      Serial.print( CurrentChar );
    Index ++;
    CurrentChar = *(Val + Index);
  }
}
void DebugPrint( const __FlashStringHelper* Val )
{
  if( !DebugEnabled ) return;
  if( Serial.availableForWrite() >= sizeof( Val ) )
    Serial.print( Val );
}
void DebugPrintln( int Val )
{
  if( !DebugEnabled ) return;
  if( Serial.availableForWrite() >= sizeof( Val ) )
    Serial.println( Val );  
}
void DebugPrintln( float Val )
{
  if( !DebugEnabled ) return;
  if( Serial.availableForWrite() >= sizeof( Val ) )
    Serial.println( Val );  
}
void DebugPrintln( bool Val )
{
  if( !DebugEnabled ) return;
  if( Serial.availableForWrite() >= sizeof( Val ) )
    Serial.println( Val );  
}
void DebugPrintln( String Val )
{
  if( !DebugEnabled ) return;
  if( Serial.availableForWrite() >= sizeof( Val ) )
    Serial.println( Val );  
}
void DebugPrintULln( unsigned long Val )
{
  if( !DebugEnabled ) return;
  if( Serial.availableForWrite() >= sizeof( Val ) )
    Serial.println( Val );  
}
void DebugPrintln( char *Val )
{
  if( !DebugEnabled ) return;
  char CurrentChar;
  byte Index = 0;
  CurrentChar = *(Val + Index);
  while( (Index < 200) && (CurrentChar != 0) )
  {
    if( Serial.availableForWrite() >= sizeof( CurrentChar ) )
      Serial.print( CurrentChar );
    Index ++;
    CurrentChar = *(Val + Index);
  }
  if( Serial.availableForWrite() >= sizeof( '\n' ) )
    Serial.println( );
}
void DebugPrintln( const __FlashStringHelper* Val )
{
  if( !DebugEnabled ) return;
  if( Serial.availableForWrite() >= sizeof( Val ) )
    Serial.println( Val );
}
