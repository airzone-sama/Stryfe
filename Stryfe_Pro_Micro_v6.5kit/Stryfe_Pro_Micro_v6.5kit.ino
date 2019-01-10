#include <Servo.h>
#include "Bounce2.h"

int BatteryS = 3;
#define MotorKV 3000 
#define LOW_SPEED_REV 45
#define HIGH_SPEED_REV 100

#define PIN_REVTRIGGER 2 
#define PIN_REVDETECTOR A2
#define PIN_MAINMOTOR1 9  
#define PIN_MAINMOTOR2 10
#define PIN_BATTERYDETECT A1
//#define PIN_JAMDOOR 20
#define PIN_SPEEDPOT A3

// Anything more than this is detected as pulling the 2-stage trigger
#define REV_DETECTOR_THRESHOLD (float)0.7

// Servo Objects
Servo MainMotor1; 
Servo MainMotor2;

/*
 * New style acceleration code
 */

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


// Physical Switch Status
bool RevTriggerPressed = false; // Rev Trigger is Depressed
bool DetectingRev = false; // Full auto kit is revving

// Debounce Variables
#define DebounceWindow 5 // Debounce Window = 5ms
Bounce RevTriggerBounce = Bounce();

// Battery Monitoring
#define BATTERY_2S_MIN 6.0
#define BATTERY_2S_MAX 8.4
#define BATTERY_3S_MIN 9.0
#define BATTERY_3S_MAX 12.6
#define BATTERY_4S_MIN 12.0
#define BATTERY_4S_MAX 16.8
#define BATTERY_CALFACTOR 0.45 // Adjustment for calibration.
float BatteryMaxVoltage;
float BatteryMinVoltage;
float BatteryCurrentVoltage;
bool BatteryFlat = false;


// System Modes
#define MODE_CONFIG 1
#define MODE_LOWBATTERY 3
#define MODE_NORMAL 4
int CurrentSystemMode = MODE_NORMAL;


void setup() 
{
  float MotorRPM;

  // Set up comms
  Serial.begin(57600);
  Serial.println( F("Booting.. ") );   

  Serial.println( F("Configuring Debouncing") );
  pinMode(PIN_REVTRIGGER, INPUT_PULLUP);
  RevTriggerBounce.attach( PIN_REVTRIGGER );
  RevTriggerBounce.interval( DebounceWindow );

  Serial.println( F("Debouncing Configured") );

  // Set up battery monitoring
  pinMode( PIN_BATTERYDETECT, INPUT );    

  // Set up rev detection circuit
  pinMode( PIN_REVDETECTOR, INPUT );

  // Set up the pot
  pinMode( PIN_SPEEDPOT, INPUT );

  Serial.println( F("Initialising ESC") );
  // Set up motors
  MainMotor1.attach(PIN_MAINMOTOR1);
  MainMotor2.attach(PIN_MAINMOTOR2);
  // Arm ESC's
  MainMotor1.writeMicroseconds(MinMotorSpeed);
  MainMotor2.writeMicroseconds(MinMotorSpeed);
  delay(3000);   // Wait for ESC to initialise (7 seconds)
  Serial.println( F("ESC Initialised") );

  if( BatteryS == 3 )
  {
    BatteryMinVoltage = BATTERY_3S_MIN;
    BatteryMaxVoltage = BATTERY_3S_MAX;

    DecelerateTime = MOTOR_SPINDOWN_3S;
    AccelerateTime = MOTOR_SPINUP_3S;    
  }
  else
  {
    BatteryMinVoltage = BATTERY_4S_MIN;
    BatteryMaxVoltage = BATTERY_4S_MAX;    

    DecelerateTime = MOTOR_SPINDOWN_4S;
    AccelerateTime = MOTOR_SPINUP_4S;    
  }

  CalculateRampRates();

  Serial.println( F("Booted.") );
}

/*
 * This is a boot time init sub to calcualte the Acceleration and 
 * deceleration ramp rates of the motors.
 * 
 */
void CalculateRampRates()
{
  long SpeedRange = (long)(MaxMotorSpeed - MinMotorSpeed) * 1000; // Multiply by 1000 to give some resolution due to integer calculations
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


  Serial.print( "Ramp Up per MS = " );
  Serial.println( MotorRampUpPerMS );

  Serial.print( "Ramp Down per MS = " );
  Serial.println( MotorRampDownPerMS );

  Serial.print( "AccelerateTime = " );
  Serial.println( AccelerateTime );

  Serial.print( "DecelerateTime = " );
  Serial.println( DecelerateTime );

  Serial.print( "SpeedRange = " );
  Serial.println( SpeedRange );
}


/*
 * Read the sensor / button state for all buttons, including software debounce.
 */
void ProcessButtons()
{
  RevTriggerBounce.update(); // Update the pin bounce state
  RevTriggerPressed = !(RevTriggerBounce.read());

//  JamDoorBounce.update(); // Update the pin bounce state
//  JamDoorPressed = !(JamDoorBounce.read()); 

  float SensorValue = analogRead( PIN_REVDETECTOR );
  float CurrentVoltage = 0.0;
  CurrentVoltage = ((SensorValue * 5.0)  / 1024.0 * (float)((47.0 + 10.0) / 10.0)) + BATTERY_CALFACTOR;  // Voltage dividor - 47k and 10k

  if( CurrentVoltage <= REV_DETECTOR_THRESHOLD)
  {
    DetectingRev = false;
  }
  else
  {
    DetectingRev = true;
  }

  //if( DetectingRev )
    //Serial.println( CurrentVoltage );
}


/*
 * Account for changes to the rotary encoder and feed in a new max motor speed.
 */
// We need to set the Target Motor Speed here.
void ProcessSpeedControl()
{
  static byte LastSetMaxSpeed = 100;
  int CurrentPotInput = 0;

  if( CommandRev == COMMAND_REV_HALF ) SetMaxSpeed = LOW_SPEED_REV;
  if( CommandRev == COMMAND_REV_FULL ) 
  {
    CurrentPotInput = analogRead( PIN_SPEEDPOT );
    SetMaxSpeed = map( CurrentPotInput, 0, 1023, 35, 110 ); // Pull out of bounds so that the ends of the pot are zones.
    //SetMaxSpeed = MotorSpeedFull;
  }
  if( CommandRev == COMMAND_REV_NONE ) SetMaxSpeed = 0;

  if( LastSetMaxSpeed == SetMaxSpeed ) return; // Speed hasn't changed

  if( CommandRev > COMMAND_REV_NONE ) 
  {
    SetMaxSpeed = constrain( SetMaxSpeed, 30, 100 ); // Constrain between 30% and 100%
  }
  
  TargetMotorSpeed = map( SetMaxSpeed, 0, 100, MinMotorSpeed, MaxMotorSpeed );  // Find out our new target speed.

  LastSetMaxSpeed = SetMaxSpeed;

  Serial.print( F("New max speed % = ") );
  Serial.println( SetMaxSpeed );

  Serial.print( F("New target speed = ") );
  Serial.println( TargetMotorSpeed );


  Serial.print( F("CommandRev = ") );
  Serial.println( CommandRev );
  
} 

void ProcessRevCommand()
{

  if( RevTriggerPressed )
    CommandRev = COMMAND_REV_FULL;
  else if( DetectingRev )
    CommandRev = COMMAND_REV_HALF;
  else
    CommandRev = COMMAND_REV_NONE;
  
}

void loop() 
{

  ProcessBatteryMonitor();
  ProcessButtons();
  ProcessSystemMode();

  // Only process firing input commands when in MODFE_NORMAL.. Otherwise allow for current cycle to finish.
  if( CurrentSystemMode == MODE_NORMAL )
  {
    // Perform command logic
    ProcessRevCommand();   
  } 
  else
  {
    // Decelerater the motors
    CommandRev = COMMAND_REV_NONE;
    Serial.print( "CurrentSystemMode = ");
    Serial.println( CurrentSystemMode );
  }

  // Calculate new timing variables

  if( PrevCommandRev != CommandRev )
  {
    TimeLastMotorSpeedChanged = millis();
    PrevCommandRev = CommandRev;
  }
  
  //ProcessManualSpeedControl();

  // Process speed control  
  ProcessSpeedControl();
  // Calcualte the new motor speed
  ProcessMotorSpeed();
  // Send the speed to the ESC
  ProcessMainMotors();

}

/*
 * Communicate with the main motors
 */
void ProcessMainMotors()
{
  static long PreviousMotorSpeed = MinMotorSpeed;

  if( PreviousMotorSpeed != CurrentMotorSpeed ) 
  { 
    MainMotor1.writeMicroseconds( CurrentMotorSpeed );
    MainMotor2.writeMicroseconds( CurrentMotorSpeed );
    // Debugging output
    Serial.println(CurrentMotorSpeed);

    PreviousMotorSpeed = CurrentMotorSpeed;
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
  if( RunNumber <= 10 )
    return;
  RunNumber = 0;
  
  #define NUM_SAMPLES 10
  static int CollectedSamples = 0;
  static float SampleAverage = 0;
  float SensorValue = analogRead( PIN_BATTERYDETECT );
  if( CollectedSamples < NUM_SAMPLES )
  {
    CollectedSamples ++;
    SampleAverage += SensorValue;
  }
  else
  {
    BatteryCurrentVoltage = (((float)SampleAverage / (float)CollectedSamples * 5.0)  / 1024.0 * (float)((47.0 + 10.0) / 10.0)) + BATTERY_CALFACTOR;  // Voltage dividor - 47k and 10k
    if( BatteryCurrentVoltage < BatteryMinVoltage )
    {
      if( BatteryCurrentVoltage > 1.0 ) // If the current voltage is 0, we are probably debugging
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
    CollectedSamples = 0;
    SampleAverage = 0;
  }

  //Serial.println( BatteryFlat );
}

void ProcessSystemMode()
{
  if( BatteryFlat ) // Battery low
  {
    CurrentSystemMode = MODE_LOWBATTERY;
    return;
  }
  
  CurrentSystemMode = MODE_NORMAL;
}
