/*
   UDP Autosteer code for Teensy 4.1
   For AgOpenGPS
   01 Feb 2022
   Like all Arduino code - copied from somewhere else :)
   So don't claim it as your own
*/

// Part of TFF v0.3.7. Was "originally added in UM982 Fallback Firmware
// v0.7, carried over unchanged" through v0.2 — no longer accurate as
// of v0.3: the WAS calculation gained a WasSource == WAS_SOURCE_KEYA
// branch, checked first, before the pre-existing Brand-based branching
// (physical ADS1115 WAS for BRAND_NONE, CAN valve feedback for every
// other Brand — both still exactly as before, untouched). See the WAS
// source selection comment block in the main .ino for the full
// rationale, and zKeya.ino for keyaGetSteerAngle().

////////////////// User Settings /////////////////////////

//How many degrees before decreasing Max PWM
#define LOW_HIGH_DEGREES 3.0

/*  PWM Frequency ->
     490hz (default) = 0
     122hz = 1
     3921hz = 2
*/
#define PWM_Frequency 0

/////////////////////////////////////////////

// if not in eeprom, overwrite
#define EEP_Ident 2400

//   ***********  Motor drive connections  **************888
//Connect ground only for cytron, Connect Ground and +5v for IBT2

//Dir1 for Cytron Dir, Both L and R enable for IBT2
#define DIR1_RL_ENABLE  4

//PWM1 for Cytron PWM, Left PWM for IBT2
#define PWM1_LPWM  2

//Not Connected for Cytron, Right PWM for IBT2
#define PWM2_RPWM  3

//--------------------------- Switch Input Pins ------------------------
#define STEERSW_PIN 32
#define WORKSW_PIN 34
#define REMOTE_PIN 37

//Define sensor pin for current or pressure sensor
#define CURRENT_SENSOR_PIN A17
#define PRESSURE_SENSOR_PIN A10

#define CONST_180_DIVIDED_BY_PI 57.2957795130823

#include <Wire.h>
#include <EEPROM.h>
#include "zADS1115.h"
ADS1115_lite adc(ADS1115_DEFAULT_ADDRESS);     // Use this for the 16-bit version ADS1115

#include <IPAddress.h>
#include "BNO08x_AOG.h"

#ifdef ARDUINO_TEENSY41
// ethernet
#include <NativeEthernet.h>
#include <NativeEthernetUdp.h>
#endif

#ifdef ARDUINO_TEENSY41
//uint8_t Ethernet::buffer[200]; // udp send and receive buffer
uint8_t autoSteerUdpData[UDP_TX_PACKET_MAX_SIZE];  // Buffer For Receiving UDP Data
#endif

//loop time variables in microseconds
const uint16_t LOOP_TIME = 25;  //40Hz
uint32_t autsteerLastTime = LOOP_TIME;
uint32_t currentTime = LOOP_TIME;

const uint16_t WATCHDOG_THRESHOLD = 100;
const uint16_t WATCHDOG_FORCE_VALUE = WATCHDOG_THRESHOLD + 2; // Should be greater than WATCHDOG_THRESHOLD
uint8_t watchdogTimer = WATCHDOG_FORCE_VALUE;

//Heart beat hello AgIO
uint8_t helloFromIMU[] = { 128, 129, 121, 121, 5, 0, 0, 0, 0, 0, 71 };
uint8_t helloFromAutoSteer[] = { 0x80, 0x81, 126, 126, 5, 0, 0, 0, 0, 0, 71 };
int16_t helloSteerPosition = 0;

//fromAutoSteerData FD 253 - ActualSteerAngle*100 -5,6, SwitchByte-7, pwmDisplay-8
uint8_t PGN_253[] = {0x80,0x81, 126, 0xFD, 8, 0, 0, 0, 0, 0,0,0,0, 0xCC };
int8_t PGN_253_Size = sizeof(PGN_253) - 1;

//fromAutoSteerData FD 250 - sensor values etc
uint8_t PGN_250[] = { 0x80,0x81, 126, 0xFA, 8, 0, 0, 0, 0, 0,0,0,0, 0xCC };
int8_t PGN_250_Size = sizeof(PGN_250) - 1;
uint8_t aog2Count = 0;
float sensorReading;
float sensorSample;

elapsedMillis gpsSpeedUpdateTimer = 0;

//EEPROM
int16_t EEread = 0;

//Relays
// Renamed from the original "isRelayActiveHigh" (Chris Kinal's code,
// part of this file since before the CAN work) — it was never actually
// referenced anywhere in this codebase (confirmed by search), and its
// old name collided with the unrelated, actively-used
// aogConfig.isRelayActiveHigh struct member added for the CAN
// hitch/implement lift feature (see AogConfig in the main .ino and
// SetRelaysFendt()/SetRelaysClaas() below). C++ scoping means the two
// never actually conflicted at compile time — a struct member and a
// bare global with the same name are different symbols — but the
// identical name was confusing to read next to each other. Renamed,
// not removed: still unused, still harmless, just no longer misleading.
bool unusedLegacyIsRelayActiveHigh = true;
uint8_t relay = 0, relayHi = 0, uTurn = 0;
uint8_t tram = 0;

//Switches
uint8_t remoteSwitch = 0, workSwitch = 0, steerSwitch = 1, switchByte = 0;

//On Off
uint8_t guidanceStatus = 0;
uint8_t prevGuidanceStatus = 0;
bool guidanceStatusChanged = false;

//speed sent as *10
float gpsSpeed = 0;

//steering variables
float steerAngleActual = 0;
float steerAngleSetPoint = 0; //the desired angle from AgOpen
int16_t steeringPosition = 0; //from steering sensor
float steerAngleError = 0; //setpoint - actual

//pwm variables
int16_t pwmDrive = 0, pwmDisplay = 0;
float pValue = 0;
float errorAbs = 0;
float highLowPerDeg = 0;

//Steer switch button  ***********************************************************************************************************
uint8_t currentState = 1, reading, previous = 0;
uint8_t pulseCount = 0; // Steering Wheel Encoder
bool encEnable = false; //debounce flag
uint8_t thisEnc = 0, lastEnc = 0;

//Variables for settings
struct Storage {
  uint8_t Kp = 40;              // proportional gain
  uint8_t lowPWM = 10;          // band of no action
  int16_t wasOffset = 0;
  uint8_t minPWM = 9;
  uint8_t highPWM = 60;         // max PWM value
  float steerSensorCounts = 30;
  float AckermanFix = 1;        // sent as percent
};  Storage steerSettings;      // 11 bytes

//Variables for settings - 0 is false
struct Setup {
  uint8_t InvertWAS = 0;
  uint8_t IsRelayActiveHigh = 0;    // if zero, active low (default)
  uint8_t MotorDriveDirection = 0;
  uint8_t SingleInputWAS = 1;
  uint8_t CytronDriver = 1;
  uint8_t SteerSwitch = 0;          // 1 if switch selected
  uint8_t SteerButton = 0;          // 1 if button selected
  uint8_t ShaftEncoder = 0;
  uint8_t PressureSensor = 0;
  uint8_t CurrentSensor = 0;
  uint8_t PulseCountMax = 5;
  uint8_t IsDanfoss = 0;
  uint8_t IsUseY_Axis = 0;     //Set to 0 to use X Axis, 1 to use Y avis
}; Setup steerConfig;               // 9 bytes

void steerConfigInit()
{
  if (steerConfig.CytronDriver) 
  {
    pinMode(PWM2_RPWM, OUTPUT);
  }
}

/// <summary>
/// Precomputes highLowPerDeg — how many PWM units per degree of steer
/// error the PID output ramps through in the "low-to-high" PWM
/// interpolation zone (see calcSteeringPID() in c01_AutosteerPID.ino,
/// where this value is actually used). Called once, whenever
/// steerSettings.highPWM/lowPWM change (from EEPROM load or a fresh
/// SETXXX from AGO) — not recomputed every loop iteration, since the
/// two PWM bounds rarely change while the vehicle is actually
/// running.
/// </summary>
void steerSettingsInit()
{
  // for PWM High to Low interpolator
  highLowPerDeg = ((float)(steerSettings.highPWM - steerSettings.lowPWM)) / LOW_HIGH_DEGREES;
}

void autosteerSetup()
{
  //PWM rate settings. Set them both the same!!!!
  /*  PWM Frequency ->
       490hz (default) = 0
       122hz = 1
       3921hz = 2
  */
  if (PWM_Frequency == 0)
  {
    analogWriteFrequency(PWM1_LPWM, 490);
    analogWriteFrequency(PWM2_RPWM, 490);
  }
  else if (PWM_Frequency == 1)
  {
    analogWriteFrequency(PWM1_LPWM, 122);
    analogWriteFrequency(PWM2_RPWM, 122);
  }
  else if (PWM_Frequency == 2)
  {
    analogWriteFrequency(PWM1_LPWM, 3921);
    analogWriteFrequency(PWM2_RPWM, 3921);
  }

  //keep pulled high and drag low to activate, noise free safe
  pinMode(WORKSW_PIN, INPUT_PULLUP);
  pinMode(STEERSW_PIN, INPUT_PULLUP);
  pinMode(REMOTE_PIN, INPUT_PULLUP);
  pinMode(DIR1_RL_ENABLE, OUTPUT);

  // Disable digital inputs for analog input pins
  pinMode(CURRENT_SENSOR_PIN, INPUT_DISABLE);
  pinMode(PRESSURE_SENSOR_PIN, INPUT_DISABLE);

  //set up communication
  Wire1.end();
  Wire1.begin();
    
  // Check ADC 
  if(adc.testConnection())
  {
    Serial.println("ADC Connecton OK");
  }
  else
  {
    Serial.println("ADC Connecton FAILED!");
    Autosteer_running = false;
  }

  //50Khz I2C
  //TWBR = 144;   //Is this needed?

  EEPROM.get(0, EEread);              // read identifier

  if (EEread != EEP_Ident)            // check on first start and write EEPROM
  {
    EEPROM.put(0, EEP_Ident);
    EEPROM.put(10, steerSettings);
    EEPROM.put(40, steerConfig);
    EEPROM.put(60, networkAddress);
    EEPROM.put(AOGCONFIG_EEPROM_ADDR, aogConfig);
  }
  else
  {
    EEPROM.get(10, steerSettings);     // read the Settings
    EEPROM.get(40, steerConfig);
    EEPROM.get(60, networkAddress);
    EEPROM.get(AOGCONFIG_EEPROM_ADDR, aogConfig);
  }

  steerSettingsInit();
  steerConfigInit();

  if (Autosteer_running) 
  {
    Serial.println("Autosteer running, waiting for AgOpenGPS");
    // Autosteer Led goes Red if ADS1115 is found
    digitalWrite(AUTOSTEER_ACTIVE_LED, 0);
    digitalWrite(AUTOSTEER_STANDBY_LED, 1);
  }
  else
  {
    Autosteer_running = false;  //Turn off auto steer if no ethernet (Maybe running T4.0)
//    if(!Ethernet_running)Serial.println("Ethernet not available");
    Serial.println("Autosteer disabled, GPS only mode");   
    return;
  }

  adc.setSampleRate(ADS1115_REG_CONFIG_DR_128SPS); //128 samples per second
  adc.setGain(ADS1115_REG_CONFIG_PGA_6_144V);

}// End of Setup

void autosteerLoop()
{
#ifdef ARDUINO_TEENSY41
  ReceiveUdp();
#endif
  //Serial.println("AutoSteer loop");

  // Loop triggers every 100 msec and sends back gyro heading, and roll, steer angle etc
  currentTime = systick_millis_count;

  if (currentTime - autsteerLastTime >= LOOP_TIME)
  {
    autsteerLastTime = currentTime;

    //reset debounce
    encEnable = true;

    //If connection lost to AgOpenGPS, the watchdog will count up and turn off steering
    if (watchdogTimer++ > 250) watchdogTimer = WATCHDOG_FORCE_VALUE;

    //read all the switches
    workSwitch = digitalRead(WORKSW_PIN);  // read work switch
    // Ported from the original CAN implementation: if the tractor's own
    // CAN bus reports a work-switch/section signal (e.g. rear hitch
    // position via ISOBUS/K-Bus — see workCAN in zCAN_All_Brands.ino),
    // let it override the physical WORKSW_PIN reading. workCAN was
    // already being correctly SET from CAN messages in our code, but
    // never read anywhere in this file — this line was the missing
    // connection. Harmless when Brand==BRAND_NONE: workCAN then simply
    // never gets set to 1 by anything (zCAN_All_Brands.ino's CAN_setup/
    // K_Receive/VBus_Receive are all skipped in that mode), so this
    // line has no effect and workSwitch behaves exactly as before.
    if (workCAN == 1) workSwitch = 0;      // CAN work switch ON

    if (steerConfig.SteerSwitch == 1)         //steer switch on - off
    {
      steerSwitch = digitalRead(STEERSW_PIN); //read auto steer enable switch open = 0n closed = Off
    }
    else if (steerConfig.SteerButton == 1)    //steer Button momentary
    {
      reading = digitalRead(STEERSW_PIN);
      if (reading == LOW && previous == HIGH)
      {
        if (currentState == 1)
        {
          currentState = 0;
          steerSwitch = 0;
        }
        else
        {
          currentState = 1;
          steerSwitch = 1;
        }
      }
      previous = reading;
    }
    else                                      // No steer switch and no steer button
    {
      // So set the correct value. When guidanceStatus = 1,
      // it should be on because the button is pressed in the GUI
      // But the guidancestatus should have set it off first
      if (guidanceStatusChanged && guidanceStatus == 1 && steerSwitch == 1 && previous == 0)
      {
        steerSwitch = 0;
        previous = 1;
      }

      // This will set steerswitch off and make the above check wait until the guidanceStatus has gone to 0
      if (guidanceStatusChanged && guidanceStatus == 0 && steerSwitch == 0 && previous == 1)
      {
        steerSwitch = 1;
        previous = 0;
      }
    }

    if (steerConfig.ShaftEncoder && pulseCount >= steerConfig.PulseCountMax)
    {
      steerSwitch = 1; // reset values like it turned off
      currentState = 1;
      previous = 0;
    }

    // Pressure sensor?
    if (steerConfig.PressureSensor)
    {
      sensorSample = (float)analogRead(PRESSURE_SENSOR_PIN);
      sensorSample *= 0.25;
      sensorReading = sensorReading * 0.6 + sensorSample * 0.4;
      if (sensorReading >= steerConfig.PulseCountMax)
      {
          steerSwitch = 1; // reset values like it turned off
          currentState = 1;
          previous = 0;
      }
    }

    // Current sensor?
    if (steerConfig.CurrentSensor)
    {
      sensorSample = (float)analogRead(CURRENT_SENSOR_PIN);
      sensorSample = (abs(775 - sensorSample)) * 0.5;
      sensorReading = sensorReading * 0.7 + sensorSample * 0.3;    
      sensorReading = min(sensorReading, 255);

      if (sensorReading >= steerConfig.PulseCountMax)
      {
          steerSwitch = 1; // reset values like it turned off
          currentState = 1;
          previous = 0;
      }
    }

    // --------CAN CutOut-------------------------- 
    // Ported directly from the original CAN implementation
    // (Autosteer_AOGv5_Teensy4_1UDP_SteerReadyCAN_AIOModified.ino),
    // same variable names, same two known-ready sentinel values, same
    // comment. This was the missing link a tester's bug report traced
    // back to: the tractor's own CAN system correctly reports
    // steeringValveReady when the driver grabs the wheel (or otherwise
    // takes over), and that DID stop the physical valve — but nothing
    // in this file ever read steeringValveReady, so steerSwitch/
    // switchByte never reflected it, and AgOpenGPS's own UI never
    // found out the tractor had disengaged itself.
    //
    // The two "ready" values: 16 is confirmed and documented (see
    // zCAN_All_Brands.ino, Brand==3/5 Fendt/FendtOne — explicitly set
    // to 16 there when the valve reports ready). 20 is NOT documented
    // anywhere in the original source — it's used in this exact check
    // but never explained which brand sets it or why. Ported as-is
    // since the pattern (treat anything other than a known-ready value
    // as not-ready) is sound regardless, but flagging this openly:
    // if disengage-while-turning doesn't work correctly on a non-Fendt
    // CAN brand, verifying what that brand's "ready" byte actually is
    // (CAN sniffing on the bench) is the place to look first.
    //
    // DEVIATION FROM ORIGINAL: gated on Brand != BRAND_NONE, which the
    // original never needed (that codebase was CAN-only hardware).
    // Without this gate, steeringValveReady defaults to 0 in non-CAN
    // mode and is never touched by anything (zCAN_All_Brands.ino is
    // fully skipped) — 0 is neither 16 nor 20, so an ungated version of
    // this check would force steerSwitch=1 on every single loop
    // iteration in non-CAN mode, permanently preventing autosteer from
    // ever engaging at all. This gate is required, not optional.
    if (Brand != BRAND_NONE && steeringValveReady != 20 && steeringValveReady != 16)
    {
        steerSwitch = 1; // reset values like it turned off
        currentState = 1;
        previous = HIGH;  // note: original uses HIGH here specifically,
                          // unlike the ShaftEncoder/pressure/current
                          // blocks above which use 0 — preserved exactly
                          // as in the source rather than harmonized
    }

    remoteSwitch = digitalRead(REMOTE_PIN); //read auto steer enable switch open = 0n closed = Off
    switchByte = 0;
    switchByte |= (remoteSwitch << 2); //put remote in bit 2
    switchByte |= (steerSwitch << 1);   //put steerswitch status in bit 1 position
    switchByte |= workSwitch;

    /*
      #if Relay_Type == 1
        SetRelays();       //turn on off section relays
      #elif Relay_Type == 2
        SetuTurnRelays();  //turn on off uTurn relays
      #endif
    */

    //get steering position
    // ---------------------------------------------------------------
    // WAS source: physical ADS1115 sensor (BRAND_NONE, classic PWM/
    // relay steering) vs CAN steering valve feedback (any other Brand,
    // 0-7). See the WAS source selection comment block in the main
    // .ino for why these must never both write steerAngleActual at
    // once. Brand 7 is grouped with the other CAN brands for now
    // (uses the generic AgOpenGPS CAN protocol/ID) — its own upstream
    // reference implementation left this branch unimplemented/empty,
    // so this is our best-evidence default; revisit if that turns out
    // to be wrong once brand 7 is actually used in the field.
    // ---------------------------------------------------------------
    // Keya encoder as WAS — checked first, takes priority over the
    // existing Brand-based branching below (physical potentiometer vs
    // CAN-brand valve feedback) entirely. Reuses steerSensorCounts/
    // AckermanFix — see keyaGetSteerAngle() (zKeya.ino) for why that's
    // safe (identical formula to the physical-WAS path below, so
    // switching WasSource on an already-calibrated vehicle doesn't
    // need a second calibration pass).
    if (WasSource == WAS_SOURCE_KEYA)
    {
      steerAngleActual = keyaGetSteerAngle();
    }
    else if (Brand == BRAND_NONE)
    {
      if (steerConfig.SingleInputWAS)   //Single Input ADS
      {
        adc.setMux(ADS1115_REG_CONFIG_MUX_SINGLE_0);
        steeringPosition = adc.getConversion();
        adc.triggerConversion();//ADS1115 Single Mode

        steeringPosition = (steeringPosition >> 1); //bit shift by 2  0 to 13610 is 0 to 5v
        helloSteerPosition = steeringPosition - 6800;
      }
      else    //ADS1115 Differential Mode
      {
        adc.setMux(ADS1115_REG_CONFIG_MUX_DIFF_0_1);
        steeringPosition = adc.getConversion();
        adc.triggerConversion();

        steeringPosition = (steeringPosition >> 1); //bit shift by 2  0 to 13610 is 0 to 5v
        helloSteerPosition = steeringPosition - 6800;
      }

      //DETERMINE ACTUAL STEERING POSITION

      //convert position to steer angle. 32 counts per degree of steer pot position in my case
      //  ***** make sure that negative steer angle makes a left turn and positive value is a right turn *****
      if (steerConfig.InvertWAS)
      {
        steeringPosition = (steeringPosition - 6805  - steerSettings.wasOffset);   // 1/2 of full scale
        steerAngleActual = (float)(steeringPosition) / -steerSettings.steerSensorCounts;
      }
      else
      {
        steeringPosition = (steeringPosition - 6805  + steerSettings.wasOffset);   // 1/2 of full scale
        steerAngleActual = (float)(steeringPosition) / steerSettings.steerSensorCounts;
      }
    }
    else
    {
      //DETERMINE ACTUAL STEERING POSITION  *********From CAN-Bus************
      // While not actively steering, mirror the valve's own reported
      // position back as the target — avoids a jump the instant
      // steering engages (matches upstream reference behaviour).
      if (intendToSteer == 0) setCurve = estCurve;

      steeringPosition = (setCurve - 32128 + steerSettings.wasOffset);
      if (Brand == 3 || Brand == 5)  // Fendt / FendtOne: curve uses 10x scale
        steerAngleActual = (float)(steeringPosition) / (steerSettings.steerSensorCounts * 10);
      else
        steerAngleActual = (float)(steeringPosition) / steerSettings.steerSensorCounts;

      // Raw curve value isn't linear in degrees — run it through the
      // calibration table (see main .ino for inputWAS/outputWAS[Fendt]).
      float mappedWAS;
      if (Brand == 3 || Brand == 5)
        mappedWAS = multiMap<float>(steerAngleActual, inputWAS, outputWASFendt, 21);
      else
        mappedWAS = multiMap<float>(steerAngleActual, inputWAS, outputWAS, 21);
      steerAngleActual = mappedWAS;
    }

    //Ackerman fix
    if (steerAngleActual < 0) steerAngleActual = (steerAngleActual * steerSettings.AckermanFix);

    // Called here, AFTER AckermanFix above — NOT immediately after
    // keyaGetSteerAngle() in the WAS_SOURCE_KEYA branch above, where
    // it was first placed. CAUGHT DURING THE DEEP VARIABLE-FLOW
    // REVIEW, FIXED BEFORE FIRST USE: AckermanFix only applies to
    // NEGATIVE steerAngleActual values, so reading it too early would
    // have fed keyaAutoZeroUpdate() a systematically different value
    // than the rest of the steering logic uses, specifically for
    // negative angles — small (AckermanFix is close to 1.0), but a
    // genuine, avoidable inconsistency for a mechanism whose entire
    // point is precision. See zKeyaAutoZero.ino for the full
    // mechanism this call drives.
    if (WasSource == WAS_SOURCE_KEYA)
    {
      keyaAutoZeroUpdate();
    }

    // WAS plausibility check — diagnostic only (see the block comment
    // above setup() in the main .ino: reported via PDIAG, never acted
    // on). Two independent checks, either one sets the flag:
    //   - out of range: more than 15° beyond the calibrated wasLeft/
    //     wasRight endpoints — a real wheel shouldn't physically read
    //     there, so this is more likely a wiring/sensor fault than a
    //     legitimate extreme steering angle.
    //   - implausible jump: more than 20° change in a single loop()
    //     iteration — loop() runs far faster than any physical wheel
    //     can move, so a jump this size in one iteration means the
    //     signal itself glitched, not that the wheel actually moved
    //     that fast.
    // Thresholds are deliberately generous (favouring missed
    // detections over false alarms) since this is diagnostic-only —
    // a false positive here only adds a harmless log line, but being
    // too sensitive would make the log noisy enough to defeat its own
    // purpose.
    {
        static float prevSteerAngleForPlausibility = 0.0f;
        static bool  firstPlausibilityCheck = true;
        wasImplausible = false;
        if (!firstPlausibilityCheck)
        {
            float jump = fabsf(steerAngleActual - prevSteerAngleForPlausibility);
            bool outOfRange = (steerAngleActual < wasLeft - 15.0f) || (steerAngleActual > wasRight + 15.0f);
            bool suddenJump = (jump > 20.0f);
            wasImplausible = outOfRange || suddenJump;
        }
        firstPlausibilityCheck = false;
        prevSteerAngleForPlausibility = steerAngleActual;
    }

    // Keya-as-WAS safety gate — ported directly from the same
    // mechanism the original auto-zero repo uses: force the watchdog
    // into its "tripped" state (identical to how a lost-communication
    // timeout already disables steering elsewhere in this same
    // function) until the very first zero-point has been established.
    // Without this, a fresh boot's steerAngleActual would be based on
    // an arbitrary, never-calibrated reference — the root cause
    // behind the original, real "motor just spins non-stop" field
    // report this whole file exists to fix. Only applies when Keya is
    // actually configured as the WAS source; every other WasSource
    // path is completely unaffected.
    // CAUGHT AT COMPILE TIME (v0.3.10 shipped with this broken — a
    // genuine oversight, not a design change): wasZeroDone is declared
    // in zKeyaAutoZero.ino, which compiles AFTER this file
    // alphabetically ("c00_Autosteer" < "zKeyaAutoZero") — direct
    // access here never could have worked. azGetWasZeroDone() already
    // exists (built for PDIAG reporting) — reused directly rather than
    // adding a near-duplicate getter.
    if (WasSource == WAS_SOURCE_KEYA && azGetWasZeroDone() == 0)
    {
      watchdogTimer = WATCHDOG_FORCE_VALUE;
    }

    if (watchdogTimer < WATCHDOG_THRESHOLD)
    {
      //Enable H Bridge for IBT2, hyd aux, etc for cytron
      if (steerConfig.CytronDriver)
      {
        if (steerConfig.IsRelayActiveHigh)
        {
          digitalWrite(PWM2_RPWM, 0);
        }
        else
        {
          digitalWrite(PWM2_RPWM, 1);
        }
      }
      else digitalWrite(DIR1_RL_ENABLE, 1);

      steerAngleError = steerAngleActual - steerAngleSetPoint;   //calculate the steering error
      //if (abs(steerAngleError)< steerSettings.lowPWM) steerAngleError = 0;

      calcSteeringPID();  //do the pid
      motorDrive();       //out to motors the pwm value
      intendToSteer = 1;  //CAN: we intend to steer — see VBus_Send()
      // Autosteer Led goes GREEN if autosteering

      digitalWrite(AUTOSTEER_ACTIVE_LED, 1);
      digitalWrite(AUTOSTEER_STANDBY_LED, 0);
    }
    else
    {
      //we've lost the comm to AgOpenGPS, or just stop request
      //Disable H Bridge for IBT2, hyd aux, etc for cytron
      //
      // ****** If CAN engage is ON (1), don't turn off safety valve ******
      // Ported directly from the original CAN implementation (same
      // comment, same engageCAN==0 guard). engageCAN reflects the
      // tractor's own physical CAN engage button being actively held
      // (see K_Receive()/VBus_Receive() in zCAN_All_Brands.ino) — while
      // that's true, the original deliberately skips disabling the
      // valve enable line here, even on a watchdog/comm-loss condition.
      //
      // DEVIATION FROM ORIGINAL: the original only had one output path
      // (PWM2_RPWM, Cytron-style) to guard. This codebase (unchanged
      // from Chris Kinal's original, predating the CAN merge) has two —
      // Cytron and IBT2/DIR1_RL_ENABLE — so the guard is applied to
      // both rather than just one, to preserve the same intent
      // regardless of which output type is configured. Also relies on
      // engageCAN being correctly reset back to 0 after ~1s via
      // relayTime (see loop() in the main .ino) — without that reset
      // this guard would hold the valve open permanently after the
      // first CAN engage, which is why that reset was added alongside
      // this change rather than left for later.
      if (engageCAN == 0)
      {
        if (steerConfig.CytronDriver)
        {
          if (steerConfig.IsRelayActiveHigh)
          {
            digitalWrite(PWM2_RPWM, 1);
          }
          else
          {
            digitalWrite(PWM2_RPWM, 0);
          }
        }
        else digitalWrite(DIR1_RL_ENABLE, 0); //IBT2
      }
      intendToSteer = 0;  //CAN: not steering — see VBus_Send()

      pwmDrive = 0; //turn off steering motor
      motorDrive(); //out to motors the pwm value
      pulseCount = 0;
      // Autosteer Led goes back to RED when autosteering is stopped
      digitalWrite (AUTOSTEER_STANDBY_LED, 1);
      digitalWrite (AUTOSTEER_ACTIVE_LED, 0);
    }
  } //end of timed loop

  //This runs continuously, outside of the timed loop, keeps checking for new udpData, turn sense
  //delay(1);

  // Speed pulse
  if (gpsSpeedUpdateTimer < 1000)
  {
      if (speedPulseUpdateTimer > 200) // 100 (10hz) seems to cause tone lock ups occasionally
      {
          speedPulseUpdateTimer = 0;

          //130 pp meter, 3.6 kmh = 1 m/sec = 130hz or gpsSpeed * 130/3.6 or gpsSpeed * 36.1111
          //gpsSpeed = ((float)(autoSteerUdpData[5] | autoSteerUdpData[6] << 8)) * 0.1;
          float speedPulse = gpsSpeed * 36.1111;

          //Serial.print(gpsSpeed); Serial.print(" -> "); Serial.println(speedPulse);

          if (gpsSpeed > 0.11) { // 0.10 wasn't high enough
              tone(velocityPWM_Pin, uint16_t(speedPulse));
          }
          else {
              noTone(velocityPWM_Pin);
          }
      }
  }
  else  // if gpsSpeedUpdateTimer hasn't update for 1000 ms, turn off speed pulse
  {
      noTone(velocityPWM_Pin);
  }

  if (encEnable)
  {
    thisEnc = digitalRead(REMOTE_PIN);
    if (thisEnc != lastEnc)
    {
      lastEnc = thisEnc;
      if ( lastEnc) EncoderFunc();
    }
  }

} // end of main loop

int currentRoll = 0;
int rollLeft = 0;
int steerLeft = 0;

#ifdef ARDUINO_TEENSY41
// UDP Receive
void ReceiveUdp()
{
    // When ethernet is not running, return directly. parsePacket() will block when we don't
    if (!Ethernet_running)
    {
        return;
    }

    uint16_t len = Eth_udpAutoSteer.parsePacket();

    // if (len > 0)
    // {
    //  Serial.print("ReceiveUdp: ");
    //  Serial.println(len);
    // }

    // Check for len > 4, because we check byte 0, 1, 3 and 3
    if (len > 4)
    {
        Eth_udpAutoSteer.read(autoSteerUdpData, UDP_TX_PACKET_MAX_SIZE);

        if (autoSteerUdpData[0] == 0x80 && autoSteerUdpData[1] == 0x81 && autoSteerUdpData[2] == 0x7F) //Data
        {
            if (autoSteerUdpData[3] == 0xFE && Autosteer_running)  //254
            {
                gpsSpeed = ((float)(autoSteerUdpData[5] | autoSteerUdpData[6] << 8)) * 0.1;
                gpsSpeedUpdateTimer = 0;

                prevGuidanceStatus = guidanceStatus;

                guidanceStatus = autoSteerUdpData[7];
                guidanceStatusChanged = (guidanceStatus != prevGuidanceStatus);

                //Bit 8,9    set point steer angle * 100 is sent
                steerAngleSetPoint = ((float)(autoSteerUdpData[8] | ((int8_t)autoSteerUdpData[9]) << 8)) * 0.01; //high low bytes

                //Serial.print("steerAngleSetPoint: ");
                //Serial.println(steerAngleSetPoint);

                //Serial.println(gpsSpeed);

                // Ported from the original CAN implementation: Fendt/
                // FendtOne (Brand 3/5) are exempt from the speed<0.1
                // disengage rule — their own factory CAN system handles
                // this internally, and this codebase previously applied
                // the cutoff unconditionally to every brand including
                // Fendt, a real behavioural difference from the source
                // material found during a full re-comparison against it.
                if ((bitRead(guidanceStatus, 0) == 0) || (steerSwitch == 1))
                {
                    watchdogTimer = WATCHDOG_FORCE_VALUE; //turn off steering motor
                }
                else if (Brand != 3 && Brand != 5 && gpsSpeed < 0.1)
                {
                    watchdogTimer = WATCHDOG_FORCE_VALUE; //turn off steering motor
                }
                else          //valid conditions to turn on autosteer
                {
                    watchdogTimer = 0;  //reset watchdog
                }

                //Bit 10 Tram
                tram = autoSteerUdpData[10];

                //Bit 11
                relay = autoSteerUdpData[11];

                //Bit 12
                relayHi = autoSteerUdpData[12];

                //----------------------------------------------------------------------------
                //Serial Send to agopenGPS

                int16_t sa = (int16_t)(steerAngleActual * 100);

                PGN_253[5] = (uint8_t)sa;
                PGN_253[6] = sa >> 8;

                // heading
                PGN_253[7] = (uint8_t)9999;
                PGN_253[8] = 9999 >> 8;

                // roll
                PGN_253[9] = (uint8_t)8888;
                PGN_253[10] = 8888 >> 8;

                PGN_253[11] = switchByte;
                PGN_253[12] = (uint8_t)pwmDisplay;

                //checksum
                int16_t CK_A = 0;
                for (uint8_t i = 2; i < PGN_253_Size; i++)
                    CK_A = (CK_A + PGN_253[i]);

                PGN_253[PGN_253_Size] = CK_A;

                //off to AOG
                SendUdp(PGN_253, sizeof(PGN_253), Eth_ipDestination, portDestination);

                //Steer Data 2 -------------------------------------------------
                if (steerConfig.PressureSensor || steerConfig.CurrentSensor)
                {
                    if (aog2Count++ > 2)
                    {
                        //Send fromAutosteer2
                        PGN_250[5] = (byte)sensorReading;

                        //add the checksum for AOG2
                        CK_A = 0;

                        for (uint8_t i = 2; i < PGN_250_Size; i++)
                        {
                            CK_A = (CK_A + PGN_250[i]);
                        }

                        PGN_250[PGN_250_Size] = CK_A;

                        //off to AOG
                        SendUdp(PGN_250, sizeof(PGN_250), Eth_ipDestination, portDestination);
                        aog2Count = 0;
                    }
                }

                //Serial.println(steerAngleActual);
                //--------------------------------------------------------------------------
            }

            //steer settings
            else if (autoSteerUdpData[3] == 0xFC && Autosteer_running)  //252
            {
                //PID values
                steerSettings.Kp = ((float)autoSteerUdpData[5]);   // read Kp from AgOpenGPS

                steerSettings.highPWM = autoSteerUdpData[6]; // read high pwm

                steerSettings.lowPWM = (float)autoSteerUdpData[7];   // read lowPWM from AgOpenGPS

                steerSettings.minPWM = autoSteerUdpData[8]; //read the minimum amount of PWM for instant on

                float temp = (float)steerSettings.minPWM * 1.2;
                steerSettings.lowPWM = (byte)temp;

                steerSettings.steerSensorCounts = autoSteerUdpData[9]; //sent as setting displayed in AOG

                steerSettings.wasOffset = (autoSteerUdpData[10]);  //read was zero offset Lo

                steerSettings.wasOffset |= (autoSteerUdpData[11] << 8);  //read was zero offset Hi

                steerSettings.AckermanFix = (float)autoSteerUdpData[12] * 0.01;

                //crc
                //autoSteerUdpData[13];

                //store in EEPROM
                EEPROM.put(10, steerSettings);

                // Ported from the original CAN implementation: re-
                // broadcast AGO's PID/config settings onto V_Bus for
                // Brand==7 (AgOpenGPS generic CAN module). canConfig()
                // (zCAN_All_Brands.ino) already existed in this
                // codebase but was never called anywhere — confirmed
                // by grep before this fix. Harmless for every other
                // Brand value, since the condition is never true there.
                if (Brand == 7) canConfig();

                // Re-Init steer settings
                steerSettingsInit();
            }

            else if (autoSteerUdpData[3] == 0xFB)  //251 FB - SteerConfig
            {
                uint8_t sett = autoSteerUdpData[5]; //setting0

                if (bitRead(sett, 0)) steerConfig.InvertWAS = 1; else steerConfig.InvertWAS = 0;
                if (bitRead(sett, 1)) steerConfig.IsRelayActiveHigh = 1; else steerConfig.IsRelayActiveHigh = 0;
                if (bitRead(sett, 2)) steerConfig.MotorDriveDirection = 1; else steerConfig.MotorDriveDirection = 0;
                if (bitRead(sett, 3)) steerConfig.SingleInputWAS = 1; else steerConfig.SingleInputWAS = 0;
                if (bitRead(sett, 4)) steerConfig.CytronDriver = 1; else steerConfig.CytronDriver = 0;
                if (bitRead(sett, 5)) steerConfig.SteerSwitch = 1; else steerConfig.SteerSwitch = 0;
                if (bitRead(sett, 6)) steerConfig.SteerButton = 1; else steerConfig.SteerButton = 0;
                if (bitRead(sett, 7)) steerConfig.ShaftEncoder = 1; else steerConfig.ShaftEncoder = 0;

                steerConfig.PulseCountMax = autoSteerUdpData[6];

                //was speed
                //autoSteerUdpData[7];

                sett = autoSteerUdpData[8]; //setting1 - Danfoss valve etc

                if (bitRead(sett, 0)) steerConfig.IsDanfoss = 1; else steerConfig.IsDanfoss = 0;
                if (bitRead(sett, 1)) steerConfig.PressureSensor = 1; else steerConfig.PressureSensor = 0;
                if (bitRead(sett, 2)) steerConfig.CurrentSensor = 1; else steerConfig.CurrentSensor = 0;
                if (bitRead(sett, 3)) steerConfig.IsUseY_Axis = 1; else steerConfig.IsUseY_Axis = 0;

                //crc
                //autoSteerUdpData[13];

                EEPROM.put(40, steerConfig);

                // Ported from the original CAN implementation — see the
                // identical note in the PGN 0xFC handler above.
                if (Brand == 7) canConfig();

                // Re-Init
                steerConfigInit();

            }//end FB

            // Ported from the original CAN implementation. Neither of
            // these two PGNs existed anywhere in this codebase before
            // this fix — confirmed by grep before implementing. Both
            // feed SetRelaysFendt()/SetRelaysClaas() (below), which in
            // turn use the pressGo/liftGo/pressEnd/liftEnd/pressCSM1/
            // pressCSM2 CAN senders already present in
            // zCAN_All_Brands.ino from the original CAN merge, but
            // never called by anything until now.
            else if (autoSteerUdpData[3] == 0xEF)  //239 Machine Data
            {
                hydLift = autoSteerUdpData[7];
            }//end EF

            else if (autoSteerUdpData[3] == 0xEE)  //238 Machine Settings
            {
                aogConfig.raiseTime = autoSteerUdpData[5];
                aogConfig.lowerTime = autoSteerUdpData[6];
                //aogConfig.enableToolLift = autoSteerUdpData[7]; //This is wrong AgOpen is putting enable in sett,1

                uint8_t sett = autoSteerUdpData[8];  //setting0
                if (bitRead(sett, 0)) aogConfig.isRelayActiveHigh = 1; else aogConfig.isRelayActiveHigh = 0;
                if (bitRead(sett, 1)) aogConfig.enableToolLift = 1; else aogConfig.enableToolLift = 0;

                //save in EEPROM
                EEPROM.put(AOGCONFIG_EEPROM_ADDR, aogConfig);
            }//end EE

            else if (autoSteerUdpData[3] == 200) // Hello from AgIO
            {
                if(Autosteer_running)
                {
                int16_t sa = (int16_t)(steerAngleActual * 100);

                helloFromAutoSteer[5] = (uint8_t)sa;
                helloFromAutoSteer[6] = sa >> 8;

                helloFromAutoSteer[7] = (uint8_t)helloSteerPosition;
                helloFromAutoSteer[8] = helloSteerPosition >> 8;
                helloFromAutoSteer[9] = switchByte;

                SendUdp(helloFromAutoSteer, sizeof(helloFromAutoSteer), Eth_ipDestination, portDestination);
                }
                if(useIMU)
                {
                 SendUdp(helloFromIMU, sizeof(helloFromIMU), Eth_ipDestination, portDestination); 
                }
            }

            else if (autoSteerUdpData[3] == 201)
            {
             //make really sure this is the subnet pgn
             if (autoSteerUdpData[4] == 5 && autoSteerUdpData[5] == 201 && autoSteerUdpData[6] == 201)
             {
              networkAddress.ipOne = autoSteerUdpData[7];
              networkAddress.ipTwo = autoSteerUdpData[8];
              networkAddress.ipThree = autoSteerUdpData[9];
        
              //save in EEPROM and restart
              EEPROM.put(60, networkAddress);
              SCB_AIRCR = 0x05FA0004; //Teensy Reset
              }
            }//end 201

            //whoami
            else if (autoSteerUdpData[3] == 202)
            {
                //make really sure this is the reply pgn
                if (autoSteerUdpData[4] == 3 && autoSteerUdpData[5] == 202 && autoSteerUdpData[6] == 202)
                {
                    IPAddress rem_ip = Eth_udpAutoSteer.remoteIP();

                    //hello from AgIO
                    uint8_t scanReply[] = { 128, 129, Eth_myip[3], 203, 7,
                        Eth_myip[0], Eth_myip[1], Eth_myip[2], Eth_myip[3], 
                        rem_ip[0],rem_ip[1],rem_ip[2], 23 };

                    //checksum
                    int16_t CK_A = 0;
                    for (uint8_t i = 2; i < sizeof(scanReply) - 1; i++)
                    {
                        CK_A = (CK_A + scanReply[i]);
                    }
                    scanReply[sizeof(scanReply)-1] = CK_A;

                    static uint8_t ipDest[] = { 255,255,255,255 };
                    uint16_t portDest = 9999; //AOG port that listens

                    //off to AOG
                    SendUdp(scanReply, sizeof(scanReply), ipDest, portDest);
                }
            }
        } //end if 80 81 7F
    }
}
#endif

#ifdef ARDUINO_TEENSY41
/// <summary>
/// Thin wrapper around Ethernet.h's UDP send — writes a raw byte
/// buffer to the given IP/port and closes the packet immediately.
/// Used throughout this file wherever a reply needs to go back to
/// AGO on the AutoSteer UDP socket (Eth_udpAutoSteer specifically,
/// not the diagnostics/PDIAG socket, which sends independently — see
/// zHandlers.ino). Kept this small and generic deliberately, rather
/// than baking in a specific packet format, since it's reused for
/// several genuinely different reply types.
/// </summary>
void SendUdp(uint8_t *data, uint8_t datalen, IPAddress dip, uint16_t dport)
{
  Eth_udpAutoSteer.beginPacket(dip, dport);
  Eth_udpAutoSteer.write(data, datalen);
  Eth_udpAutoSteer.endPacket();
}
#endif

//ISR Steering Wheel Encoder
void EncoderFunc()
{
  if (encEnable)
  {
    pulseCount++;
    encEnable = false;
  }
}

// =====================================================================
// SetRelaysFendt() / SetRelaysClaas()
// =====================================================================
// Ported directly from the original CAN implementation (same variable
// names, same logic, same comments). Headland hydraulic hitch/implement
// auto-raise/lower via CAN — presses the tractor's own "GO"/"END" (or,
// for Claas, "CSM1"/"CSM2") headland buttons in response to AGO's own
// hitch command (the "relay" byte from PGN 254) or the tractor's own
// hydraulic-lift-detected state (hydLift, from PGN 0xEF), depending on
// aogConfig.isRelayActiveHigh. Only actually presses anything when
// aogConfig.enableToolLift is set (from PGN 0xEE) — matches original.
//
// goDown/endDown (main .ino) and pressGo/liftGo/pressEnd/liftEnd/
// pressCSM1/pressCSM2 (zCAN_All_Brands.ino) all already existed in this
// codebase from the original CAN merge, but nothing ever called these
// two orchestration functions, so none of that CAN-sender code was
// ever actually reachable — confirmed by grep before this fix.
//
// Called from loop() in the main .ino: "if (Brand == 3) SetRelaysFendt();
// if (Brand == 0) SetRelaysClaas();" — same brand gating as the original.
void SetRelaysFendt(void)
{
  if (goDown)  liftGo();   //Lift Go button if pressed - CAN Page
  if (endDown) liftEnd();  //Lift End button if pressed - CAN Page

  //If Invert Relays is selected in hitch settings, Section 1 is used as trigger.
  if (aogConfig.isRelayActiveHigh == 1)
  {
    bitState = (bitRead(relay, 0));
  }
  //If not selected hitch command is used on headland used as Trigger
  else
  {
    if (hydLift == 1) bitState = 1;
    if (hydLift == 2) bitState = 0;
  }
  //Only if tool lift is enabled AgOpen will press headland buttons via CAN
  if (aogConfig.enableToolLift == 1)
  {
    if (bitState  && !bitStateOld) pressGo();  //Press Go button - CAN Page
    if (!bitState && bitStateOld)  pressEnd(); //Press End button - CAN Page
  }

  bitStateOld = bitState;
}

void SetRelaysClaas(void)
{
  //If Invert Relays is selected in hitch settings, Section 1 is used as trigger.
  if (aogConfig.isRelayActiveHigh == 1)
  {
    bitState = (bitRead(relay, 0));
  }
  //If not selected hitch command is used on headland used as Trigger
  else
  {
    if (hydLift == 1) bitState = 1;
    if (hydLift == 2) bitState = 0;
  }
  //Only if tool lift is enabled AgOpen will press headland buttons via CAN
  if (aogConfig.enableToolLift == 1)
  {
    if (bitState  && !bitStateOld) pressCSM1(); //Press Go button - CAN Page
    if (!bitState && bitStateOld)  pressCSM2(); //Press End button - CAN Page
  }

  bitStateOld = bitState;
}
