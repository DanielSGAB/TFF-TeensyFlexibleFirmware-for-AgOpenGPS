/*
   UDP Autosteer code for Teensy 4.1
   For AgOpenGPS
   01 Feb 2022
   Like all Arduino code - copied from somewhere else :)
   So don't claim it as your own
*/

/// <summary>
/// The core steering control loop — despite the file/function name,
/// this is PROPORTIONAL-ONLY (the comment inside says so explicitly;
/// no integral or derivative term at all), computing pwmDrive from
/// steerAngleError * Kp. Three refinements on top of the raw
/// proportional output:
///   1. Min-PWM compensation: adds/subtracts steerSettings.minPWM
///      unconditionally whenever driving at all, so the motor
///      actually starts moving immediately rather than stalling
///      against its own static friction at very small PWM values.
///   2. Low/high PWM interpolation: below LOW_HIGH_DEGREES of error,
///      the maximum allowed PWM ramps linearly from lowPWM up toward
///      highPWM (via the highLowPerDeg precomputed in
///      steerSettingsInit()) — full highPWM is only reached once the
///      error is large enough, avoiding a harsh, maxed-out correction
///      for tiny steering errors.
///   3. Danfoss proportional-valve remapping: Danfoss valves expect a
///      PWM duty cycle centred at 50% (128) rather than a signed
///      value around zero — pwmDrive is rescaled into the
///      [65...190] range this specific valve type needs, so
///      motorDrive() (further down this file) never needs its own
///      Danfoss-specific branch.
/// </summary>
void calcSteeringPID(void)
{
    //Proportional only
    pValue = steerSettings.Kp * steerAngleError;
    pwmDrive = (int16_t)pValue;

    errorAbs = abs(steerAngleError);
    int16_t newMax = 0;

    if (errorAbs < LOW_HIGH_DEGREES)
    {
        newMax = (errorAbs * highLowPerDeg) + steerSettings.lowPWM;
    }
    else newMax = steerSettings.highPWM;

    //add min throttle factor so no delay from motor resistance.
    if (pwmDrive < 0) pwmDrive -= steerSettings.minPWM;
    else if (pwmDrive > 0) pwmDrive += steerSettings.minPWM;

    //Serial.print(newMax); //The actual steering angle in degrees
    //Serial.print(",");

    //limit the pwm drive
    if (pwmDrive > newMax) pwmDrive = newMax;
    if (pwmDrive < -newMax) pwmDrive = -newMax;

    if (steerConfig.MotorDriveDirection) pwmDrive *= -1;

    if (steerConfig.IsDanfoss)
    {
        // Danfoss: PWM 25% On = Left Position max  (below Valve=Center)
        // Danfoss: PWM 50% On = Center Position
        // Danfoss: PWM 75% On = Right Position max (above Valve=Center)
        pwmDrive = (constrain(pwmDrive, -250, 250));

        // Calculations below make sure pwmDrive values are between 65 and 190
        // This means they are always positive, so in motorDrive, no need to check for
        // steerConfig.isDanfoss anymore
        pwmDrive = pwmDrive >> 2; // Devide by 4
        pwmDrive += 128;          // add Center Pos.

        // pwmDrive now lies in the range [65 ... 190], which would be great for an ideal opamp
        // However the TLC081IP is not ideal. Approximating from fig 4, 5 TI datasheet, @Vdd=12v, T=@40Celcius, 0 current
        // Voh=11.08 volts, Vol=0.185v
        // (11.08/12)*255=235.45
        // (0.185/12)*255=3.93
        // output now lies in the range [67 ... 205], the center position is now 136
        //pwmDrive = (map(pwmDrive, 4, 235, 0, 255));
    }
}

//#########################################################################################

void motorDrive(void)
{
  // --- CAN steering curve (only computed if a CAN brand is active) ---
  // Uses the signed pwmDrive value from calcSteeringPID() *before* the
  // PWM code below flips its sign for direction handling — this must
  // run first so both output paths see the same original PID output.
  // Brand == BRAND_NONE (8): skipped entirely, zero effect on setCurve,
  // matching the original CAN-less firmware exactly.
  if (Brand != BRAND_NONE)
  {
      if (Brand == 3 || Brand == 5)
          setCurve = setCurve - (pwmDrive * 10);  // Fendt/FendtOne: full 16-bit scale, needs to move faster
      else
          setCurve = (setCurve - pwmDrive);
  }

  // Keya CAN steering motor — checked first, completely bypasses the
  // PWM output paths below (CytronDriver/IBT2) when selected. Unlike
  // those two, which only ever differ in HOW the same pwmDrive value
  // reaches two physical PWM pins, Keya needs an entirely different
  // transport (CAN frames, not PWM pins) — see keyaSendSpeed()
  // (zKeya.ino) for the actual protocol. pwmValue is passed through
  // unmodified (not pre-negated the way the PWM paths below mutate
  // pwmDrive in place) since keyaSendSpeed() does its own scaling and
  // sign handling internally.
  if (MotorDriveType == MOTOR_DRIVE_KEYA)
  {
    keyaSendSpeed(pwmDrive, intendToSteer);
    pwmDisplay = pwmDrive;
    return;
  }

  // Used with Cytron MD30C Driver
  // Steering Motor
  // Dir + PWM Signal
  if (steerConfig.CytronDriver)
  {
    // Cytron MD30C Driver Dir + PWM Signal
    if (pwmDrive >= 0)
    {
      bitSet(PORTD, 4);  //set the correct direction
    }
    else
    {
      bitClear(PORTD, 4);
      pwmDrive = -1 * pwmDrive;
    }

    //write out the 0 to 255 value
    analogWrite(PWM1_LPWM, pwmDrive);
    pwmDisplay = pwmDrive;
  }
  else
  {
    // IBT 2 Driver Dir1 connected to BOTH enables
    // PWM Left + PWM Right Signal

    if (pwmDrive > 0)
    {
      analogWrite(PWM2_RPWM, 0);//Turn off before other one on
      analogWrite(PWM1_LPWM, pwmDrive);
    }
    else
    {
      pwmDrive = -1 * pwmDrive;
      analogWrite(PWM1_LPWM, 0);//Turn off before other one on
      analogWrite(PWM2_RPWM, pwmDrive);
    }

    pwmDisplay = pwmDrive;
  }
}
