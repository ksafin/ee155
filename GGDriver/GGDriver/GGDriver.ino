#include <Servo.h>
#include <IntervalTimer.h>
#include <Metro.h>
#include <Adafruit_MCP23017.h>
#include <Encoder.h>

// ======================== SPI BRIDGE DEFINITION  ======================== 
#define MASTER_SERIAL Serial1
#define IRQ 26
#define CTS 27
#define RTS 28
#define SPI_RESET 29

// BIT MASKS
// Masks for extracting relevant data
#define componentId_mask 0b00000011
#define functionId_mask 0b11111100
#define parameterQty_mask 0b00000011

// BUFFER
uint8_t rxbuff[1000];
uint8_t nparams;
byte header;
uint8_t buffw = 0;
uint8_t buffr = 0;
boolean hasPacket = false;
uint8_t fid;
uint8_t cid;
uint8_t params[30];

typedef union
{
 float val;
 uint8_t bytes[4];
} FLOATUNION;

typedef union
{
 short val;
 uint8_t bytes[2];
} SHORTUNION;

typedef union
{
 uint32_t val;
 uint8_t bytes[4];
} INTUNION;

FLOATUNION fu;
SHORTUNION su;
INTUNION iu;

// FUNCTION ID DEFINITIONS
#define FID_SETPWM    0x00  // Set Motor PWM
#define FID_SETILIM   0x01  // Set Motor Current Limit
#define FID_GETSPD    0x02  // Get Motor Speed (RPM)
#define FID_PIDEN     0x03  // Enable PID 
#define FID_GETCUR    0x04  // Get Motor Current
#define FID_SETPID    0x05  // Set PID Constants
#define FID_SETSPD    0x06  // Set Motor Speed (RPM)
#define FID_RESENC    0x07  // Reset Motor Encoder
#define FID_GETENC    0x08  // Get Motor Encoder Ticks
#define FID_ROTPWM    0x09  // Rotate for some Revs at given PWM
#define FID_ROTSPD    0x0A  // Rotate for some revs at given RPM
#define FID_COAST     0x0B  // Enable Coast
#define FID_BRAKE     0x0C  // Enable Brake
#define FID_CSPD      0x0D  // Set Coast Speed
#define FID_STOP      0x0E  // Stop Motor
#define FID_REL       0x0F  // Release Motor
#define FID_FAULT     0x10  // Get Motor Fault
#define FID_SETANG    0x11  // Set Servo Angle
#define FID_SETSSP    0x12  // Set Continuous Servo Speed
#define FID_TCURR     0x13  // Get Total Current
#define FID_STEPILIM  0x14  // Set Stepper Current Limit
#define FID_STEPCUR   0x15  // Get Stepper Current
#define FID_RESSTEP   0x16  // Reset Stepper Steps
#define FID_GETSTP    0x17  // Get Stepper Steps
#define FID_STEP      0x18  // Advanced Stepper for Steps
#define FID_HOLD      0x19  // Hold Stepper
#define FID_STEPSPD   0x1A  // Set Stepper Speed, in RPM
#define FID_ONESTP    0x1B  // Advance One Step
#define FID_STEPREL   0x1C  // Release Stepper
#define FID_SETPPR    0x1D  // Set PPR for motor


// ======================== LED DEFINITION  ======================== 
// Pins for RGB LED cathodes -- drive LOW to enable particular color
#define LED_R 2
#define LED_G 7
#define LED_B 8

// ======================== MOTOR DEFINITION  ======================== 

// MOTOR PWM PINS
// Drive these pins with PWM to enable motor at given Duty Cycle
#define MOTOR1_PWM 3
#define MOTOR2_PWM 4
#define MOTOR3_PWM 5
#define MOTOR4_PWM 6

// MOTOR GPIO PINS
// Pins to control motor drivers
// IN_A/B pins determine CW or CCW direction
// EN_A/B pins determine fault state and enable half bridges (when HIGH)
Adafruit_MCP23017 mcp;
#define IN3_A 0
#define EN3_A 1
#define EN3_B 2
#define IN3_B 3
#define IN4_B 4
#define EN4_B 5
#define EN4_A 6
#define IN4_A 7
#define IN2_B 8
#define EN2_B 9
#define EN2_A 10
#define IN2_A 11
#define IN1_A 12
#define EN1_A 13
#define EN1_B 14
#define IN1_B 15

// ENCODER PINS
// Pins for encoder A&B channels for each motor.
#define ENC1_A 14
#define ENC1_B 23
#define ENC2_A 20
#define ENC2_B 21
#define ENC3_A 16
#define ENC3_B 17
#define ENC4_A 22
#define ENC4_B 15

// CURRENT SENSE PINS
// Analog pins on which to measure current foe a given motor
// Produced at 140 mV/A
#define ISENSE1 A10
#define ISENSE2 A11
#define ISENSE3 A12
#define ISENSE4 A13

// ======================== SERVO DEFINITION  ======================== 
// PWM for Servos to control angle
#define SERVO1_PWM 9  
#define SERVO2_PWM 10
#define SERVO3_PWM 25
#define SERVO4_PWM 32

// Pin to enable logic shifter for 3.3V-5V servo logic
#define LS_ENABLE 11

// ======================== AUXILIARY DEFINITION  ======================== 
// Analog pin to read battery voltage
#define BATT_SENSE A14

// ======================== MIXED-USE CONSTANTS  ======================== 
// Easy-use constants for in-code clarity to refer to motors
#define MOTOR1 1
#define MOTOR2 2
#define MOTOR3 3
#define MOTOR4 4

// Easy-use constants for in-code clarity to refer to sevos
#define SERVO1 1
#define SERVO2 2
#define SERVO3 3
#define SERVO4 4

// 140 mV per A on current sense. Divide analog reading by this to get current
#define MV_PER_A 0.140

// ======================== MOTOR STATE VARIABLES  ======================== 

// PWM STATES
// Value from 0 - 2^PWM_RES to determine Motor PWM duty cycle
volatile uint16_t motor1_pwm = 0;
volatile uint16_t motor2_pwm = 0;
volatile uint16_t motor3_pwm = 0;
volatile uint16_t motor4_pwm = 0;

// CURRENT DRAW
// Current through motor, calculated, from 0 - 30A
double motor1_curr = 0;
double motor2_curr = 0;
double motor3_curr = 0;
double motor4_curr = 0;

// OVERCURRENT
// State of overcurrent fault for each motor (false = no fault)
boolean motor1_oc = false;
boolean motor2_oc = false;
boolean motor3_oc = false;
boolean motor4_oc = false;

// CURRENT LIMITS
// Current limit for each motor, set in A.
// Triggers OC fault when passed.
double motor1_ilim = 30;
double motor2_ilim = 30;
double motor3_ilim = 30;
double motor4_ilim = 30;

// RPM STATES
// Most recently calculated RPMs for each motor.
volatile double motor1_rpm = 0;
volatile double motor2_rpm = 0;
volatile double motor3_rpm = 0;
volatile double motor4_rpm = 0;

// COAST SPEEDS (ms to OFF)
// Coast speed for each motor
// This is the time, in ms, the motor will take to get from its PWM to 0.
uint16_t motor1_coastSpd = 500;
uint16_t motor2_coastSpd = 500;
uint16_t motor3_coastSpd = 500;
uint16_t motor4_coastSpd = 500;

// PPR VALUES
// Number of encoder pulses per revolution of the motor shaft.
// This is used to determine RPM, and is specific for every motor.
// Default is 134.4, which is the ppr for an Andymark Neverest 19.2:1
float m1_ppr = 7;
float m2_ppr = 7;
float m3_ppr = 7;
float m4_ppr = 7;

// FAULT STATES
// Fault state for each motor, as determined by the bridge chip.
// False for no fault -- fautls are voltage fault & temperature fault.
boolean fault1 = false;
boolean fault2 = false;
boolean fault3 = false;
boolean fault4 = false;

// CLOCKWISE STATES (TRUE = CW, FALSE = CCW)
boolean motor1_cw = true;
boolean motor2_cw = true;
boolean motor3_cw = true;
boolean motor4_cw = true;

// BRAKE STATES (TRUE = BRAKE, FALSE = COAST)
boolean motor1_brake = true;
boolean motor2_brake = true;
boolean motor3_brake = true;
boolean motor4_brake = true;

// GLOBAL PID SETTINGS
// Timestep between PID calculations, in ms.
uint16_t timeStep = 20;

// PID VARIABLES FOR EACH MOTOR
// Whether to use PID. If true, enables setting RPM via PID.
// If false, motors are driven just by PWM. 
boolean m1_usePID = true;
boolean m2_usePID = false;
boolean m3_usePID = false;
boolean m4_usePID = false;

// Desired RPM to reach, under PID conditions.
double m1_targetRPM = 0.0;
double m2_targetRPM = 0.0;
double m3_targetRPM = 0.0;
double m4_targetRPM = 0.0;

// Current PID error. (distance from desired RPM, at current RPM)
double m1_err = 0.0;
double m2_err = 0.0;
double m3_err = 0.0;
double m4_err = 0.0;

// Proportionality constants for each motors PID
double m1_Kp = 0.0;
double m2_Kp = 0.0;
double m3_Kp = 0.0;
double m4_Kp = 0.0;

// Derivative constants for each motors PID
double m1_Kd = 0.0;
double m2_Kd = 0.0;
double m3_Kd = 0.0;
double m4_Kd = 0.0;

// Integral constants for each motors PID
double m1_Ki = 0.0;
double m2_Ki = 0.0;
double m3_Ki = 0.0;
double m4_Ki = 0.0;

// Current accrued integral value for each motors PID
double m1_integral = 0.0;
double m2_integral = 0.0;
double m3_integral = 0.0;
double m4_integral = 0.0;

// Current accrued derivative value for each motors PID
double m1_derivative = 0.0;
double m2_derivative = 0.0;
double m3_derivative = 0.0;
double m4_derivative = 0.0;

// The error in the last cycle, to calculate derivative value
double m1_errLast = 0.0;
double m2_errLast = 0.0;
double m3_errLast = 0.0;
double m4_errLast = 0.0;

// Ideal double-based PWM values for calculations
double m1_optPWM = 0.0;
double m2_optPWM = 0.0;
double m3_optPWM = 0.0;
double m4_optPWM = 0.0;


// MOTOR PINOUT EXPLANATION
// EN_A/DIAG_A -- OUTPUT PIN. WHEN LOW, DISABLES HALF-BRIDGE A. Pulled LOW by fault.
// EN_B/DIAG_B -- OUTPUT PIN, WHEN LOW, DISABLED HALF-BRIDGE B. Pulled LOW by fault.
// IN_A -- CLOCKWISE INPUT (HIGH = TRUE)
// IN_B -- COUNTER CLOCKWISE INPUT (HIGH = TRUE)
// LOGIC:
// IN_A && IN_B == 1 -> HIGH IMPEDANCE (BREAK TO VCC)
// IN_A == 1, IN_B == 0 -> CLOCKWISE
// IN_A == 0, IN_B == 1 -> COUNTER CLOCKWISE
// IN_A == 0, IN_B == 0 -> HIGH IMPEDANCE (BREAK TO GND)

// ======================== SERVO STATE VARIABLES  ========================
// Servo objects
Servo servo1;
Servo servo2;
Servo servo3;
Servo servo4;

// ======================== SETTING VARIABLES  ========================
// DIVIDE BY THIS TO GET BATTERY VOLTAGE
const double BATT_V_MULTIPLIER = 0.2115384615;

// BATTERY VOLTAGE, in volts
double battVoltage;

// PWM FREQUENCY & RESOLUTION
const double PWM_FREQ = 11718.75; // in hertz
const uint8_t PWM_RES = 8; // 2^PWM_RES is max PWM write value

// ANALOG READING
const uint8_t ADC_RES = 12; // 2^ADC_RES; 4096, max analog value
const uint8_t ADC_AVG = 16; // ADC AVERAGING -- number of samples to avg

// Encoder objects
Encoder enc1(ENC1_A, ENC1_B);
Encoder enc2(ENC2_A, ENC2_B);
Encoder enc3(ENC3_A, ENC3_B);
Encoder enc4(ENC4_A, ENC4_B);

// The number of encoder ticks 100ms ago (for RPM calculation)
long m1_past_ticks = 0;
long m2_past_ticks = 0;
long m3_past_ticks = 0;
long m4_past_ticks = 0;

// Time between encoder readings for RPM updates
uint16_t rpmTimeStep = 100; // in ms

// Total current of all four motors
double totalCurrent;

// Checks amount of time remaining to coast
IntervalTimer m1_timer;
IntervalTimer m2_timer;
IntervalTimer m3_timer;
IntervalTimer m4_timer;

// Timer for rotating for a fixed number of rotations
IntervalTimer rotTimer;

// Timer for rpm measurements
Metro encTimer = Metro(rpmTimeStep);

// Timer for PID iteration (ms)
Metro pidTimer = Metro(timeStep);

// Settings for reaction to current limits being hit
long waitTime_oc = 500000; // microseconds; 500 ms wait time after overcurrent.
long waitTime_ot = 500000; // microseconds; 500 ms wait time after overtemp.

// ======================== END OF VARIABLES  ========================

void setup() {
  // INITIALIZE PWM for each motor
  analogWriteResolution(PWM_RES);
  analogWriteFrequency(MOTOR1_PWM, PWM_FREQ);
  analogWriteFrequency(MOTOR2_PWM, PWM_FREQ);
  analogWriteFrequency(MOTOR3_PWM, PWM_FREQ);
  analogWriteFrequency(MOTOR4_PWM, PWM_FREQ);

  // INITIALIZE ADC
  analogReadResolution(ADC_RES);
  analogReadAveraging(ADC_AVG);

  // INITIALIZE ALL PINS
  // LED -- ACTIVE LOW (DISABLED)
  pinMode(LED_R, OUTPUT);
  pinMode(LED_G, OUTPUT);
  pinMode(LED_B, OUTPUT);
  digitalWrite(LED_R, HIGH);
  digitalWrite(LED_G, HIGH);
  digitalWrite(LED_B, HIGH);

  // MOTOR PINS
  // Motors disabled by PWM pulldown
  pinMode(MOTOR1_PWM, OUTPUT);
  pinMode(MOTOR2_PWM, OUTPUT);
  pinMode(MOTOR3_PWM, OUTPUT);
  pinMode(MOTOR4_PWM, OUTPUT);
  pinMode(ISENSE1, INPUT);
  pinMode(ISENSE2, INPUT);
  pinMode(ISENSE3, INPUT);
  pinMode(ISENSE4, INPUT);

  // Enable Multiplexer & configure
  mcp.begin();
  mcp.pinMode(IN1_A, OUTPUT);
  mcp.pinMode(IN1_B, OUTPUT);
  mcp.pinMode(IN2_A, OUTPUT);
  mcp.pinMode(IN2_B, OUTPUT);
  mcp.pinMode(IN3_A, OUTPUT);
  mcp.pinMode(IN3_B, OUTPUT);
  mcp.pinMode(IN4_A, OUTPUT);
  mcp.pinMode(IN4_B, OUTPUT);
  mcp.pinMode(EN1_A, INPUT);
  mcp.pinMode(EN1_B, INPUT);
  mcp.pinMode(EN2_A, INPUT);
  mcp.pinMode(EN2_B, INPUT);
  mcp.pinMode(EN3_A, INPUT);
  mcp.pinMode(EN3_B, INPUT);
  mcp.pinMode(EN4_A, INPUT);
  mcp.pinMode(EN4_B, INPUT);

  // Disable all bridges by default
  mcp.digitalWrite(IN1_A, LOW);
  mcp.digitalWrite(IN1_B, LOW);
  mcp.digitalWrite(IN2_A, LOW);
  mcp.digitalWrite(IN2_B, LOW);
  mcp.digitalWrite(IN3_A, LOW);
  mcp.digitalWrite(IN3_B, LOW);
  mcp.digitalWrite(IN4_A, LOW);
  mcp.digitalWrite(IN4_B, LOW);

  // INITIALIZE SERVOS
  servo1.attach(SERVO1_PWM);
  servo2.attach(SERVO2_PWM);
  servo3.attach(SERVO3_PWM);
  servo4.attach(SERVO4_PWM);
  pinMode(LS_ENABLE, OUTPUT);
  digitalWrite(LS_ENABLE, HIGH); // enable logic shifter

  // SPI BRIDGE
  MASTER_SERIAL.begin(38400);

  // AUX
  pinMode(BATT_SENSE, INPUT);

  // Read current encoder readings, to start off.
  m1_past_ticks = enc1.read();
  m2_past_ticks = enc2.read();
  m3_past_ticks = enc3.read();
  m4_past_ticks = enc4.read();

  Serial.begin(115200);
}

// ======================== READ CURRENTS ======================== 
// Measure ADC value, divide by max ADC value, multiply by 3.3 to get voltage.
// Divide by MV_PER_A divider to determine current, in Amps.
// Add all currents to get total motor current
void readCurrents() {
  motor1_curr = ((analogRead(ISENSE1)/((float)(pow(2,ADC_RES)))) * 3.3) / MV_PER_A;
  motor2_curr = ((analogRead(ISENSE2)/((float)(pow(2,ADC_RES)))) * 3.3) / MV_PER_A;
  motor3_curr = ((analogRead(ISENSE3)/((float)(pow(2,ADC_RES)))) * 3.3) / MV_PER_A;
  motor4_curr = ((analogRead(ISENSE4)/((float)(pow(2,ADC_RES)))) * 3.3) / MV_PER_A;
  totalCurrent = motor1_curr + motor2_curr + motor3_curr + motor4_curr;
}

// ======================== CHECK FOR FAULTS  ======================== 
// Reads diagnostic pins for each half-bridge to determine fault states
// If either shows an error, assert fault for entire motor chip
void readFaultStates() {
  fault1 = (!mcp.digitalRead(EN1_A)) || (!mcp.digitalRead(EN1_B));
  fault2 = (!mcp.digitalRead(EN2_A)) || (!mcp.digitalRead(EN2_B));
  fault3 = (!mcp.digitalRead(EN2_A)) || (!mcp.digitalRead(EN2_B));
  fault4 = (!mcp.digitalRead(EN2_A)) || (!mcp.digitalRead(EN2_B));
}

// ======================== GET MOTOR FAULT  ======================== 
// Returns motor fault state, 0 for none, 1 for OC, 2 for all others, 3 for both
//motor -- an int, 1-4, designating which motor to control
uint8_t getMotorFault(uint8_t motor) {
  switch(motor) {
    case 1: if(motor1_oc && fault1) { return 3; } else if(fault1) {return 2;} else if(motor1_oc) { return 1; } else { return 0; }
    case 2: if(motor2_oc && fault2) { return 3; } else if(fault2) {return 2;} else if(motor2_oc) { return 1; } else { return 0; }
    case 3: if(motor3_oc && fault3) { return 3; } else if(fault3) {return 2;} else if(motor3_oc) { return 1; } else { return 0; }
    case 4: if(motor4_oc && fault4) { return 3; } else if(fault4) {return 2;} else if(motor4_oc) { return 1; } else { return 0; }
  }
}

// ======================== SET MOTOR PWM  ======================== 
// Set the PWM duty cycle the provided motor
// motor -- an int, 1-4, designating which motor to control
// pwm -- an int, 0 - 2^PWM_RES, designating duty cycle
void setMotorPWM(uint8_t motor, uint16_t pwm) {
  switch(motor) {
    case 1: setMotorDirection(MOTOR1, motor1_cw); motor1_pwm = pwm; analogWrite(MOTOR1_PWM, pwm); break;
    case 2: setMotorDirection(MOTOR2, motor2_cw); motor2_pwm = pwm; analogWrite(MOTOR2_PWM, pwm); break;
    case 3: setMotorDirection(MOTOR3, motor3_cw); motor3_pwm = pwm; analogWrite(MOTOR3_PWM, pwm); break;
    case 4: setMotorDirection(MOTOR4, motor4_cw); motor4_pwm = pwm; analogWrite(MOTOR4_PWM, pwm); break;
  }
}

// ======================== SET MOTOR COAST SPEED  ======================== 
// Set the coast speed of a given motor, in ms
// motor -- an int, 1-4, designating which motor to control
// speed -- time, in ms, to take for a coast from any PWM to 0.
void setMotorCoastSpeed(uint8_t motor, uint16_t spd) {
  switch(motor) {
    case 1: motor1_coastSpd = spd; break;
    case 2: motor1_coastSpd = spd; break;
    case 3: motor1_coastSpd = spd; break;
    case 4: motor1_coastSpd = spd; break;
  }
}

// ======================== SET MOTOR CURRENT LIMIT  ======================== 
// Set the current limit for the provided motor to ilim
// motor -- an int, 1-4, designating which motor to control
// ilim -- a double, 0 - 30.0, designating current limit for a motor
void setMotorCurrentLimit(uint8_t motor, double ilim) {
  switch(motor) {
    case 1: motor1_ilim = ilim; break;
    case 2: motor2_ilim = ilim; break;
    case 3: motor3_ilim = ilim; break;
    case 4: motor4_ilim = ilim; break;
  }
}

// ======================== SET MOTOR DIRECTION  ======================== 
// Sets the direction of a given motor
// motor -- an int, 1-4, designating which motor to control
// cw -- a boolean designating clockwise direction (true = cw, false = ccw)
// Sets IN1_A to true and IN1_B to false for cw, reverse for ccw
void setMotorDirection(uint8_t motor, boolean cw) {
  switch(motor) {
    case 1: motor1_cw = cw;
            mcp.digitalWrite(IN1_A, cw);
            mcp.digitalWrite(IN1_B, !cw);
            break;
    case 2: motor2_cw = cw;
            mcp.digitalWrite(IN2_A, cw);
            mcp.digitalWrite(IN2_B, !cw);
            break;
    case 3: motor3_cw = cw;
            mcp.digitalWrite(IN3_A, cw);
            mcp.digitalWrite(IN3_B, !cw);
            break;
    case 4: motor4_cw = cw;
            mcp.digitalWrite(IN4_A, cw);
            mcp.digitalWrite(IN4_B, !cw);
            break;
  }
}

// ======================== GET MOTOR DIRECTION  ======================== 
// Gets the current motor direction, as a boolean.
// motor -- an int, 1-4, designating which motor to check
// Returned value is true for cw, false for ccw
boolean getMotorDirection(uint8_t motor) {
  switch(motor) {
    case 1: return motor1_cw;
    case 2: return motor2_cw;
    case 3: return motor3_cw;
    case 4: return motor4_cw;
    default: return false;
  }
}

// ======================== GET MOTOR RPM ======================== 
// Gets the current motor speed, in rpm.
// motor -- an int, 1-4, designating which motor to check
// Returned value is rpm
double getMotorRPM(uint8_t motor) {
  switch(motor) {
    case 1: return motor1_rpm;
    case 2: return motor2_rpm;
    case 3: return motor3_rpm;
    case 4: return motor4_rpm;
    default: return 0;
  }
}

// ======================== SET MOTOR PPR ======================== 
// Sets the ppr of a motor.
// motor -- an int, 1-4, designating which motor to check
// ppr -- a double, designating ppr. Must be positive.
void setMotorPPR(uint8_t motor, double ppr) {
  switch(motor) {
    case 1: m1_ppr = ppr; break;
    case 2: m2_ppr = ppr; break;
    case 3: m3_ppr = ppr; break;
    case 4: m4_ppr = ppr; break;
  }
}

// ======================== GET MOTOR CURRENT DRAW  ======================== 
// Returns current draw for a particular motor
// motor -- an int, 1-4, designating which motor to check
// Returned value is a double, from 0 - 30.0 A indicating motor current draw
double getMotorCurrent(uint8_t motor) {
  switch(motor) {
    case 1: return motor1_curr; 
    case 2: return motor2_curr;
    case 3: return motor3_curr;
    case 4: return motor4_curr;
    default: return 0;
  }
}

// ======================== ENABLE OR DISABLE PID  ======================== 
// Enables or disables PID for a given motor
// enPID -- a boolean, true to enable PID, false to disable
void enablePID(uint8_t motor, boolean enPID) {
  switch(motor) {
    case 1: m1_usePID = enPID; break;
    case 2: m2_usePID = enPID; break;
    case 3: m3_usePID = enPID; break;
    case 4: m4_usePID = enPID; break;
  }
}

// ======================== RETURN ENCODER TICKS  ======================== 
// Returns the current encoder ticks
// motor -- an int, 1-4, designating which motor to check
// Returns a long, indicating the number of motor ticks
// Note: number of ticks / ppr gives # of rotations since reset
long getEncoderTicks(uint8_t motor) {
  switch(motor) {
    case 1: return enc1.read();
    case 2: return enc2.read();
    case 3: return enc3.read();
    case 4: return enc4.read();
    default: return 0;
  }
}

// ======================== RESET ENCODER TO 0  ======================== 
// Resets a given motors encoder value to 0
// motor -- an int, 1-4, designating which motor to control
void resetEncoder(uint8_t motor) {
  switch(motor) {
    case 1: enc1.write(0);
    case 2: enc2.write(0);
    case 3: enc3.write(0);
    case 4: enc4.write(0);
  }
}

// ======================== SET BRAKE SETTING  ======================== 
// Sets the brake setting for a motor (true for brake, false for coast)
// motor -- an int, 1-4, designating which motor to control
// isBrake -- true for brake, false for coast
void useBrakeMotor(uint8_t motor, boolean isBrake) {
  switch(motor) {
    case 1: motor1_brake = isBrake; break;
    case 2: motor1_brake = isBrake; break;
    case 3: motor1_brake = isBrake; break;
    case 4: motor1_brake = isBrake; break;
  }
}

// ======================== BRAKE MOTOR TO VCC  ======================== 
// Brakes a motor to VCC (sets both motor voltages to VCC)
// motor -- an int, 1-4, designating which motor to control
// Sets both IN pins to high to set motors to VCC - VCC
void brakeMotor(uint8_t motor) {
  switch(motor) {
    case 1: mcp.digitalWrite(IN1_A, HIGH);
            mcp.digitalWrite(IN1_B, HIGH);
            setMotorPWM(MOTOR1, 0);
            m1_targetRPM = 0;
            break;
    case 2: mcp.digitalWrite(IN2_A, HIGH);
            mcp.digitalWrite(IN2_B, HIGH);
            setMotorPWM(MOTOR2, 0);
            m2_targetRPM = 0;
            break;
    case 3: mcp.digitalWrite(IN3_A, HIGH);
            mcp.digitalWrite(IN3_B, HIGH);
            setMotorPWM(MOTOR3, 0);
            m3_targetRPM = 0;
            break;
    case 4: mcp.digitalWrite(IN4_A, HIGH);
            mcp.digitalWrite(IN4_B, HIGH);
            setMotorPWM(MOTOR4, 0);
            m4_targetRPM = 0;
            break;
  }
}

// ======================== CLEAR MOTOR FAULT  ======================== 
// Clears the fault for a motor, if any is present, by pulling the EN pins high
// motor -- an int, 1-4, designating which motor to control
void clearFault(uint8_t motor) {
  switch(motor) {
    case 1: mcp.digitalWrite(EN1_A, HIGH);
            mcp.digitalWrite(EN1_B, HIGH);
            break;
    case 2: mcp.digitalWrite(EN2_A, HIGH);
            mcp.digitalWrite(EN2_B, HIGH);
            break;
    case 3: mcp.digitalWrite(EN3_A, HIGH);
            mcp.digitalWrite(EN3_B, HIGH);
            break;
    case 4: mcp.digitalWrite(EN4_A, HIGH);
            mcp.digitalWrite(EN4_B, HIGH);
            break;
  }
}

// ======================== BRAKE MOTOR TO GND  ======================== 
// Releases a motor by setting the IN pins both to LOW
// motor -- an int, 1-4, designating which motor to control
void releaseMotor(uint8_t motor) {
  switch(motor) {
    case 1: mcp.digitalWrite(IN1_A, LOW);
            mcp.digitalWrite(IN1_B, LOW);
            break;
    case 2: mcp.digitalWrite(IN2_A, LOW);
            mcp.digitalWrite(IN2_B, LOW);
            break;
    case 3: mcp.digitalWrite(IN3_A, LOW);
            mcp.digitalWrite(IN3_B, LOW);
            break;
    case 4: mcp.digitalWrite(IN4_A, LOW);
            mcp.digitalWrite(IN4_B, LOW);
            break;
  }
}

// ======================== COAST A MOTOR  ======================== 
// Takes a motor and coasts it from its current PWM to a duty cycle of zero
// Sets an interrupt timer for every X microseconds by dividing the coast speed by the current PWM
// Each interrupt, subtracts 1 from the PWM DC until it reqaches zero, then ends the interrupt routine.
// motor -- an int, 1-4, designating which motor to control
void coastMotor(uint8_t motor) {
  switch(motor) {
    case 1: m1_timer.begin(coastMotor1,round((((float)motor1_coastSpd) * 1000) / motor1_pwm)); break;
    case 2: m2_timer.begin(coastMotor2,round((((float)motor2_coastSpd) * 1000) / motor2_pwm)); break;
    case 3: m3_timer.begin(coastMotor3,round((((float)motor3_coastSpd) * 1000) / motor3_pwm)); break;
    case 4: m4_timer.begin(coastMotor4,round((((float)motor4_coastSpd) * 1000) / motor4_pwm)); break;
  }
}

// ======================== STOP MOTOR BRAKE & COAST  ======================== 
// Stops a motor, according to its brake or coast setting
/// motor -- an int, 1-4, designating which motor to control
void stopMotor(uint8_t motor) {
   switch(motor) {
    case 1: if(motor1_brake) { brakeMotor(motor); }
            else { coastMotor(motor); } break;
    case 2: if(motor2_brake) { brakeMotor(motor); }
            else { coastMotor(motor); } break;
    case 3: if(motor3_brake) { brakeMotor(motor); }
            else { coastMotor(motor); } break;
    case 4: if(motor4_brake) { brakeMotor(motor); }
            else { coastMotor(motor); } break;
  }
}

// ======================== UPDATE RPMS FOR ALL MOTORS  ======================== 
// Updates RPM readings for all four motors, as per the encoder-reading timer
// Adjust the time between encoder readings by changing the constructor for encTimer
// Checks difference between current encoder ticks and the last set of encoder ticks
// Multiplies this by the # of timesteps in 1 second to get rps, multiplies by 60 to get rpm
// Reads the current values for subsequent updates.
void updateRPMs() {
  if(encTimer.check() == 1) {
    motor1_rpm = abs((((double)absoluteDifference(enc1.read(),m1_past_ticks)) / m1_ppr)  * (1000.0/rpmTimeStep) * 60.0 / 4.0);  
    motor2_rpm = abs((((double)absoluteDifference(enc2.read(),m2_past_ticks)) / m2_ppr)  * (1000.0/rpmTimeStep) * 60.0 / 4.0);
    motor3_rpm = abs((((double)absoluteDifference(enc3.read(),m3_past_ticks)) / m3_ppr)  * (1000.0/rpmTimeStep) * 60.0 / 4.0);
    motor4_rpm = abs((((double)absoluteDifference(enc4.read(),m4_past_ticks)) / m4_ppr)  * (1000.0/rpmTimeStep) * 60.0 / 4.0);
    m1_past_ticks = enc1.read();
    m2_past_ticks = enc2.read();
    m3_past_ticks = enc3.read();
    m4_past_ticks = enc4.read();
  }
}

// ======================== MEASURE ENCODER CHANGE  ======================== 
// Measures the absolute difference between two values.
long absoluteDifference(long val1, long val2) {
  if(val1 > val2) return val1-val2;
  return val2 - val1;
}

// ======================== INTERRUPTS FOR COASTING ======================== 
// These interrupt functions are called every given number of microseconds.
// Each call, the PWM gets subtracted by 1 until the PWM is 0, then the interrupt is cancelled.
void coastMotor1() {
  setMotorPWM(MOTOR1, motor1_pwm - 1);
  if(motor1_pwm == 0) m1_timer.end();
}

void coastMotor2() {
  setMotorPWM(MOTOR2, motor2_pwm - 1);
  if(motor2_pwm == 0) m2_timer.end();
}

void coastMotor3() {
  setMotorPWM(MOTOR3, motor3_pwm - 1);
  if(motor3_pwm == 0) m3_timer.end();
}

void coastMotor4() {
  setMotorPWM(MOTOR4, motor4_pwm - 1);
  if(motor4_pwm == 0) m4_timer.end();
}

// ======================== INTERRUPTS FOR OVERCURRENT  ======================== 
// These interrupts are called once after a delay, to clear an OC fault and enable the motor.
void m1_overcurrent() {
  motor1_oc = false;
  setMotorDirection(MOTOR1, motor1_cw); 
}

void m2_overcurrent() {
  motor2_oc = false;
  setMotorDirection(MOTOR2, motor2_cw); 
}

void m3_overcurrent() {
  motor3_oc = false;
  setMotorDirection(MOTOR3, motor3_cw); 
}

void m4_overcurrent() {
  motor4_oc = false;
  setMotorDirection(MOTOR4, motor4_cw); 
}

// ======================== FAULT INTERRUPT FUNCTIONS  ======================== 
// These functions are called when there is a fault for a given motor
// The fault is internally cleared, the motor is re-enabled to 0 PWM.
// The fault is then cleared by the clearFault() function
void m1_fault() {
  fault1 = false;
  motor1_pwm = 0;
  clearFault(MOTOR1);
  setMotorPWM(MOTOR1, 0);
  setMotorDirection(MOTOR1, motor1_cw); 
}

void m2_fault() {
  fault2 = false;
  motor2_pwm = 0;
  clearFault(MOTOR2);
  setMotorPWM(MOTOR2, 0);
  setMotorDirection(MOTOR2, motor2_cw); 
}

void m3_fault() {
  fault3 = false;
  motor3_pwm = 0;
  clearFault(MOTOR3);
  setMotorPWM(MOTOR3, 0);
  setMotorDirection(MOTOR3, motor3_cw); 
}

void m4_fault() {
  fault4 = false;
  motor4_pwm = 0;
  clearFault(MOTOR4);
  setMotorPWM(MOTOR4, 0);
  setMotorDirection(MOTOR4, motor4_cw); 
}

// ======================== CHECK FOR OVERCURRENT FUNCTION  ======================== 
// The currents are checked for each of the motors, and it is asserted if a given
// motor is overcurrent. If so, the flag is set to true, the motor is braked, and
// The interrupt to clear the overcurrent fault is set for a given delay.
void checkCurrents() {
  if(motor1_curr > motor1_ilim) {
    motor1_oc = true;
    releaseMotor(MOTOR1);
    m1_timer.begin(m1_overcurrent, waitTime_oc);
  }
  
  if(motor2_curr > motor2_ilim) {
    motor2_oc = true;
    releaseMotor(MOTOR2);
    m2_timer.begin(m2_overcurrent, waitTime_oc);
  }
  
  if(motor3_curr > motor3_ilim) {
    motor3_oc = true;
    releaseMotor(MOTOR3);
    m3_timer.begin(m3_overcurrent, waitTime_oc);
  }
  
  if(motor4_curr > motor4_ilim) {
    motor4_oc = true;
    releaseMotor(MOTOR4);
    m4_timer.begin(m4_overcurrent, waitTime_oc);
  }
}

// ======================== FAULT FIXING FUNCTION  ======================== 
// This function is called to enable interrupts that clear a given fault, after a delay.
void fixFaults() {
  if(fault1) m1_timer.begin(m1_fault, waitTime_ot);
  if(fault2) m2_timer.begin(m2_fault, waitTime_ot);
  if(fault3) m3_timer.begin(m3_fault, waitTime_ot);
  if(fault4) m4_timer.begin(m4_fault, waitTime_ot);
}

// ======================== SERVO SET ANGLE  ======================== 
// This function sets the angle for a given servo to a desired angle.
// servo -- an int, 1-4, designating which servo to control
void setServoAngle(uint8_t servo, uint16_t angle) {
  switch(servo) {
    case 1: servo1.write(angle);  break;
    case 2: servo2.write(angle); break;
    case 3: servo3.write(angle); break;
    case 4: servo4.write(angle); break;
  }
}

// Determines the "angle" to set a servo to, depending on the kind of servo it is.
// The servo class is built for 180 degree servos, so this function scales the
// value given to a servo object from 180 to 90, 270, and 360.
/*uint8_t getAngleValue(uint16_t desAngle, uint16_t maxAngle) {
  if(maxAngle == 90) {
    return round(desAngle * 2);
  }
  if((maxAngle == 180) || (maxAngle == 0)) {
    return desAngle;
  }
  if(maxAngle == 270) {
    return round((desAngle * 2) / 3.0);
  }
  if(maxAngle == 360) {
    return round(desAngle / 2.0);
  }
}*/

// ======================== ROTATE FOR LINEAR DISTANCE  ======================== 
// This function rotates for a given linear distance given a wheel circumfrence, at a given PWM
// motor -- an int, 1-4, designating which servo to control
/*void rotateMotorForDistance(uint8_t motor, double cirum, uint8_t distance, uint8_t setPWM) {
  switch(motor) {
    case 1: runMotor1ForRotations(rots, rpmOrPwm, isRPM); break;
    case 2: runMotor2ForRotations(rots, rpmOrPwm, isRPM); break;
    case 3: runMotor3ForRotations(rots, rpmOrPwm, isRPM); break;
    case 4: runMotor4ForRotations(rots, rpmOrPwm, isRPM); break;
  }
}*/

// ======================== ROTATE AT GIVEN SPEED  ======================== 
// This function rotates for a given number of rotations at a given speed
// servo -- an int, 1-4, designating which servo to control
void rotateMotorForRotations(uint8_t motor, double rots, uint16_t rpmOrPwm, boolean isRPM) {
  switch(motor) {
    case 1: runMotor1ForRotations(rots, rpmOrPwm, isRPM); break;
    case 2: runMotor2ForRotations(rots, rpmOrPwm, isRPM); break;
    case 3: runMotor3ForRotations(rots, rpmOrPwm, isRPM); break;
    case 4: runMotor4ForRotations(rots, rpmOrPwm, isRPM); break;
  }
}

// Runs a given motor for a certain number of rotations, either under constant
// PWM or constant RPM. If PID is disabled for this motor, does nothing
// rots -- integer indicating number of rotations
// rpmOrPwm -- value holding either PWM duty cycle or RPM
// isRPM -- if true, value is RPM. if false, it's PWM.
void runMotor1ForRotations(double rots, uint16_t rpmOrPwm, boolean isRPM) {
  if((isRPM) && (!m1_usePID)) return;
  long curTicks = getEncoderTicks(MOTOR1);
  long targetTicks = (long)(curTicks + (rots * m1_ppr * 4));
  if(isRPM) m1_targetRPM = rpmOrPwm;
  if(!isRPM) setMotorPWM(MOTOR1, rpmOrPwm);
  while(curTicks < targetTicks) {
    if(isRPM) runPID();
    curTicks = getEncoderTicks(MOTOR1);
  }
  brakeMotor(MOTOR1);
}

void runMotor2ForRotations(double rots, uint16_t rpmOrPwm, boolean isRPM) {
  if((isRPM) && (!m2_usePID)) return;
  long curTicks = getEncoderTicks(MOTOR2);
  long targetTicks = (long)(curTicks + (rots * m2_ppr * 4));
  if(isRPM) m2_targetRPM = rpmOrPwm;
  if(!isRPM) setMotorPWM(MOTOR2, rpmOrPwm);
  while(curTicks < targetTicks) {
    if(isRPM) runPID();
    curTicks = getEncoderTicks(MOTOR2);
  }
  brakeMotor(MOTOR2);
}

void runMotor3ForRotations(double rots, uint16_t rpmOrPwm, boolean isRPM) {
  if((isRPM) && (!m3_usePID)) return;
  long curTicks = getEncoderTicks(MOTOR3);
  long targetTicks = (long)(curTicks + (rots * m3_ppr * 4));
  if(isRPM) m3_targetRPM = rpmOrPwm;
  if(!isRPM) setMotorPWM(MOTOR3, rpmOrPwm);
  while(curTicks < targetTicks) {
    if(isRPM) runPID();
    curTicks = getEncoderTicks(MOTOR3);
  }
  brakeMotor(MOTOR3);
}

void runMotor4ForRotations(double rots, uint16_t rpmOrPwm, boolean isRPM) {
  if((isRPM) && (!m4_usePID)) return;
  long curTicks = getEncoderTicks(MOTOR4);
  long targetTicks = (long)(curTicks + (rots * m4_ppr * 4));
  if(isRPM) m4_targetRPM = rpmOrPwm;
  if(!isRPM) setMotorPWM(MOTOR4, rpmOrPwm);
  while(curTicks < targetTicks) {
    if(isRPM) runPID();
    curTicks = getEncoderTicks(MOTOR4);
  }
  brakeMotor(MOTOR4);
}


// Runs PID for all motors for which PID is enabled
// Runs every so often depending on the configured PID period.
void runPID() {
  if(pidTimer.check()) {
    if(m1_usePID) runMotor1PID();
    if(m2_usePID) runMotor2PID();
    if(m3_usePID) runMotor3PID();
    if(m4_usePID) runMotor4PID();
  }
}

// Set PID settings for a motor and enable PID
void setPID(uint8_t motor, double Kp, double Kd, double Ki) {
  switch(motor) {
    case 1: m1_Kp = Kp; m1_Kd = Kd; m1_Ki = Ki; m1_usePID = true; break;
    case 2: m2_Kp = Kp; m2_Kd = Kd; m2_Ki = Ki; m2_usePID = true; break;
    case 3: m3_Kp = Kp; m3_Kd = Kd; m3_Ki = Ki; m3_usePID = true; break;
    case 4: m4_Kp = Kp; m4_Kd = Kd; m4_Ki = Ki; m4_usePID = true; break;
  }
}

// Set PID target RPM
void setMotorSpeed(uint8_t motor, double targetRPM) {
  switch(motor) {
    case 1: m1_targetRPM = targetRPM; break;
    case 2: m2_targetRPM = targetRPM; break; 
    case 3: m3_targetRPM = targetRPM; break; 
    case 4: m4_targetRPM = targetRPM; break; 
  }
}

// Run PID algorithm for motor 1
void runMotor1PID() {
  m1_err = m1_targetRPM - motor1_rpm;
  m1_optPWM = round((m1_Kp * m1_err) + (m1_Ki * m1_integral) + (m1_Kd * m1_derivative));
  if(m1_optPWM > pow(2,PWM_RES)) { m1_optPWM = pow(2,PWM_RES); }
  else if (m1_optPWM < 0) { m1_optPWM = 0; }
  else m1_integral = m1_integral + (m1_err * (float)timeStep);
  m1_derivative = (m1_err - m1_errLast) / (float)timeStep;
  m1_errLast = m1_err;
  setMotorPWM(MOTOR1, (uint8_t)m1_optPWM);
}

// Run PID algorithm for motor 2
void runMotor2PID() {
  m2_err = m2_targetRPM - motor2_rpm;
  m2_optPWM = round((m2_Kp * m2_err) + (m2_Ki * m2_integral) + (m2_Kd * m2_derivative));
  if(m2_optPWM > pow(2,PWM_RES)) { m2_optPWM = pow(2,PWM_RES); }
  else if (m2_optPWM < 0) { m2_optPWM = 0; }
  else m2_integral = m2_integral + (m2_err * (float)timeStep);
  m2_derivative = (m2_err - m2_errLast) / (float)timeStep;
  m2_errLast = m2_err;
  setMotorPWM(MOTOR2, (uint8_t)m2_optPWM);
}

// Run PID algorithm for motor 3
void runMotor3PID() {
  m3_err = m3_targetRPM - motor3_rpm;
  m3_optPWM = round((m3_Kp * m3_err) + (m3_Ki * m3_integral) + (m3_Kd * m3_derivative));
  if(m3_optPWM > pow(2,PWM_RES)) { m3_optPWM = pow(2,PWM_RES); }
  else if (m3_optPWM < 0) { m3_optPWM = 0; }
  else m3_integral = m3_integral + (m3_err * (float)timeStep);
  m3_derivative = (m3_err - m3_errLast) / (float)timeStep;
  m3_errLast = m3_err;
  setMotorPWM(MOTOR3, (uint8_t)m3_optPWM);
}

// Run PID algorithm for motor 4
void runMotor4PID() {
  m4_err = m4_targetRPM - motor4_rpm;
  m4_optPWM = round((m4_Kp * m4_err) + (m4_Ki * m4_integral) + (m4_Kd * m4_derivative));
  if(m4_optPWM > pow(2,PWM_RES)) { m4_optPWM = pow(2,PWM_RES); }
  else if (m4_optPWM < 0) { m4_optPWM = 0; }
  else m4_integral = m4_integral + (m4_err * (float)timeStep);
  m4_derivative = (m4_err - m4_errLast) / (float)timeStep;
  m4_errLast = m4_err;
  setMotorPWM(MOTOR4, (uint8_t)m4_optPWM);
}

// Checks if there's a fault, either overtemp, voltage, or overcurrent
// If any of these are true, a system-wide fault is asserted.
boolean isFault() {
  return (fault1 || fault2 || fault3 || fault4 || motor1_oc || motor2_oc || motor3_oc || motor4_oc);
}

// Updated current battery voltage, as a double representing volts
// Reads analog reading, divides by 2^ADC_RES, multiplies by 3.3 to get voltage
// Divides by the resistive divider factor to recover battery voltage
void updateBatteryVoltage() {
  battVoltage = ((analogRead(BATT_SENSE) / ((float)pow(2,ADC_RES))) * 3.3) /  BATT_V_MULTIPLIER;
}

// Updates LED to a given color depending on current state of board
void updateLED() {
  // Fault State (either OT or OC fault present) -- RED
  if(isFault()) { 
    //digitalWrite(LED_R, LOW); digitalWrite(LED_G, HIGH); digitalWrite(LED_B, HIGH);
  }
  // Doing nothing (no motors being driven) -- BLUE
  else if((motor1_pwm == 0) && (motor2_pwm == 0) && (motor3_pwm == 0) && (motor4_pwm == 0)) {
    digitalWrite(LED_R, HIGH); digitalWrite(LED_G, HIGH); digitalWrite(LED_B, LOW);
  }
  // Battery voltage low -- MAGENTA
  else if(battVoltage < 10.0) {
    digitalWrite(LED_R, LOW); digitalWrite(LED_G, HIGH); digitalWrite(LED_B, LOW);
  }
  // All Good (functioning normally) -- GREEN
  else {
    digitalWrite(LED_R, HIGH); digitalWrite(LED_G, LOW); digitalWrite(LED_B, HIGH);
  }
}

void parsePacket() {
  if(isMotorPacket()) {
    switch(fid) {
      case FID_SETPWM: {
          setMotorDirection(cid, (boolean)params[1]);
          setMotorPWM(cid, params[0]);
          Serial.print("Set Motor ");
          Serial.print(cid);
          Serial.print(" to PWM ");
          Serial.print(params[0]);
          Serial.print(" and direction ");
          Serial.println(params[1]);
          break; }
      case FID_SETILIM: {
          setMotorCurrentLimit(cid, ((double)(getShort(params[0], params[1])) / 1000.0));
          Serial.print("Set Motor ");
          Serial.print(cid);
          Serial.print(" to current limit ");
          Serial.println(((double)su.val) / 1000.0);
          break; }
      case FID_GETSPD: {
          sendShort((uint16_t)getMotorRPM(cid));
          Serial.println("Replying with motor speed");
          break; }
      case FID_PIDEN: {
          enablePID(cid, params[0]);
          Serial.print("Setting PID to ");
          Serial.println(params[0]);
          break; }
      case FID_GETCUR: {
          sendShort((uint16_t)(getMotorCurrent(cid) * 1000));
          Serial.println("Replying with motor current");
          break; }
      case FID_SETPID: {
          float Kp = getFloat(params[0], params[1], params[2], params[3]);
          float Kd = getFloat(params[4], params[5], params[6], params[7]);
          float Ki = getFloat(params[8], params[9], params[10], params[11]);
          setPID(cid, Kp, Kd, Ki);
          Serial.println("Setting PID constants");
          break; }
      case FID_SETSPD: {
          setMotorDirection(cid, (boolean)params[2]);
          setMotorSpeed(cid, getShort(params[0], params[1]));
          Serial.print("Setting motor speed to ");
          Serial.println(m1_targetRPM);
          break; }
      case FID_RESENC: {
          resetEncoder(cid);
          Serial.println("Resetting encoder");
          break; }
      case FID_GETENC: {
          long ticks = getEncoderTicks(cid);
          uint32_t tt = (uint32_t)abs(ticks);
          sendInt(tt);
          if(ticks < 0) { MASTER_SERIAL.write(1); } // neg
          else { MASTER_SERIAL.write(0); } // pos
          Serial.println("Returning encoder ticks");
          break; }
      case FID_ROTPWM: {
          setMotorDirection(cid, params[1]);
          float rotsPWM = getFloat(params[2], params[3], params[4], params[5]);
          rotateMotorForRotations(cid, rotsPWM, params[0], false);
          Serial.println("Rotating at PWM");
          break; }
      case FID_ROTSPD: {
          setMotorDirection(cid, params[2]);
          uint16_t rpm = getShort(params[0], params[1]);
          float rotsRPM = getFloat(params[3], params[4], params[5], params[6]);
          rotateMotorForRotations(cid, rotsRPM, rpm, true);
          Serial.println("Rotating at RPM");
          break; }
      case FID_COAST: {
          useBrakeMotor(cid, false);
          Serial.println("Setting motor to Coast");
          break; }
      case FID_BRAKE: {
          useBrakeMotor(cid, true);
          Serial.println("Setting motor to Brake");
          break; }
      case FID_CSPD: {
          setMotorCoastSpeed(cid,getShort(params[0], params[1]));
          Serial.println("Setting coast speed to ");
          Serial.println(motor1_coastSpd);
          break; }
      case FID_STOP: {
          stopMotor(cid);
          Serial.println("Stopping motor");
          break; }
      case FID_REL: {
          releaseMotor(cid);
          Serial.println("Releasing motor");
          break; }
      case FID_FAULT: {
          MASTER_SERIAL.write(getMotorFault(cid));
          Serial.println("Replying with motor fault");
          break; }
      case FID_SETPPR: {
          setMotorPPR(cid, getFloat(params[0], params[1], params[2], params[3]));
          Serial.println("Setting motor PPR");
          break; }
    }
  } else if (isServoPacket()) {
    switch(fid) {
      case FID_SETANG:{
          setServoAngle(cid, params[0]);
          Serial.println("Setting servo angle");
          break; }
      case FID_SETSSP: {
          setServoAngle(cid, params[0]);
          Serial.println("Setting servo speed");
          break; }
    }
  } else if (isStepperPacket()) {
    
  }
}

boolean isMotorPacket() {
  if ((fid <= 0x10) || (fid = 0x1D)) return true;
  return false;
}

boolean isServoPacket() {
  if ((fid <= 0x11) || (fid = 0x12)) return true;
  return false;
}

boolean isStepperPacket() {
  if ((fid >= 0x14) || (fid <= 0x1B)) return true;
  return false;
}

void sendShort(uint16_t val) {
  su.val = val;
  MASTER_SERIAL.write(su.bytes[0]);
  MASTER_SERIAL.write(su.bytes[1]);
}

void sendFloat(float val) {
  fu.val = val;
  MASTER_SERIAL.write(fu.bytes[0]);
  MASTER_SERIAL.write(fu.bytes[1]);
  MASTER_SERIAL.write(fu.bytes[2]);
  MASTER_SERIAL.write(fu.bytes[3]);
}

void sendInt(int val) {
  iu.val = val;
  MASTER_SERIAL.write(iu.bytes[0]);
  MASTER_SERIAL.write(iu.bytes[1]);
  MASTER_SERIAL.write(iu.bytes[2]);
  MASTER_SERIAL.write(iu.bytes[3]);
}

float getFloat(uint8_t p1, uint8_t p2, uint8_t p3, uint8_t p4) {
  fu.bytes[0] = p1;
  fu.bytes[1] = p2;
  fu.bytes[2] = p3;
  fu.bytes[3] = p4;
  return fu.val;
}

uint16_t getShort(uint8_t p1, uint8_t p2) {
  su.bytes[0] = p1;
  su.bytes[1] = p2;
  return su.val;
}

// Enter receive buffer & process all active packets, one by one
void readPackets() {
  // If any new packets have been read, loop over them all
  while (hasPacket) {
    // Read header & nparams bytes, make parameter holding array
    header = rxbuff[buffr];
    advanceInBuffer();
    nparams = rxbuff[buffr];
    advanceInBuffer();

    // Recover Function ID & Component ID from header
    fid = header & functionId_mask;
    fid = fid >> 2;
    cid = componentId_mask & header;

    // Read all payload bytes into params
    for (int i = 0; i < nparams; i++) {
      params[i] = rxbuff[buffr];
      advanceInBuffer();
    }

    parsePacket();

    // Header can never be 0, so if next index is 0, there's no new packet.
    // If not 0, continue reading packets.
    if(rxbuff[buffr] == 0) hasPacket = false;
  }
}

void advanceInBuffer() {
  rxbuff[buffr] = 0; // Reset current buffer bin
  buffr++; // Advance buffer reading index
  if(buffr == 1000) buffr = 0; // Reset if at end of buffer
}

void printPacket() {
  Serial.print("FID: ");
  Serial.println(fid);
  Serial.print("CID: ");
  Serial.println(cid);
  for (int i = 0; i < nparams; i++) {
    Serial.print("Parameter ");
    Serial.print(i);
    Serial.print(": ");
    Serial.println(params[i]); 
  }
}

// Reads current serial data into buffer
void readIntoBuffer() {
  while(MASTER_SERIAL.available()) {
    rxbuff[buffw] = (uint8_t)MASTER_SERIAL.read();
    buffw++;
    if(buffw == 1000) buffw = 0;
    hasPacket = true;

    // This is necessary because of delay between SPI bytes being sent
    // RPi throttles the SPI bus speed with a 220-300 us delay between bytes
    // We use 300 us to be conservative
    delayMicroseconds(300);
  }
}

// For Testing
void printBuffer(int numElems) {
  for (int i = 0; i < numElems; i++) {
    Serial.print("[");
    Serial.print(rxbuff[i]);
    Serial.print("], ");
  }
  Serial.println();
}

// ======================== PRIMARY LOOP  ======================== 

void loop() {
  readCurrents();
  readFaultStates();
  fixFaults();
  updateRPMs();
  checkCurrents();
  runPID();
  updateLED();
  readIntoBuffer();
  readPackets();
  updateBatteryVoltage();
}
