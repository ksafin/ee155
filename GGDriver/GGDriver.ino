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

// ======================== LED DEFINITION  ======================== 
#define LED_R 2
#define LED_G 7
#define LED_B 8

// ======================== MOTOR DEFINITION  ======================== 

// MOTOR PWM PINS
#define MOTOR1_PWM 3
#define MOTOR2_PWM 4
#define MOTOR3_PWM 5
#define MOTOR4_PWM 6

// MOTOR GPIO PINS
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
#define ENC1_A 14
#define ENC1_B 23
#define ENC2_A 20
#define ENC2_B 21
#define ENC3_A 16
#define ENC3_B 17
#define ENC4_A 22
#define ENC4_B 15

// CURRENT SENSE PINS
#define ISENSE1 A10
#define ISENSE2 A11
#define ISENSE3 A12
#define ISENSE4 A13

// ======================== SERVO DEFINITION  ======================== 
#define SERVO1_PWM 10
#define SERVO2_PWM 9
#define SERVO3_PWM 25
#define SERVO4_PWM 32
#define LS_ENABLE 11

// ======================== AUXILIARY DEFINITION  ======================== 
#define BATT_SENSE A14

// ======================== MIXED-USE CONSTANTS  ======================== 
#define MOTOR1 1
#define MOTOR2 2
#define MOTOR3 3
#define MOTOR4 4

#define SERVO1 1
#define SERVO2 2
#define SERVO3 3
#define SERVO4 4

// 140 mV per A on current sense
#define MV_PER_A 140.0

// ======================== MOTOR STATE VARIABLES  ======================== 

// PWM STATES
volatile uint16_t motor1_pwm = 0;
volatile uint16_t motor2_pwm = 0;
volatile uint16_t motor3_pwm = 0;
volatile uint16_t motor4_pwm = 0;

// CURRENT DRAW
double motor1_curr = 0;
double motor2_curr = 0;
double motor3_curr = 0;
double motor4_curr = 0;

// OVERCURRENT
boolean motor1_oc = false;
boolean motor2_oc = false;
boolean motor3_oc = false;
boolean motor4_oc = false;

// CURRENT LIMITS
double motor1_ilim = 30;
double motor2_ilim = 30;
double motor3_ilim = 30;
double motor4_ilim = 30;

// RPM STATES
volatile uint16_t motor1_rpm = 0;
volatile uint16_t motor2_rpm = 0;
volatile uint16_t motor3_rpm = 0;
volatile uint16_t motor4_rpm = 0;

// COAST SPEEDS (ms to OFF)
uint16_t motor1_coastSpd = 500;
uint16_t motor2_coastSpd = 500;
uint16_t motor3_coastSpd = 500;
uint16_t motor4_coastSpd = 500;

// PPR VALUES
float m1_ppr = 134.4;
float m2_ppr = 134.4;
float m3_ppr = 134.4;
float m4_ppr = 134.4;

// FAULT STATES
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
uint16_t timeStep;

// PID VARIABLES FOR EACH MOTOR
boolean m1_usePID = false;
boolean m2_usePID = false;
boolean m3_usePID = false;
boolean m4_usePID = false;

double m1_targetRPM = 0.0;
double m2_targetRPM = 0.0;
double m3_targetRPM = 0.0;
double m4_targetRPM = 0.0;

uint16_t m1_err = 0.0;
uint16_t m2_err = 0.0;
uint16_t m3_err = 0.0;
uint16_t m4_err = 0.0;

double m1_Kp = 0.0;
double m2_Kp = 0.0;
double m3_Kp = 0.0;
double m4_Kp = 0.0;

double m1_Kd = 0.0;
double m2_Kd = 0.0;
double m3_Kd = 0.0;
double m4_Kd = 0.0;

double m1_Ki = 0.0;
double m2_Ki = 0.0;
double m3_Ki = 0.0;
double m4_Ki = 0.0;

double m1_integral = 0.0;
double m2_integral = 0.0;
double m3_integral = 0.0;
double m4_integral = 0.0;

double m1_derivative = 0.0;
double m2_derivative = 0.0;
double m3_derivative = 0.0;
double m4_derivative = 0.0;

double m1_errLast = 0.0;
double m2_errLast = 0.0;
double m3_errLast = 0.0;
double m4_errLast = 0.0;


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

// SERVO DEFINITION STATES
// REPRESENTS ANGLE OF SERVO, IN DEGREES
// CONTINUOUS ROTATION SERVO HAS AN ANGLE OF 0
uint16_t servo1_angle = 180;
uint16_t servo2_angle = 180;
uint16_t servo3_angle = 180;
uint16_t servo4_angle = 180;

Servo servo1;
Servo servo2;
Servo servo3;
Servo servo4;

// ======================== SETTING VARIABLES  ========================
// DIVIDE BY THIS TO GET BATTERY VOLTAGE
const double BATT_V_MULTIPLIER = 0.2115384615;

// BATTERY VOLTAGE
double battVoltage;

// PWM FREQUENCY & RESOLUTION
const double PWM_FREQ = 11718.75;
const uint8_t PWM_RES = 12;

// ANALOG READING
const uint8_t ADC_RES = 12; // 2^ADC_RES; 4096
const uint8_t ADC_AVG = 16; // ADC AVERAGING

Encoder enc1(ENC1_A, ENC1_B);
Encoder enc2(ENC2_A, ENC2_B);
Encoder enc3(ENC3_A, ENC3_B);
Encoder enc4(ENC4_A, ENC4_B);

long m1_100ms_ticks = 0;
long m2_100ms_ticks = 0;
long m3_100ms_ticks = 0;
long m4_100ms_ticks = 0;

// Total current of all four motors
int totalCurrent;

// Checks amount of time remaining to coast
IntervalTimer m1_timer;
IntervalTimer m2_timer;
IntervalTimer m3_timer;
IntervalTimer m4_timer;

// Timer for rpm measurements
Metro encTimer = Metro(100);

// Timer for PID iteration (ms)
Metro pidTimer = Metro(timeStep);

// Settings for reaction to current limits being hit
long waitTime_oc = 500000; // microseconds; 500 ms wait time after overcurrent.
long waitTime_ot = 500000; // microseconds; 500 ms wait time after overtemp.

// ======================== END OF VARIABLES  ========================

void setup() {
  // INITIALIZE PWM
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
  pinMode(SPI_RESET, OUTPUT);

  // AUX
  pinMode(BATT_SENSE, INPUT);

  m1_100ms_ticks = enc1.read();
  m2_100ms_ticks = enc2.read();
  m3_100ms_ticks = enc3.read();
  m4_100ms_ticks = enc4.read();
}

// ======================== READ CURRENTS ======================== 
void readCurrents() {
  motor1_curr = ((analogRead(ISENSE1)/((float)(pow(2,ADC_RES)))) * 3.3) / MV_PER_A;
  motor2_curr = ((analogRead(ISENSE2)/((float)(pow(2,ADC_RES)))) * 3.3) / MV_PER_A;
  motor3_curr = ((analogRead(ISENSE3)/((float)(pow(2,ADC_RES)))) * 3.3) / MV_PER_A;
  motor4_curr = ((analogRead(ISENSE4)/((float)(pow(2,ADC_RES)))) * 3.3) / MV_PER_A;
  totalCurrent = motor1_curr + motor2_curr + motor3_curr + motor4_curr;
}

// ======================== CHECK FOR FAULTS  ======================== 
void readFaultStates() {
  fault1 = (~mcp.digitalRead(EN1_A)) || (~mcp.digitalRead(EN1_B));
  fault2 = (~mcp.digitalRead(EN2_A)) || (~mcp.digitalRead(EN2_B));
  fault3 = (~mcp.digitalRead(EN2_A)) || (~mcp.digitalRead(EN2_B));
  fault4 = (~mcp.digitalRead(EN2_A)) || (~mcp.digitalRead(EN2_B));
}

// ======================== SET MOTOR PWM  ======================== 
void setMotorPWM(uint8_t motor, uint16_t pwm) {
  switch(motor) {
    case 1: motor1_pwm = pwm; analogWrite(MOTOR1_PWM, pwm); break;
    case 2: motor2_pwm = pwm; analogWrite(MOTOR2_PWM, pwm); break;
    case 3: motor3_pwm = pwm; analogWrite(MOTOR3_PWM, pwm); break;
    case 4: motor4_pwm = pwm; analogWrite(MOTOR4_PWM, pwm); break;
  }
}

// ======================== SET MOTOR CURRENT LIMIT  ======================== 
void setMotorCurrentLimit(uint8_t motor, double ilim) {
  switch(motor) {
    case 1: motor1_ilim = ilim; break;
    case 2: motor2_ilim = ilim; break;
    case 3: motor3_ilim = ilim; break;
    case 4: motor4_ilim = ilim; break;
  }
}

// ======================== SET MOTOR DIRECTION  ======================== 
void setMotorDirection(uint8_t motor, boolean cw) {
  switch(motor) {
    case 1: motor1_cw = cw;
            mcp.digitalWrite(IN1_A,cw);
            mcp.digitalWrite(IN1_B,~cw);
            break;
    case 2: motor2_cw = cw;
            mcp.digitalWrite(IN2_A,cw);
            mcp.digitalWrite(IN2_B,~cw);
            break;
    case 3: motor3_cw = cw;
            mcp.digitalWrite(IN3_A,cw);
            mcp.digitalWrite(IN3_B,~cw);
            break;
    case 4: motor4_cw = cw;
            mcp.digitalWrite(IN4_A,cw);
            mcp.digitalWrite(IN4_B,~cw);
            break;
  }
}

// ======================== GET MOTOR DIRECTION  ======================== 
boolean getMotorDirection(uint8_t motor) {
  switch(motor) {
    case 1: return motor1_cw;
    case 2: return motor2_cw;
    case 3: return motor3_cw;
    case 4: return motor4_cw;
    default: return false;
  }
}

// ======================== GET MOTOR CURRENT DRAW  ======================== 
double getMotorCurrent(uint8_t motor) {
  switch(motor) {
    case 1: return motor1_curr; 
    case 2: return motor2_curr;
    case 3: return motor3_curr;
    case 4: return motor4_curr;
    default: return 0;
  }
}

// ======================== RETURN ENCODER TICKS  ======================== 
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
void resetEncoder(uint8_t motor) {
  switch(motor) {
    case 1: enc1.write(0);
    case 2: enc1.write(0);
    case 3: enc1.write(0);
    case 4: enc1.write(0);
  }
}

// ======================== SET BRAKE SETTING  ======================== 
void useBrakeMotor(uint8_t motor, boolean isBrake) {
  switch(motor) {
    case 1: motor1_brake = isBrake; break;
    case 2: motor1_brake = isBrake; break;
    case 3: motor1_brake = isBrake; break;
    case 4: motor1_brake = isBrake; break;
  }
}

// ======================== BRAKE MOTOR TO VCC  ======================== 
void brakeMotor(uint8_t motor) {
  switch(motor) {
    case 1: mcp.digitalWrite(IN1_A, HIGH);
            mcp.digitalWrite(IN1_B, HIGH);
            break;
    case 2: mcp.digitalWrite(IN2_A, HIGH);
            mcp.digitalWrite(IN2_B, HIGH);
            break;
    case 3: mcp.digitalWrite(IN3_A, HIGH);
            mcp.digitalWrite(IN3_B, HIGH);
            break;
    case 4: mcp.digitalWrite(IN4_A, HIGH);
            mcp.digitalWrite(IN4_B, HIGH);
            break;
  }
}

// ======================== CLEAR MOTOR FAULT  ======================== 
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
void coastMotor(uint8_t motor) {
  switch(motor) {
    case 1: m1_timer.begin(coastMotor1,round((((float)motor1_coastSpd) * 1000) / motor1_pwm)); break;
    case 2: m2_timer.begin(coastMotor2,round((((float)motor2_coastSpd) * 1000) / motor2_pwm)); break;
    case 3: m3_timer.begin(coastMotor3,round((((float)motor3_coastSpd) * 1000) / motor3_pwm)); break;
    case 4: m4_timer.begin(coastMotor4,round((((float)motor4_coastSpd) * 1000) / motor4_pwm)); break;
  }
}

// ======================== STOP MOTOR BRAKE & COAST  ======================== 
void stopMotor(uint8_t motor) {
   switch(motor) {
    case 1: if(motor1_brake) { brakeMotor(motor); }
            else { coastMotor(motor); }
    case 2: if(motor2_brake) { brakeMotor(motor); }
            else { coastMotor(motor); }
    case 3: if(motor3_brake) { brakeMotor(motor); }
            else { coastMotor(motor); }
    case 4: if(motor4_brake) { brakeMotor(motor); }
            else { coastMotor(motor); }
  }
}

// ======================== UPDATE RPMS FOR ALL MOTORS  ======================== 
void updateRPMs() {
  if(encTimer.check() == 1) {
    motor1_rpm = (absoluteDifference(enc1.read(),m1_100ms_ticks) / m1_ppr) * 10; 
    motor2_rpm = (absoluteDifference(enc2.read(),m2_100ms_ticks) / m2_ppr) * 10;
    motor3_rpm = (absoluteDifference(enc3.read(),m3_100ms_ticks) / m3_ppr) * 10;
    motor4_rpm = (absoluteDifference(enc4.read(),m4_100ms_ticks) / m4_ppr) * 10;
    m1_100ms_ticks = enc1.read();
    m2_100ms_ticks = enc2.read();
    m3_100ms_ticks = enc3.read();
    m4_100ms_ticks = enc4.read();
  }
}

// ======================== MEASURE ENCODER CHANGE  ======================== 
long absoluteDifference(long val1, long val2) {
  if(val1 > val2) return val1-val2;
  return val2 - val1;
}

// ======================== INTERRUPTS FOR COASTING ======================== 
void coastMotor1() {
  motor1_pwm = motor1_pwm - 1;
  if(motor1_pwm == 0) m1_timer.end();
}

void coastMotor2() {
  motor2_pwm = motor2_pwm - 1;
  if(motor2_pwm == 0) m2_timer.end();
}

void coastMotor3() {
  motor3_pwm = motor3_pwm - 1;
  if(motor3_pwm == 0) m3_timer.end();
}

void coastMotor4() {
  motor4_pwm = motor4_pwm - 1;
  if(motor4_pwm == 0) m4_timer.end();
}

// ======================== INTERRUPTS FOR OVERCURRENT  ======================== 
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

// ======================== INTERRUPTS FOR OVERTEMP  ======================== 
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
void fixFaults() {
  if(fault1) m1_timer.begin(m1_fault, waitTime_ot);
  if(fault2) m2_timer.begin(m2_fault, waitTime_ot);
  if(fault3) m3_timer.begin(m3_fault, waitTime_ot);
  if(fault4) m4_timer.begin(m4_fault, waitTime_ot);
}

// ======================== SERVO SET ANGLE  ======================== 
void setServoAngle(uint8_t servo, uint16_t angle) {
  switch(servo) {
    case 1: servo1.write(getAngleValue(angle, servo1_angle)); break;
    case 2: servo2.write(getAngleValue(angle, servo2_angle)); break;
    case 3: servo3.write(getAngleValue(angle, servo3_angle)); break;
    case 4: servo4.write(getAngleValue(angle, servo4_angle)); break;
  }
}

uint8_t getAngleValue(uint16_t desAngle, uint16_t maxAngle) {
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
}

void runPID() {
  if(pidTimer.check()) {
    if(m1_usePID) runMotor1PID();
    if(m2_usePID) runMotor2PID();
    if(m3_usePID) runMotor3PID();
    if(m4_usePID) runMotor4PID();
  }
}

void runMotor1PID() {
  m1_err = m1_targetRPM - motor1_rpm;
  motor1_pwm = round((m1_Kp * m1_err) + (m1_Ki * m1_integral) + (m1_Kd * m1_derivative));
  if(motor1_pwm > pow(2,PWM_RES)) { motor1_pwm = pow(2,PWM_RES); }
  else if (motor1_pwm < 0) { motor1_pwm = 0; }
  else m1_integral = m1_integral + (m1_err * timeStep);
  m1_derivative = (m1_err - m1_errLast) / timeStep;
  m1_errLast = m1_err;
  setMotorPWM(MOTOR1, motor1_pwm);
}

void runMotor2PID() {
  m2_err = m2_targetRPM - motor2_rpm;
  motor2_pwm = round((m2_Kp * m2_err) + (m2_Ki * m2_integral) + (m2_Kd * m2_derivative));
  if(motor2_pwm > pow(2,PWM_RES)) { motor2_pwm = pow(2,PWM_RES); }
  else if (motor2_pwm < 0) { motor2_pwm = 0; }
  else m2_integral = m2_integral + (m2_err * timeStep);
  m2_derivative = (m2_err - m2_errLast) / timeStep;
  m2_errLast = m2_err;
  setMotorPWM(MOTOR2, motor2_pwm);
}

void runMotor3PID() {
  m3_err = m3_targetRPM - motor3_rpm;
  motor3_pwm = round((m3_Kp * m3_err) + (m3_Ki * m3_integral) + (m3_Kd * m3_derivative));
  if(motor3_pwm > pow(2,PWM_RES)) { motor3_pwm = pow(3,PWM_RES); }
  else if (motor3_pwm < 0) { motor3_pwm = 0; }
  else m3_integral = m3_integral + (m3_err * timeStep);
  m3_derivative = (m3_err - m3_errLast) / timeStep;
  m3_errLast = m3_err;
  setMotorPWM(MOTOR3, motor3_pwm);
}

void runMotor4PID() {
  m4_err = m4_targetRPM - motor4_rpm;
  motor4_pwm = round((m4_Kp * m4_err) + (m4_Ki * m4_integral) + (m4_Kd * m4_derivative));
  if(motor4_pwm > pow(2,PWM_RES)) { motor4_pwm = pow(2,PWM_RES); }
  else if (motor4_pwm < 0) { motor4_pwm = 0; }
  else m4_integral = m4_integral + (m4_err * timeStep);
  m4_derivative = (m4_err - m4_errLast) / timeStep;
  m4_errLast = m4_err;
  setMotorPWM(MOTOR4, motor4_pwm);
}

boolean isFault() {
  return (fault1 || fault2 || fault3 || fault4 || motor1_oc || motor2_oc || motor3_oc || motor4_oc);
}

void updateBatteryVoltage() {
  battVoltage = ((analogRead(BATT_SENSE) / ((float)pow(2,ADC_RES))) * 3.3) /  BATT_V_MULTIPLIER;
}

void updateLED() {
  // Fault State (either OT or OC fault present) -- RED
  if(isFault()) { 
    digitalWrite(LED_R, LOW); digitalWrite(LED_G, HIGH); digitalWrite(LED_B, HIGH);
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

// ======================== PRIMARY LOOP  ======================== 
void loop() {
  readCurrents();
  readFaultStates();
  fixFaults();
  updateRPMs();
  checkCurrents();
  runPID();
  updateBatteryVoltage();
  updateLED();
}
