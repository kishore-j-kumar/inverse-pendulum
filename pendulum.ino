
// Include the AccelStepper Library
#include <AccelStepper.h>
#include <elapsedMillis.h>
#include <TimedAction.h>

// Motor Connections (unipolar motor driver)
const int In1 = 8;
const int In2 = 9;
const int In3 = 10;
const int In4 = 11;
// Motor Connections (constant voltage bipolar H-bridge motor driver)
const int AIn1 = 8;
const int AIn2 = 9;
const int BIn1 = 10;
const int BIn2 = 11;
// Motor Connections (constant current, step/direction bipolar motor driver)
const int dirPin = 4;
const int stepPin = 5;
boolean toggle1 = 0;

const float rotation_distance = 0.039;
float K_LQR = {1.0000, 0.4877, 0.6240, -0.0373};

float angle;
float stepperPos;
float stepperVel;
float angleVel;

int delayTime;
float acceleration;
float previous_rot_velocity;
float current_rot_velocity; 
float rotational_acceleration;
float mass = 0.011 + 0.04429; 

const unsigned int MAX_MESSAGE_LENGTH = 12;

// Creates an instance - Pick the version you want to use and un-comment it. That's the only required change.
//AccelStepper myStepper(AccelStepper::FULL4WIRE, AIn1, AIn2, BIn1, BIn2);  // works for TB6612 (Bipolar, constant voltage, H-Bridge motor driver)
//AccelStepper myStepper(AccelStepper::FULL4WIRE, In1, In3, In2, In4);    // works for ULN2003 (Unipolar motor driver)
AccelStepper stepper(AccelStepper::DRIVER, stepPin, dirPin);           // works for a4988 (Bipolar, constant current, step/direction driver)

elapsedMillis printTime;

const int maxSpeedLimit = 20000.0;  // set this to the maximum speed you want to use.


// Run motor in thread
void getAngle() {
  while (Serial.available() > 0) {
    static char message[MAX_MESSAGE_LENGTH];
    static unsigned int message_pos = 0;

    char inByte = Serial.read();
    if (inByte != ',') {
      message[message_pos] = inByte;
      message_pos++;
    } else {
      message[message_pos] = '\0';
      angle = atof(message);
      message_pos = 0;
    }
  }
}

void move_stepper() {
  // correctionDistance = 10
  digitalWrite(dirPin,HIGH);
  for(int x = 0; x < 200; x++) {
    digitalWrite(stepPin,HIGH); 
    delayMicroseconds(475); 
    digitalWrite(stepPin,LOW); 
    delayMicroseconds(475); 
  }

}

// Converts requested rotational velocity (rps) to delay interval time (ms)
void setDelayTime(float acceleration) {
  //rotational velocity: rotations/second
  // int proposedDelayTime = abs(int(2500 / rotational_velocity));
  // if (proposedDelayTime < 475) {
  //   delayTime = 475;
  // } else {
  //   delayTime = proposedDelayTime;
  // }
  float absAcceleration = abs(acceleration);
  float proportion = absAcceleration / 0.785;
  delayTime = (2000 - (2000 - 475) * (proportion));
}

float linear_to_rot(float rotational_value) {
  return rotational_value / rotation_distance;
}

// Runs stepper motor at given acceleration
void setStepper() {
  stepper.move(1);
}

void moveStepper() {
  digitalWrite(stepPin,HIGH); 
  delayMicroseconds(delayTime); 
  digitalWrite(stepPin,LOW); 
  delayMicroseconds(delayTime); 
}

// Initialize thread to recieve angle data
TimedAction angleThread = TimedAction(10, getAngle);
// Initialize thread to set stepper motor acceleration
TimedAction stepperThread = TimedAction(1, setStepper);


void setup() {
  delayTime = 475;
  acceleration = 0;
  previous_rot_velocity = 0;
  current_rot_velocity = 0; 
  rotational_acceleration = 0;

  Serial.begin(115200);
  Serial.flush();
  
  // set the maximum speed and initial speed.
  // myStepper.setMaxSpeed(maxSpeedLimit); 
  // myStepper.setSpeed(10000);    // initial speed target
  // myStepper.setAcceleration(1000000);

  stepper.setMaxSpeed(100000);
  stepper.setAcceleration(1000000);
  stepper.setSpeed(100000);

  cli();
  TCCR1A = 0;// set entire TCCR1A register to 0
  TCCR1B = 0;// same for TCCR1B
  TCNT1  = 0;//initialize counter value to 0
  // set compare match register for 1hz increments
  OCR1A = 156;// = (16*10^6) / (1*1024) - 1 (must be <65536)
  // turn on CTC mode
  TCCR1B |= (1 << WGM12);
  // Set CS12 and CS10 bits for 1024 prescaler
  TCCR1B |= (1 << CS12) | (1 << CS10);  
  // enable timer compare interrupt
  TIMSK1 |= (1 << OCIE1A);

  sei();
  pinMode(LED_BUILTIN, OUTPUT);
  // move_stepper();
}


void move_stepper(int value){
  if (stepper.distanceToGo() == 0)
    {
      stepper.move(value % 200);
    }
    stepper.setAcceleration(1000*value);
    Serial.println(stepper.acceleration());
    stepper.run();
    stepperPos = stepper.position();
    stepperVel = stepper.velocity();
}

void loop() {
  angleThread.check();
  angleVel = (angle-oldAngle)/loopTime;
  float oldAngle = angle;

  int output = K_LQR*{stepperPos, stepperVel, angle, angleVel};

  float correctionFactor = output * angle; //takes mass + motor tuning into account
  move_stepper(correctionFactor*1542);

}
