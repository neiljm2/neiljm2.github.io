#include <math.h>
#define STEP_PIN1 22  // Pin connected to STEP on A4988
#define DIR_PIN1 23    // Pin connected to DIR on A4988
#define STEP_PIN2 24  // Pin connected to STEP on A4988
#define DIR_PIN2 25    // Pin connected to DIR on A4988
#define STEP_PIN3 26  // Pin connected to STEP on A4988
#define DIR_PIN3 27  // Pin connected to DIR on A4988
#define STEP_PIN4 28  // Pin connected to STEP on A4988
#define DIR_PIN4 29    // Pin connected to DIR on A4988
#define IN1 7
#define IN2 6
#define pwmPin5 3
#define IN3 10
#define IN4 11
#define pwmPin6 9
#define encPin1 A1
#define encPin2 A2
#define encPin3 A3
#define encPin4 A4
#define encPin5 A15
#define encPin6 A6

//Joint limits

float motor1LB = -360.0; //units of degrees
float motor1UB =  360.0;

float motor2LB = -205.0;
float motor2UB =  45.0;

float motor3LB = -145.0;
float motor3UB =  145.0;

float motor4LB = -720.0;
float motor4UB =  720.0;

float motor5LB = -360.0/360.0;////EDIT FOR DCS
float motor5UB =  360.0/360.0;

float motor6LB = -360.0/360.0;
float motor6UB =  360.0/360.0;

//Stepper globals
bool steppersOff = true; 
bool motor1Running = false;  // State to track HIGH/LOW for step pulse
bool motor2Running = false;
bool motor3Running = false;  // State to track HIGH/LOW for step pulse
bool motor4Running = false;

const unsigned long clockFreq = 16000000; //Hz

int steps1;
int steps2;
int steps3;
int steps4;
int mostSteps;
short mostStepsIndex;

int count1 = 0;
int count2 = 0;
int count3 = 0;
int count4 = 0;

float desPos1; //degrees
float desPos2; //degrees
float desPos3; //degrees
float desPos4; //degrees

float previousDesPos1 = 0.0;
float previousDesPos2 = 0.0;
float previousDesPos3 = 0.0;
float previousDesPos4 = 0.0;

float curPos1; //degrees
float curPos2; //degrees
float curPos3; //degrees
float curPos4; //degrees

float theta1; //degrees
float theta2; //degrees
float theta3; //degrees
float theta4; //degrees

int rot1 = 0;
int rot2 = 0;
int rot3 = 0;
int rot4 = 0;

short steps[4];
float frequencies[4];
float timescale; 
float encZeroes[4] = {0.0, 0.0, 0.0, 0.0};

//DC Globals

const float loopTime = 1; // every X ms, the code enters update_speed()
int timePassed = 0;
unsigned long prevMillis = 0; // last time (ms) the code entered update_speed()

bool passZero5 = false;
bool passZero6 = false;

float currPos5 = 0;
float desPos5 = 0;
int currPWM5 = 100; // current++ PWM output
bool motor5Running = false;

float currPos6 = 0;
float desPos6 = 0;
int currPWM6 = 100; // current++ PWM output
bool motor6Running = false;

int toc = 0;
int tic = 0;

float error5 = 0;
float error6 = 0;

int rot5 = 0;
int rot6 = 0;

float Ki5 = 10;
float Ki6 = 3.8;

void setup() {
  // Set STEP and DIR pins as outputs
  pinMode(STEP_PIN1, OUTPUT);
  pinMode(DIR_PIN1, OUTPUT);
  pinMode(STEP_PIN2, OUTPUT);
  pinMode(DIR_PIN2, OUTPUT);
  pinMode(STEP_PIN3, OUTPUT);
  pinMode(DIR_PIN3, OUTPUT);
  pinMode(STEP_PIN4, OUTPUT);
  pinMode(DIR_PIN4, OUTPUT);
  pinMode(encPin1, INPUT);
  pinMode(encPin2, INPUT);
  pinMode(encPin3, INPUT);
  pinMode(encPin4, INPUT);
  pinMode(34, OUTPUT);
  pinMode(35, OUTPUT);
  pinMode(36, OUTPUT);
  pinMode(37, OUTPUT);
  digitalWrite(34, HIGH);
  digitalWrite(35, HIGH);
  digitalWrite(36, HIGH);
  digitalWrite(37, HIGH);
  //DC pin settings
  pinMode(encPin5, INPUT);
  pinMode(encPin6, INPUT);
  pinMode(pwmPin5, OUTPUT);
  pinMode(pwmPin6, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  readEncoderDC(currPos5, currPos6, desPos5, desPos6);
  // Set the initial direction (rotate in one direction)
  digitalWrite(DIR_PIN1, LOW);  // Set stepper direction pin (can be HIGH or LOW)
  digitalWrite(DIR_PIN2, LOW);
  digitalWrite(DIR_PIN3, LOW); 
  digitalWrite(DIR_PIN4, LOW);
  // Start serial communication for debugging
  Serial.begin(115200);

  readEncoderStepper(&curPos1, A1, 0.0);
  readEncoderStepper(&curPos2, A2, 0.0);
  readEncoderStepper(&curPos3, A3, 0.0);    
  readEncoderStepper(&curPos4, A4, 0.0);
  encZeroes[0] = curPos1;
  encZeroes[1] = curPos2;
  encZeroes[2] = curPos3;
  encZeroes[3] = curPos4;

  // Disable interrupts while configuring
  noInterrupts();
  //----------------------------------------------------------------------------------------------------
  // Reset Timer1
  TCCR1A = 0;  // Normal operation, no PWM
  TCCR1B = 0;  // Reset Timer1 settings
  
  // Set Timer1 in CTC (Clear Timer on Compare Match) mode
  TCCR1B |= (1 << WGM12);  // CTC Mode
  TCCR1B |= (1 << CS10) | (1 << CS11);   // Prescaler = 8 (16 MHz timer frequency, 2MHz effective timer frequency)

  // Set OCR1A for the 500 µs HIGH pulse (1000 timer counts)
  OCR1A = 1000; //init
  OCR1B = 499;
  // Enable Timer1 Compare Match A and B interrupts
  TIMSK1 |= (1 << OCIE1A) | (1<< OCIE1B);
  //-----------------------------------------------------------------------------------------------------
  // Reset Timer3 (2)
  TCCR3A = 0;  // Normal operation, no PWM
  TCCR3B = 0;  // Reset Timer1 settings
  
  // Set Timer3 in CTC (Clear Timer on Compare Match) mode
  TCCR3B |= (1 << WGM32);  // CTC Mode
  TCCR3B |= (1<<CS30) | (1 << CS31);   // Prescaler = 8 (16 MHz timer frequency, 2MHz effective timer frequency)

  // Set OCR1A for the 500 µs HIGH pulse (1000 timer counts)
  OCR3A = 1000; //init
  OCR3B = 499;
  // Enable Timer3 Compare Match A and B interrupts
  TIMSK3 |= (1 << OCIE3A) | (1<< OCIE3B);
  //-----------------------------------------------------------------------------------------------------
  // Reset Timer4 (3)
  TCCR4A = 0;  // Normal operation, no PWM
  TCCR4B = 0;  // Reset Timer1 settings
  
  // Set Timer3 in CTC (Clear Timer on Compare Match) mode
  TCCR4B |= (1 << WGM42);  // CTC Mode
  TCCR4B |= (1<<CS40) | (1 << CS41);   // Prescaler = 8 (16 MHz timer frequency, 2MHz effective timer frequency)

  // Set OCR1A for the 500 µs HIGH pulse (1000 timer counts)
  OCR4A = 1000; //init
  OCR4B = 499;
  // Enable Timer4 Compare Match A and B interrupts
  TIMSK4 |= (1 << OCIE4A) | (1<< OCIE4B);
  //-----------------------------------------------------------------------------------------------------
    // Reset Timer3 (2)
  TCCR5A = 0;  // Normal operation, no PWM
  TCCR5B = 0;  // Reset Timer1 settings
  
  // Set Timer3 in CTC (Clear Timer on Compare Match) mode
  TCCR5B |= (1 << WGM52);  // CTC Mode
  TCCR5B |= (1<<CS50) | (1 << CS51);   // Prescaler = 8 (16 MHz timer frequency, 2MHz effective timer frequency)

  // Set OCR1A for the 500 µs HIGH pulse (1000 timer counts)
  OCR5A = 1000; //init
  OCR5B = 499;
  // Enable Timer5 Compare Match A and B interrupts
  TIMSK5 |= (1 << OCIE5A) | (1<< OCIE5B);
  //-----------------------------------------------------------------------------------------------------
  // Enable global interrupts
  interrupts();
}

//1----------------------------------------------------------------------------------------------------
ISR(TIMER1_COMPA_vect) {
  if (motor1Running) {
    digitalWrite(STEP_PIN1, HIGH);  // Generate step pulse
    count1 ++;
  }
}

ISR(TIMER1_COMPB_vect) {
  if (motor1Running) {
    digitalWrite(STEP_PIN1, LOW);   // End step pulse
  }
}

//2----------------------------------------------------------------------------------------------------
ISR(TIMER3_COMPA_vect) {
  if (motor2Running) {
    digitalWrite(STEP_PIN2, HIGH);  // Generate step pulse
    count2 ++;
  }
}

ISR(TIMER3_COMPB_vect) {
  if (motor2Running) {
    digitalWrite(STEP_PIN2, LOW);   // End step pulse
  }
}

//3----------------------------------------------------------------------------------------------------
ISR(TIMER4_COMPA_vect) {
  if (motor3Running) {
    digitalWrite(STEP_PIN3, HIGH);  // Generate step pulse
    count3 ++;
  }
}

ISR(TIMER4_COMPB_vect) {
  if (motor3Running) {
    digitalWrite(STEP_PIN3, LOW);   // End step pulse
  }
}
//4----------------------------------------------------------------------------------------------------
ISR(TIMER5_COMPA_vect) {
  if (motor4Running) {
    digitalWrite(STEP_PIN4, HIGH);  // Generate step pulse
    count4 ++;
  }
}

ISR(TIMER5_COMPB_vect) {
  if (motor4Running) {
    digitalWrite(STEP_PIN4, LOW);   // End step pulse
  }
}

//-----------------------------------------------------------------------------------------------------
void loop() {
  (!motor1Running && !motor2Running && !motor3Running && !motor4Running) ? steppersOff = true : steppersOff = false;
  timePassed = millis() - timePassed;
  if (Serial.available() > 0) {
    parseString();
    if (steppersOff) { 
      runSteppers(); //SHOULD TAKE DESPOS1-4, AND SHOULD NOT BE GLOBAL VARIABLES || ALSO ADD A FLAG TO SIGNAL WHEN THIS IS COMPLETE AND CORRECT STEPPERS NEEDS TO RUN
    }
  }if(count1 >= steps[0]){
    motor1Running = false;
    count1 = 0;
  }if(count2 >= steps[1]){
    motor2Running = false;
    count2 = 0;
  }if(count3 >= steps[2]){
    motor3Running = false;
    count3 = 0;
  }if(count4 >= steps[3]){
    motor4Running = false;
    count4 = 0;
  }
  updateDCs();
}

void parseString(){
  String command = Serial.readString(); // Read input until newline
  command.trim();  // Remove any extra spaces or newline characters
  int commaLoc1 = command.indexOf(',');
  int commaLoc2 = -1;
  int commaLoc3 = -1;
  int commaLoc4 = -1;
  int commaLoc5 = -1;
  int commaLoc6 = -1;
  desPos1 = (command.substring(0, commaLoc1)).toFloat();
  desPos2 = desPos1;
  desPos3 = desPos1;
  desPos4 = desPos1;
  desPos5 = desPos1;
  desPos6 = desPos1;
  if (commaLoc1 != -1){
    desPos2 = (command.substring(commaLoc1+1, commaLoc2)).toFloat();
    commaLoc2 = command.indexOf(',', commaLoc1+1);
  }
  if (commaLoc2 != -1){
    desPos3 = (command.substring(commaLoc2+1, commaLoc3)).toFloat();
    commaLoc3 = command.indexOf(',', commaLoc2+1);
  }
  if (commaLoc3 != -1){
    desPos4 = (command.substring(commaLoc3+1, commaLoc4)).toFloat();
    commaLoc4 = command.indexOf(',', commaLoc3+1);
  }
  if (commaLoc4 != -1){
    desPos5 = (command.substring(commaLoc4+1, commaLoc5)).toFloat();
    commaLoc5 = command.indexOf(',', commaLoc4+1);
  }
  if (commaLoc5 != -1){
    desPos6 = (command.substring(commaLoc5+1, commaLoc6)).toFloat();
    commaLoc6 = command.indexOf(',', commaLoc5+1);
  }
}

void runSteppers(){
  float guess1 = fmod(previousDesPos1*20, 360.0)/20.0;
  if (guess1 < 0) guess1 += 18;
  float guess2 = fmod(previousDesPos2*50, 360.0)/50.0;
  if (guess2 < 0) guess2 += 7.2;
  float guess3 = fmod(previousDesPos3*20, 360.0)/20.0;
  if (guess3 < 0) guess3 += 18;
  float guess4 = fmod(previousDesPos4*20, 360.0)/20.0;
  if (guess4 < 0) guess4 += 18;

  readEncoderStepper(&curPos1, A1, guess1);
  previousDesPos1 = desPos1;
  theta1 = desPos1 - (rot1*(360/20) + curPos1); //find geared angle to move

  readEncoderStepper(&curPos2, A2, guess2);
  previousDesPos2 = desPos2;
  theta2 = desPos2 - (rot2*(360/50) + curPos2); //find geared angle to move

  readEncoderStepper(&curPos3, A3, guess3);
  previousDesPos3 = desPos3;
  theta3 = desPos3 - (rot3*(360/20) + curPos3); //find geared angle to move

  readEncoderStepper(&curPos4, A4, guess4);
  previousDesPos4 = desPos4;
  theta4 = desPos4 - (rot4*(360/20) + curPos4); //find geared angle to move

  if (theta1 > 180.0) theta1 -= 360.0;
  if (theta1 < -180.0) theta1 += 360.0;
  if (theta2 > 180.0) theta2 -= 360.0;
  if (theta2 < -180.0) theta2 += 360.0;
  if (theta3 > 180.0) theta3 -= 360.0;
  if (theta3 < -180.0) theta3 += 360.0;
  if (theta4 > 180.0) theta4 -= 360.0;
  if (theta4 < -180.0) theta4 += 360.0;

  if (!((rot1*(360/20) + curPos1 + theta1) < motor1UB && (rot1*(360/20) + curPos1 + theta1) > motor1LB)) { //if outside the bound change directions to the correct point 
    if (theta1 > 0){
      theta1 -= 360.0; 
    }
    else{
      theta1 += 360.0;
    }
  }
  rot1 = floor((rot1*(360/20) + curPos1 + theta1)*(20.0/360.0)); //find the next rotation value

  if (!((rot2*(360/50) + curPos2 + theta2) < motor2UB && (rot2*(360/50) + curPos2 + theta2) > motor2LB)) { //if outside the bound change directions to the correct point 
    if (theta2 > 0){
      theta2 -= 360.0; 
    }
    else{
      theta2 += 360.0;
    }
  }
  rot2 = floor((rot2*(360/50) + curPos2 + theta2)*(50.0/360.0)); //find the next rotation value

  if (!((rot3*(360/20) + curPos3 + theta3) < motor3UB && (rot3*(360/20) + curPos3 + theta3) > motor3LB)) { //if outside the bound change directions to the correct point 
    if (theta3 > 0){
      theta3 -= 360.0; 
    }
    else{
      theta3 += 360.0;
    }
  }
  rot3 = floor((rot3*(360/20) + curPos3 + theta3)*(20.0/360.0)); //find the next rotation value

  if (!((rot4*(360/20) + curPos4 + theta4) < motor4UB && (rot4*(360/20) + curPos4 + theta4) > motor4LB)) { //if outside the bound change directions to the correct point 
    if (theta4 > 0){
      theta4 -= 360.0; 
    }
    else{
      theta4 += 360.0;
    }
  }
  rot4 = floor((rot4*(360/20) + curPos4 + theta4)*(20.0/360.0)); //find the next rotation value

  digitalWrite(DIR_PIN1, (theta1 >= 0) ? LOW : HIGH);
  digitalWrite(DIR_PIN2, (theta2 >= 0) ? LOW : HIGH);
  digitalWrite(DIR_PIN3, (theta3 >= 0) ? LOW : HIGH);
  digitalWrite(DIR_PIN4, (theta4 >= 0) ? LOW : HIGH);
  steps[0] = short((abs(theta1)*20.0)/1.8); 
  steps[1] = short((abs(theta2)*50.0)/1.8); 
  steps[2] = short((abs(theta3)*20.0)/1.8);
  steps[3] = short((abs(theta4)*20.0)/1.8);
  mostSteps = steps[0];
  mostStepsIndex = 0;
  for (int i=1; i<4; i++){
    if (steps[i] > mostSteps){
      mostSteps = steps[i];
      mostStepsIndex = i;
    }
  }
  frequencies[mostStepsIndex] = 500;
  timescale = mostSteps/frequencies[mostStepsIndex];

  if (timescale != 0){
    for (int i = 0; i<4; i++){
      if (i != mostStepsIndex){
        if (steps[i] != 0) {
          frequencies[i] = steps[i]/timescale;
        } else frequencies[i] = 500;
      } 
    }
    noInterrupts();
    OCR1A = calcOCRA(frequencies[0], 32);
    OCR3A = calcOCRA(frequencies[1], 32);
    OCR4A = calcOCRA(frequencies[2], 32); 
    OCR5A = calcOCRA(frequencies[3], 32);
    interrupts();
    if (steps[0] > 0) motor1Running = true;
    if (steps[1] > 0) motor2Running = true;
    if (steps[2] > 0) motor3Running = true;
    if (steps[3] > 0) motor4Running = true;
  }
  Serial.println("Motors started.");
  
  Serial.println("DESIRED");
  Serial.println(desPos1);
  Serial.println(desPos2);
  Serial.println(desPos3);
  Serial.println("CURRENT");
  Serial.println(curPos1);
  Serial.println(curPos2);
  Serial.println(curPos3);
  Serial.println("ROTATIONS");
  Serial.println(rot1);
  Serial.println(rot2);
  Serial.println(rot3);
  Serial.println("STEPS");
  Serial.println(steps[0]);
  Serial.println(steps[1]);
  Serial.println(steps[2]);
  
}

void updateDCs(){
  //Serial.println("Updating DCs");
  float newPos5;
  float newPos6;
  readEncoderDC(newPos5, newPos6, desPos5, desPos6);
  
  if(newPos5 != 0){
    passZero5 = ((abs(abs(newPos5) - abs(currPos5)) > 100) ? true : false); //If we switched signs, passZero is true
  }
  if(newPos6 != 0){
    passZero6 = ((abs(abs(newPos6) - abs(currPos6)) > 100) ? true : false);
  }
  
  if(passZero5){
    (newPos5>currPos5) ? (rot5 -= 1) : (rot5 += 1);
  }
  if(passZero6){
    (newPos6>currPos6) ? (rot6 -= 1) : (rot6 += 1);
  }
  
  currPos5 = newPos5;
  currPos6 = newPos6;
  motor5Running = true; //End conditions
  motor6Running = true;

  error5 = desPos5 - currPos5; //position error
  if (error5 > 180.0) error5 -= 360.0;  // Ensure shortest path
  if (error5 < -180.0) error5 += 360.0;
  
  if(error5>0){ //If we're rotating CW
    if(currPos5+error5 >= 360.0){ //If we pass zero
      if(rot5>1) error5 -= 360.0; //If passing zero is not ok, make the error the other way
    }
  }else{ //If we're rotating CCW
    if(currPos5+error5 <= 0.0){ //If we pass zero
      if(rot5<-1) error5 += 360.0; //If passing zero is not ok, make the error the other way
    }
  }
  
  error6 = desPos6-currPos6; //position error
  if (error6 > 180.0) error6 -= 360.0;  // Ensure shortest path
  if (error6 < -180.0) error6 += 360.0;
  
  if(error6>0.0){ //If we're rotating CW
    if(currPos6+error6 >= 360.0){ //If we will pass zero
      if(rot6>1) error6 -= 360.0;//If passing zero is not ok, make the error the other way
    }
  }else{ //If we're rotating CCW
    if(currPos6+error6 <= 0.0){ //If we pass zero
      if(rot6<-1) error6 += 360.0; //If passing zero is not ok, make the error the other way
    }
  }
  
  //End conditions
  if (abs(error5) < 8.0) {
    digitalWrite(IN1, 1);
    digitalWrite(IN2, 1);
    currPWM5 = 0;
    analogWrite(pwmPin5, abs(currPWM5));
    motor5Running = false;
  }
  if (abs(error6) < 10.0) {
    digitalWrite(IN3, 1);
    digitalWrite(IN4, 1);
    currPWM6 = 0;
    analogWrite(pwmPin6, abs(currPWM6)); //CHECK
    motor6Running = false;
  }
  //Only adjust motor 5 if it hasn't hit the target yet
  if(motor5Running){
    currPWM5 = Ki5*error5;
    if (currPWM5 >= 255) currPWM5 = 255;
    if (currPWM5 <= -255) currPWM5 = -255;
    if (currPWM5 <= 0) {
      digitalWrite(IN1, 0);
      digitalWrite(IN2, 1);
      analogWrite(pwmPin5, abs(currPWM5));
    } else {
      digitalWrite(IN1, 1);
      digitalWrite(IN2, 0);
      analogWrite(pwmPin5,abs(currPWM5));
    }
  }

  //Only adjust motor 6 if it hasn't hit the target yet
  if(motor6Running){
    currPWM6 = Ki6*error6;
    if (currPWM6 >= 255) {currPWM6 = 255;}
    if (currPWM6 <= -255) {currPWM6 = -255;}
    if (currPWM6 <= 0) {
      digitalWrite(IN3, 1);
      digitalWrite(IN4, 0);
      analogWrite(pwmPin6,abs(currPWM6));
    } else {
      digitalWrite(IN3, 0);
      digitalWrite(IN4, 1);
      analogWrite(pwmPin6,abs(currPWM6));
    }
  }
}

int calcOCRA(float freq, int prescale){
  float OCRA = ((clockFreq/(freq*prescale)) - 1);
  if(OCRA > 65535){
    OCRA = 65535;
  }
  return OCRA;
}

float readEncoderStepper(float* currentPos, int analogPin, float guess){
  float checkVal;
  switch (analogPin) {
      case A1:
        *currentPos = ((((analogRead(analogPin))/1023.0)*(360.0))/20.0) - encZeroes[0]; //
        if (*currentPos < 0) *currentPos += 18;
        checkVal = (abs(*currentPos - guess)*20.0); //MAKE SURE THAT currentPos AND guess are both between 0 and 18 when checking them, BUT THEY MUST RETURN TO THEIR ACTUAL VALUE AFTER THIS
        if (checkVal < 7.2 || checkVal > 352.8) *currentPos = guess; //compare TEMP to guess and then edit current pos
        break;
      case A2:
        *currentPos = ((((analogRead(analogPin))/1023.0)*(360.0))/50.0) - encZeroes[1]; //
        if (*currentPos < 0) *currentPos += 7.2;
        checkVal = (abs(*currentPos - guess)*50.0);
        if (checkVal < 7.2 || checkVal > 352.8) *currentPos = guess;
        break;
      case A3:
        *currentPos = ((((analogRead(analogPin))/1023.0)*(360.0))/20.0) - encZeroes[2]; //
        if (*currentPos < 0) *currentPos += 18;
        checkVal = (abs(*currentPos - guess)*20.0);
        if (checkVal < 7.2 || checkVal > 352.8) *currentPos = guess;
        break;
      case A4:
        *currentPos = ((((analogRead(analogPin))/1023.0)*(360.0))/20.0) - encZeroes[3]; //
        if (*currentPos < 0) *currentPos += 18;
        checkVal = (abs(*currentPos - guess)*20.0);
        if (checkVal < 7.2 || checkVal > 352.8) *currentPos = guess;
        break;
  }
}

void print_motor_info(){
  //print current position, desired position, error, and pwm value
  Serial.println("currPos desPos error pwm rot");
  Serial.println(String(currPos5) + " " + String(desPos5) + " " + String(error5) + " " + String(currPWM5) + " " + String(rot5));
  Serial.println(String(currPos6) + " " + String(desPos6) + " " + String(error6) + " " + String (currPWM6) + " " + String(rot6));
}

float readEncoderDC(float &num1, float &num2, float guess1, float guess2){
  float checkVal1;
  float checkVal2;
  int raw_value = analogRead(encPin5);
  num1 = (raw_value/1023.0)*(360.0);
  if (guess1 < 0) guess1 += 360;
  checkVal1 = abs(num1 - guess1);
  if (checkVal1 < 1.8 || checkVal1 > 358.2) num1 = guess1;
  if (num1 == 0.0) num1+=0.01;
  int raw_value2 = analogRead(encPin6);
  num2 = (raw_value2/1023.0)*(360.0);
  if (guess2 < 0) guess2 += 360;
  checkVal2 = abs(num2 - guess2);
  if (checkVal2 < 1.8 || checkVal2 > 358.2) num2 = guess2;
  if (num2 == 0.0) num2 += 0.01;
}