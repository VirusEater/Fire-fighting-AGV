#include <Arduino.h>
#include <Servo.h>
Servo myServo;
// Define motor pins
#define PWMA 12    //Motor A PWM
#define DIRA1 34
#define DIRA2 35  //Motor A Direction
#define PWMB 8    //Motor B PWM
#define DIRB1 37
#define DIRB2 36  //Motor B Direction
#define PWMC 6   //Motor C PWM
#define DIRC1 43
#define DIRC2 42  //Motor C Direction
#define PWMD 5    //Motor D PWM
#define DIRD1 A4  //26  
#define DIRD2 A5  //27  //Motor D Direction

const int flameSensorPin = A0; // Analog pin connected to the Flame Sensor module
const int flameIndicatorPin = 47; // Digital pin connected to the LED (optional)

int pinBuzzer = 48; 

int Fan1_pin1 = 46; //PL3
int Fan1_pin2 = 45; //PL4
int Fan2_pin1 = 29; // PH3 黄色
int Fan2_pin2 = 28; //PH4 红色

#define MOTORA_FORWARD(pwm)    do{digitalWrite(DIRA1,LOW); digitalWrite(DIRA2,HIGH);analogWrite(PWMA,pwm);}while(0)
#define MOTORA_STOP(x)         do{digitalWrite(DIRA1,LOW); digitalWrite(DIRA2,LOW); analogWrite(PWMA,0);}while(0)
#define MOTORA_BACKOFF(pwm)    do{digitalWrite(DIRA1,HIGH);digitalWrite(DIRA2,LOW); analogWrite(PWMA,pwm);}while(0)

#define MOTORB_FORWARD(pwm)    do{digitalWrite(DIRB1,LOW); digitalWrite(DIRB2,HIGH);analogWrite(PWMB,pwm);}while(0)
#define MOTORB_STOP(x)         do{digitalWrite(DIRB1,LOW); digitalWrite(DIRB2,LOW); analogWrite(PWMB,0);}while(0)
#define MOTORB_BACKOFF(pwm)    do{digitalWrite(DIRB1,HIGH);digitalWrite(DIRB2,LOW); analogWrite(PWMB,pwm);}while(0)

#define MOTORC_FORWARD(pwm)    do{digitalWrite(DIRC1,LOW); digitalWrite(DIRC2,HIGH);analogWrite(PWMC,pwm);}while(0)
#define MOTORC_STOP(x)         do{digitalWrite(DIRC1,LOW); digitalWrite(DIRC2,LOW); analogWrite(PWMC,0);}while(0)
#define MOTORC_BACKOFF(pwm)    do{digitalWrite(DIRC1,HIGH);digitalWrite(DIRC2,LOW); analogWrite(PWMC,pwm);}while(0)

#define MOTORD_FORWARD(pwm)    do{digitalWrite(DIRD1,LOW); digitalWrite(DIRD2,HIGH);analogWrite(PWMD,pwm);}while(0)
#define MOTORD_STOP(x)         do{digitalWrite(DIRD1,LOW); digitalWrite(DIRD2,LOW); analogWrite(PWMD,0);}while(0)
#define MOTORD_BACKOFF(pwm)    do{digitalWrite(DIRD1,HIGH);digitalWrite(DIRD2,LOW); analogWrite(PWMD,pwm);}while(0)

#define SERIAL  Serial

//PWM Definition
#define MAX_PWM   2000
#define MIN_PWM   300

int Motor_PWM = 19000;


//    ↑A-----B↑
//     |  ↑  |
//     |  |  |
//    ↑C-----D↑
void BACK()
{
  MOTORA_BACKOFF(Motor_PWM); 
  MOTORB_FORWARD(Motor_PWM);
  MOTORC_BACKOFF(Motor_PWM); 
  MOTORD_FORWARD(Motor_PWM);
}

//    ↓A-----B↓
//     |  |  |
//     |  ↓  |
//    ↓C-----D↓
void ADVANCE()
{
  MOTORA_FORWARD(Motor_PWM); 
  MOTORB_BACKOFF(Motor_PWM);
  MOTORC_FORWARD(Motor_PWM); 
  MOTORD_BACKOFF(Motor_PWM);
}

//    =A-----B↑
//     |   ↖ |
//     | ↖   |
//    ↑C-----D=
void LEFT_1() // 右后
{
  MOTORA_STOP(Motor_PWM); 
  MOTORB_FORWARD(Motor_PWM);
  MOTORC_BACKOFF(Motor_PWM); 
  MOTORD_STOP(Motor_PWM);
}

//    ↓A-----B↑
//     |  ←  |
//     |  ←  |
//    ↑C-----D↓
void RIGHT_2() 
{
  MOTORA_FORWARD(1900); 
  MOTORB_FORWARD(1900);
  MOTORC_BACKOFF(1900); 
  MOTORD_BACKOFF(1900);
}
//    ↓A-----B=
//     | ↙   |
//     |   ↙ |
//    =C-----D↓
void LEFT_3() 
{
  MOTORA_FORWARD(Motor_PWM); 
  MOTORB_STOP(Motor_PWM);
  MOTORC_STOP(Motor_PWM); 
  MOTORD_BACKOFF(Motor_PWM);
}
//    ↑A-----B=
//     | ↗   |
//     |   ↗ |
//    =C-----D↑
void RIGHT_1() 
{
  MOTORA_BACKOFF(Motor_PWM); 
  MOTORB_STOP(Motor_PWM);
  MOTORC_STOP(Motor_PWM); 
  MOTORD_FORWARD(Motor_PWM);
}
//    ↑A-----B↓
//     |  →  |
//     |  →  |
//    ↓C-----D↑
void LEFT_2()
{
  MOTORA_BACKOFF(1900); 
  MOTORB_BACKOFF(1900);
  MOTORC_FORWARD(1900); 
  MOTORD_FORWARD(1900);
}
//    =A-----B↓
//     |   ↘ |
//     | ↘   |
//    ↓C-----D=
void RIGHT_3()
{
  MOTORA_STOP(Motor_PWM); 
  MOTORB_BACKOFF(Motor_PWM);
  MOTORC_FORWARD(Motor_PWM); 
  MOTORD_STOP(Motor_PWM);
}

//    ↑A-----B↓
//     | ↗ ↘ |
//     | ↖ ↙ |
//    ↑C-----D↓
void rotate_1()  //tate_1(uint8_t pwm_A,uint8_t pwm_B,uint8_t pwm_C,uint8_t pwm_D)
{
  MOTORA_BACKOFF(Motor_PWM); 
  MOTORB_BACKOFF(Motor_PWM);
  MOTORC_BACKOFF(Motor_PWM); 
  MOTORD_BACKOFF(Motor_PWM);
}

//    ↓A-----B↑
//     | ↙ ↖ |
//     | ↘ ↗ |
//    ↓C-----D↑
void rotate_2()  // rotate_2(uint8_t pwm_A,uint8_t pwm_B,uint8_t pwm_C,uint8_t pwm_D)
{
  MOTORA_FORWARD(Motor_PWM);
  MOTORB_FORWARD(Motor_PWM);
  MOTORC_FORWARD(Motor_PWM);
  MOTORD_FORWARD(Motor_PWM);
}
//    =A-----B=
//     |  =  |
//     |  =  |
//    =C-----D=
void STOP()
{
  MOTORA_STOP(Motor_PWM);
  MOTORB_STOP(Motor_PWM);
  MOTORC_STOP(Motor_PWM);
  MOTORD_STOP(Motor_PWM);
}


#define echoPin_L 10
#define echoPin_R 4
#define trigPin_L 11
#define trigPin_R 13
#define obst_thresh 20
long distance_cm_L, distance_cm_R;
int range = 150;


long measure_distance_L()
{
  long duration_L;
  long distance_cm_L;

  digitalWrite(trigPin_L, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin_L, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin_L, LOW);
  duration_L = pulseIn(echoPin_L, HIGH);

  distance_cm_L = (duration_L / 2.0) / 29.1;

  return distance_cm_L;

}

long measure_distance_R()
{
  long duration_R;
  long distance_cm_R;

  digitalWrite(trigPin_R, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin_R, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin_R, LOW);
  duration_R = pulseIn(echoPin_R, HIGH);

  distance_cm_R = (duration_R / 2.0) / 29.1;

  return distance_cm_R;

}

void walk()
{
  long distance_cm_L, distance_cm_R;
  unsigned long last_turn;
  int time_thresh = 3000; //max time moving in a straight line
  distance_cm_L = measure_distance_L();
  distance_cm_R = measure_distance_R();
  Serial.print("left distance: ");
  Serial.println(distance_cm_L);
  Serial.print("right distance: ");
  Serial.println(distance_cm_R);
  Serial.println(' ');
  //check if obstacle
  if ((distance_cm_L <= obst_thresh) or (distance_cm_R <= obst_thresh))
  {
    int num = random(3900);
    int dir = random(0, 2);
    Serial.println(dir);
    STOP();
    if (dir == 1){
      rotate_2();
      delay(num);
    }
    else if (dir==0){
      rotate_1();
      delay(num);
    }
    
    last_turn = millis();//reset time since last turn
    
  }
  else{
    ADVANCE();
    delay(200);
  }
  STOP();
  delay(1000);
}

int detectFire()
{
  int flameValue = analogRead(flameSensorPin); // Read the analog value from the Flame Sensor
  
  // Display the flame sensor value on the Serial Monitor
  Serial.print("Flame Sensor Value: ");
  Serial.println(flameValue);

  // Set a threshold value for flame detection (adjust according to your environment)
  int threshold = 900;

  if (flameValue < threshold) {
    // Flame detected! Add your desired action here.
    // For example, turn on an LED as a visual indication of the flame.
    digitalWrite(flameIndicatorPin, LOW);
    return 1;
  } else {
    // No flame detected, turn off the LED (optional)
    digitalWrite(flameIndicatorPin, HIGH);
    return 0;
  }

}

void buzzerON()
{
   digitalWrite(pinBuzzer,HIGH);
}

void buzzerOFF()
{  
   digitalWrite(pinBuzzer,LOW);
}

void fanON() {
  digitalWrite(Fan1_pin1,HIGH);
  digitalWrite(Fan1_pin2,LOW);
  digitalWrite(Fan2_pin1,HIGH);
  digitalWrite(Fan2_pin2,LOW);
  
}

void fanOFF() {
  digitalWrite(Fan1_pin1,LOW);
  digitalWrite(Fan1_pin2,LOW);
  digitalWrite(Fan2_pin1,LOW);
  digitalWrite(Fan2_pin2,LOW);
  
}

void water() {
  myServo.write(0); // Press down
  delay(500);
  myServo.write(90); // Move away from the button
  delay(500);
}


void setup() {
  // put your setup code here, to run once:
  SERIAL.begin(115200); // USB serial setup
  SERIAL.println("Start");
  STOP(); // Stop the robot
  //setup ultrasonic sensors
  pinMode(echoPin_L, INPUT);
  pinMode(echoPin_R, INPUT);
  pinMode(trigPin_L, OUTPUT);
  pinMode(trigPin_R, OUTPUT);

  pinMode(flameSensorPin, INPUT); // Set the Flame Sensor pin as INPUT
  pinMode(flameIndicatorPin, OUTPUT); // Set the LED pin as OUTPUT (optional)

  pinMode(pinBuzzer, OUTPUT); //设置pinBuzzer脚为输出状态

  pinMode(Fan1_pin1,OUTPUT);
  pinMode(Fan1_pin2,OUTPUT);
  pinMode(Fan2_pin1,OUTPUT);
  pinMode(Fan2_pin2,OUTPUT);
  
  myServo.attach(44);
}

void loop() {
  // put your main code here, to run repeatedly:
  int fire=detectFire();
  if (fire == 1){
    water();
    while (fire==1){
      Serial.println("Dangerous!");
      buzzerON();
      fanON();
      delay(1000);
      fire=detectFire();
    }
    water(); 
  }
  else if (fire == 0){
    Serial.println("Safe");
    buzzerOFF();
    fanOFF();
    walk();
    
  }
  
  
}
