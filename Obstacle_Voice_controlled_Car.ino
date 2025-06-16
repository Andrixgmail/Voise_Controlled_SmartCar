/* Andrea Mahlo
 * Voice Controlled Car
*/
#include <Servo.h>
#include <AFMotor.h>
#define Echo A0
#define Trig A1
#define motor 10
#define Speed 120
#define spoint 103
char value;
String voice;
int distance;
int Left;
int Right;
int L = 0;
int R = 0;
int L1 = 0;
int R1 = 0;

int F = 0;
int fw;

int led = 13;
int led2 = 9;

Servo servo;
AF_DCMotor M1(1);
AF_DCMotor M2(2);
AF_DCMotor M3(3);
AF_DCMotor M4(4);
void setup() {
  Serial.begin(9600);
  pinMode(Trig, OUTPUT);
  pinMode(Echo, INPUT);
  servo.attach(motor);
  M1.setSpeed(Speed);
  M2.setSpeed(Speed);
  M3.setSpeed(Speed);
  M4.setSpeed(Speed);
  
  pinMode(led,OUTPUT); // led pin  
  pinMode(led2,OUTPUT); // led pin  
}
void loop() {
  //Obstacle();
  //VoiceControlled();
  Bluetoothcontrol();
  voicecontrol();
  
}
void Bluetoothcontrol() {
  if (Serial.available() > 0) {
    value = Serial.read();
    Serial.println(value);
  }
  if (value == 'F') {
    forward();
    offLED();
  } else if (value == 'B') {
    backward();
    onLED(); //LED ON
  } else if (value == 'L') {
    left();
    offLED();
  } else if (value == 'R') {
    right();
    offLED();
  } else if (value == 'S') {
    Stop();
    onLED(); //LED ON
    offLED();
  }
}
void Obstacle() {
  distance = ultrasonic();
  if (distance <= 12) {
    Stop();
    backward();
    delay(100);
    Stop();
    L = leftsee();
    servo.write(spoint);
    delay(800);
    R = rightsee();
    servo.write(spoint);
    if (L < R) {
      right();
      delay(500);
      Stop();
      delay(200);
    } else if (L > R) {
      left();
      delay(500);
      Stop();
      delay(200);
    }
  } else {
    forward();
  }
}
void voicecontrol() {
  if (Serial.available() > 0) {
    value = Serial.read();
    Serial.println(value);
    if (value == '+') {
      if(R >=10)
      {
        forward();
      }
      else if (R < 10) {
        Stop();
      }
    } else if (value == '=') {
      backward();
    } else if (value == '<') {
      L = leftsee();
      servo.write(spoint);
      if (L >= 10 ) {
        left();
        delay(500);
        Stop();
      } else if (L < 10) {
        Stop();
      }
    } else if (value == '>') {
      R = rightsee();
      servo.write(spoint);
      if (R >= 10 ) {
        right();
        delay(500);
        Stop();
      } else if (R < 10) {
        Stop();
      }
    } else if (value == '*') {
      Stop();
    }
  }
}
// Ultrasonic sensor distance reading function
int ultrasonic() {
  digitalWrite(Trig, LOW);
  delayMicroseconds(4);
  digitalWrite(Trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(Trig, LOW);
  long t = pulseIn(Echo, HIGH);
  long cm = t / 29 / 2; //time convert distance
  return cm;
}
void forward() {
  M1.run(FORWARD);
  M4.run(FORWARD);
}
void backward() {
  M1.run(BACKWARD);
  M4.run(BACKWARD);
}
void right() {
  M1.run(BACKWARD);
  M4.run(FORWARD);
}
void left() {
  M1.run(FORWARD);
  M4.run(BACKWARD);
}
void Stop() {
  M1.run(RELEASE);
  M4.run(RELEASE);
}
int rightsee() {
  servo.write(20);
  delay(800);
  Left = ultrasonic();
  return Left;
}
int leftsee() {
  servo.write(180);
  delay(800);
  Right = ultrasonic();
  return Right;
}
int forwardsee()
{
  fw = ultrasonic();
}
void onLED()
{
 digitalWrite(led,HIGH); // led ON
 digitalWrite(led2,HIGH);
}
void offLED()
{
 digitalWrite(led,LOW); // led ON
 digitalWrite(led2,LOW);
}
void ldrLight()
{
  int s1=analogRead(A5); // LDR Sensor output pin connected  
 Serial.println(s1);  
 delay(50);  
 if(s1 > 300 )  
 {  
 digitalWrite(led,HIGH); // led ON
 digitalWrite(led2,HIGH);  
 }  
 else if(s1 < 300)
 {  
 digitalWrite(led,LOW); // led OFF  
 digitalWrite(led2,LOW);
 }  
}
void VoiceControlled()
{
  while(Serial.available())
  {
    delay(3);
    char c = Serial.read();
    voice+=c;
  }
   if (voice.length() > 0) {
    //voice = Serial.read();
    Serial.println(voice);
    if (voice == "forward") {
      if(R >=10)
      {
        forward();
      }
      else if (R < 10) {
        Stop();
      }
    } else if (voice == "backward") {
      backward();
    } else if (voice == "left") {
      L = leftsee();
      servo.write(spoint);
      if (L >= 10 ) {
        left();
        delay(500);
        Stop();
      } else if (L < 10) {
        Stop();
      }
    } else if (voice == "right") {
      R = rightsee();
      servo.write(spoint);
      if (R >= 10 ) {
        right();
        delay(500);
        Stop();
      } else if (R < 10) {
        Stop();
      }
    } else if (voice == "stop") {
      Stop();
    }
  }
}
