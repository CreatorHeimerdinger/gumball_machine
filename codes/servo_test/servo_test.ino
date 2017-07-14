#include <Servo.h>

#define KEY A5
#define PIN_SERVO 7

#define POS_WAIT 30
#define POS_WORK 120

Servo myservo;

void setup() 
{
  pinMode(KEY,INPUT_PULLUP);
  myservo.attach(PIN_SERVO);
  // put your setup code here, to run once:

}

void loop()
{
  if (digitalRead(KEY)==0) myservo.write(POS_WORK);
  else myservo.write(POS_WAIT);
  // put your main code here, to run repeatedly:
}
