
#define A_PLUS_1   2   // PE4
#define A_MINUS_1  6   // PH3
#define B_PLUS_1   4   // PG5
#define B_MINUS_1  8   // PH5

#define A_PLUS_2   3   // PE5
#define A_MINUS_2  7   // PH4
#define B_PLUS_2   5   // PE3
#define B_MINUS_2  9   // PH6

#include <AccelStepper.h>

AccelStepper stepper1(AccelStepper::FULL4WIRE,
                      A_PLUS_1, A_MINUS_1, B_PLUS_1, B_MINUS_1);

AccelStepper stepper2(AccelStepper::FULL4WIRE,
                      A_PLUS_2, A_MINUS_2, B_PLUS_2, B_MINUS_2);

String inBuf;

void setup()
{
  Serial.begin(115200);
  while (!Serial);
  
  const float DEFAULT_MAXSPEED = 1000; 
  const float DEFAULT_ACC      = 2000;  
  
  stepper1.setMaxSpeed(DEFAULT_MAXSPEED);
  stepper2.setMaxSpeed(DEFAULT_MAXSPEED);
  stepper1.setAcceleration(DEFAULT_ACC);
  stepper2.setAcceleration(DEFAULT_ACC);

}

void loop()
{
  serialReceive();

  stepper1.run();
  stepper2.run();
}


void serialReceive()
{
  while (Serial.available())
  {
    char c = Serial.read();
    if (c == '\r') continue;   
    if (c == '\n')          
    {
      handleCommand(inBuf);
      Serial.println(F("ok")); 
      inBuf = "";               
    }
    else
    {
      inBuf += c;
    }
    
  }
}

void handleCommand(const String &cmd)
{
  long  pos1 = 0, pos2 = 0;
  float vel  = 0;


  int firstSpace  = cmd.indexOf(' ');
  int secondSpace = cmd.indexOf(' ', firstSpace + 1);
  if (firstSpace < 0 || secondSpace < 0) {
    Serial.println(F("Format error. Example: 1000 -800 600"));
    return;
  }

  pos1 = cmd.substring(0, firstSpace).toInt();
  pos2 = cmd.substring(firstSpace + 1, secondSpace).toInt();
  vel  = cmd.substring(secondSpace + 1).toFloat();
  if (vel <= 0) {
    Serial.println(F("Speed must be > 0"));
    return;
  }


  stepper1.setMaxSpeed(vel);
  stepper2.setMaxSpeed(vel);
  stepper1.setAcceleration(vel * 2);
  stepper2.setAcceleration(vel * 2);

  stepper1.moveTo(pos1);
  stepper2.moveTo(pos2);

}
