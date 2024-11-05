#include <Arduino.h>
#include <Bounce2.h>
Bounce motorOn = Bounce();
Bounce remoteMotorOn = Bounce();
Bounce remoteEnable = Bounce();
Bounce gearSw1 = Bounce();
Bounce gearSw2 = Bounce();
Bounce direction = Bounce();

#define THROTTLE_PIN 3
#define START_PIN 4
#define BRAKE_PIN 5
#define REMOTE_RELAY_OUTPUT 6
#define GEAR_PIN1 7
#define GEAR_PIN2 8
#define REMOTE_RELAY 9
#define DIRECTION_PIN 10
#define DIRECTION_OUTPUT 11
#define REMOTE_ENABLE_PIN 12

#define FORWARD 0
#define BACKWARD 1

// put function declarations here:
void RampPwmVoltageUp(int currentPwm, int targetPwm);
void RampPwmVoltageDown(int currentPwm, int targetPwm);
int parseGearPwmLevel(int switch1, int switch2);
int gear = 0;
int gearPwmLevel = 0;
int motorState = 0;
int rampSpeed = 10;
int directionState = 0;
int allowStart=0;
int remoteEnabled=0;

int initialStartButtonPosition = 0;
void setup()
{

  Serial.begin(9600);
  pinMode(THROTTLE_PIN, OUTPUT);
  pinMode(REMOTE_RELAY, OUTPUT);
  pinMode(DIRECTION_OUTPUT, OUTPUT);
  // TCCR2B = (TCCR2B & B11111000) | B00000001;               // pin 3 and 11 PWM frequency
  remoteEnable.attach(REMOTE_ENABLE_PIN, INPUT_PULLUP);  
  motorOn.attach(START_PIN, INPUT_PULLUP); // USE INTERNAL PULL-UP
  remoteMotorOn.attach(REMOTE_RELAY_OUTPUT, INPUT_PULLUP); // USE INTERNAL PULL-UP
  gearSw1.attach(GEAR_PIN1, INPUT_PULLUP);                 // USE INTERNAL PULL-UP
  gearSw2.attach(GEAR_PIN2, INPUT_PULLUP);                 // USE INTERNAL PULL-UP
  direction.attach(DIRECTION_PIN, INPUT_PULLUP);           // USE INTERNAL PULL-UP

  // DEBOUNCE INTERVAL IN MILLISECONDS
  remoteEnable.interval(5);  // interval in ms
  motorOn.interval(5);       // interval in ms
  remoteMotorOn.interval(5); // interval in ms
  gearSw1.interval(5);       // interval in ms
  gearSw2.interval(5);       // interval in ms
  direction.interval(5);     // interval in ms
  // put your setup code here, to run once:
  digitalWrite(BRAKE_PIN, LOW);
  digitalWrite(REMOTE_RELAY, HIGH);
  // gearPwmLevel = parseGearPwmLevel(digitalRead(GEAR_PIN1), digitalRead(GEAR_PIN2));
  gearPwmLevel = 0;
  initialStartButtonPosition = digitalRead(START_PIN);
  if (initialStartButtonPosition==HIGH)
  {
    allowStart=1;
  }
  directionState = digitalRead(DIRECTION_PIN);
  digitalWrite(DIRECTION_OUTPUT, !directionState);
  Serial.print("direction: ");
  Serial.println(directionState);
  Serial.print("powerlevel: ");
  Serial.println(gearPwmLevel);
  for (int i = 0; i < 100; i++)
  {
    Serial.println(i);
    analogWrite(THROTTLE_PIN, i);
    delay(10);
  }

  delay(100);
  analogWrite(THROTTLE_PIN, 0);
  Serial.println("braking");
  digitalWrite(BRAKE_PIN, HIGH);
//  digitalWrite(REMOTE_RELAY, LOW);
}

void loop()
{

  // put your main code here, to run repeatedly:
  motorOn.update();
  remoteMotorOn.update();
  gearSw1.update();
  gearSw2.update();
  direction.update();
  remoteEnable.update();

  if (gearSw1.changed() || gearSw2.changed())
  {
    int tempGearPwmLevel;
    tempGearPwmLevel = parseGearPwmLevel(gearSw1.read(), gearSw2.read());
    if (motorOn.read() == LOW || remoteMotorOn.read() == LOW)
    {
      Serial.println("motor on");
      if (tempGearPwmLevel > gearPwmLevel)
      {
        RampPwmVoltageUp(gearPwmLevel, tempGearPwmLevel);
      }
      if (tempGearPwmLevel < gearPwmLevel)
      {
        RampPwmVoltageDown(gearPwmLevel, tempGearPwmLevel);
      }
    }
    gearPwmLevel = tempGearPwmLevel;
    Serial.print("gearPwmLevel");
    Serial.println(gearPwmLevel);
  }
  // <Bounce>.changed() RETURNS true IF THE STATE CHANGED (FROM HIGH TO LOW OR LOW TO HIGH)
  if (motorOn.changed())
  {
    
    // THE STATE OF THE INPUT CHANGED
    // GET THE STATE
    int deboucedInput = motorOn.read();

    // IF THE CHANGED VALUE IS LOW
    if (deboucedInput == LOW && allowStart && remoteEnable.read()==HIGH)
    {

      digitalWrite(BRAKE_PIN, LOW);

      gearPwmLevel = parseGearPwmLevel(gearSw1.read(), gearSw2.read());
      Serial.print("pwmlewel: ");
      Serial.println(gearPwmLevel);
      RampPwmVoltageUp(0, gearPwmLevel);

   
    }
    if (deboucedInput == LOW && !allowStart)
    {
      
      Serial.println("not starting motor, start button was low");
    }
    if (deboucedInput == HIGH)
    {
      allowStart=1;
      digitalWrite(BRAKE_PIN, HIGH);
      analogWrite(THROTTLE_PIN, 0);
      gearPwmLevel = 0;
      Serial.println("Stopping motor, braking");
    }
  }
    if (remoteEnable.changed())
  {
        int deboucedInput = remoteEnable.read();

    // IF THE CHANGED VALUE IS LOW
    if (deboucedInput == LOW)
    {
      Serial.println("remote enabled");
      digitalWrite(REMOTE_RELAY, LOW);
    }
        if (deboucedInput == HIGH)
    {
      Serial.println("remote disabled");
      digitalWrite(REMOTE_RELAY, HIGH);
    }
  }
  if (remoteMotorOn.changed())
  {
    // THE STATE OF THE INPUT CHANGED
    // GET THE STATE
    int deboucedInput = remoteMotorOn.read();
    // IF THE CHANGED VALUE IS LOW
    if (deboucedInput == LOW)
    {
      remoteEnabled=1;
      digitalWrite(BRAKE_PIN, LOW);
      gearPwmLevel = parseGearPwmLevel(gearSw1.read(), gearSw2.read());
      Serial.print("pwmlewel: ");
      Serial.println(gearPwmLevel);
      RampPwmVoltageUp(0, gearPwmLevel);
    }
    if (deboucedInput == HIGH)
    {
      digitalWrite(BRAKE_PIN, HIGH);
      analogWrite(THROTTLE_PIN, 0);
      gearPwmLevel = 0;
      Serial.println("Stopping motor, braking");
    }
  }
  if (direction.changed())
  {
    if (motorOn.read() == LOW && !remoteEnabled)
    {
      Serial.println("direction changed");
      digitalWrite(BRAKE_PIN, HIGH);
      analogWrite(THROTTLE_PIN, 0);
      //delay(1500);
      digitalWrite(BRAKE_PIN, LOW);
      directionState = !directionState;
      digitalWrite(DIRECTION_OUTPUT, !directionState);
      //delay(500);
      //RampPwmVoltageUp(0, gearPwmLevel);
   
    }
       if (remoteMotorOn.read() == LOW)
    {
      Serial.println("direction changed remotely");
      digitalWrite(BRAKE_PIN, HIGH);
      analogWrite(THROTTLE_PIN, 0);
      //delay(1500);
      digitalWrite(BRAKE_PIN, LOW);
      directionState = !directionState;
      digitalWrite(DIRECTION_OUTPUT, !directionState);
      //delay(500);
      //RampPwmVoltageUp(0, gearPwmLevel);
   
    }
    else
    {
      directionState = !directionState;
      digitalWrite(DIRECTION_OUTPUT, !directionState);
    }
   
    // digitalWrite(REMOTE_RELAY, HIGH);     //shut the power for remote relay to reset the relay status
    // delay(500);
    // digitalWrite(REMOTE_RELAY, LOW);
  }
}

// put function definitions here:
void RampPwmVoltageUp(int currentPwm, int targetPwm)
{
  // ramp up
  for (int i = currentPwm; i < targetPwm; i++)
  {
    Serial.println(i);
    analogWrite(THROTTLE_PIN, i);
    delay(10);
  }
  Serial.println("Ramp up done");
}
void RampPwmVoltageDown(int currentPwm, int targetPwm)
{
  // ramp up
  for (int i = currentPwm; i > targetPwm; i--)
  {
    Serial.println(i);
    analogWrite(THROTTLE_PIN, i);
    delay(10);
  }
  Serial.println("Ramp up done");
}
int parseGearPwmLevel(int switch1, int switch2)
{

  if (switch1 == HIGH && switch2 == HIGH)
  {
    return 180;
  }
  if (switch1 == LOW && switch2 == HIGH)
  {
    return 150;
  }
  if (switch1 == HIGH && switch2 == LOW)
  {
    return 220;
  }
  return -1;
}
