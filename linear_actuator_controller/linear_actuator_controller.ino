#include <AccelStepper.h>

// Define a stepper and the pins it will use
AccelStepper stepper(1, 8, 9);
const float initial_max_speed     = 900.0;      // In RPM
const float initial_acceleration  = 5000.0;      // In RPM/s
const int steps_per_revolution  = 400;      // Before altering this value, check the controller and alter the value there

int upper_limit = 5600;
int lower_limit = 0;

void setup()
{  
  stepper.setMaxSpeed(steps_per_revolution * initial_max_speed / 60.0);
  stepper.setAcceleration(steps_per_revolution * initial_acceleration / 60.0);
  stepper.moveTo(0);
  
  Serial.begin(9600);
}

void loop()
{
  stepper.run();
  
  if (Serial.available() > 0)
  {
    String request = Serial.readStringUntil('\n');
    String argument = request.substring(2);
    switch (request.charAt(0))
    {
      case 's':
        Serial.print("Max speed set to: ");
        Serial.print(argument.toInt());
        Serial.print(" RPM (");
        Serial.print(steps_per_revolution * argument.toInt() / 60.0);
        Serial.println(" steps/s)");
        stepper.setMaxSpeed(steps_per_revolution * argument.toInt() / 60.0);
        break;
      case 'a':
        Serial.print("Acceleration set to: ");
        Serial.print(argument.toInt());
        Serial.print(" RPM/s (");
        Serial.print(steps_per_revolution * argument.toInt() / 60.0);
        Serial.println(" steps/s^2)");
        stepper.setAcceleration(steps_per_revolution * argument.toInt() / 60.0);
        break;
      case 'l':
        if (argument.toInt() < upper_limit)
        {
          Serial.print("Updating lower limit to ");
          Serial.print(argument.toInt());
          lower_limit = argument.toInt();
        }
        else
        {
          Serial.print("Lower limit needs to be below the upper limit");
        }
        break;
      case 'u':
        if (argument.toInt() > lower_limit)
        {
          Serial.print("Updating lower limit to ");
          Serial.print(argument.toInt());
          upper_limit = argument.toInt();
        }
        else
        {
          Serial.print("Upper limit needs to be above the lower limit");
        }
      case 'm':
        if (argument.toInt() >= lower_limit && argument.toInt() <= upper_limit) 
        {
          Serial.print("Moving to: ");
          Serial.println(argument);
          stepper.moveTo(argument.toInt());    
        } 
        else if (argument.toInt() < upper_limit) 
        {
          Serial.print("Setpoint below lower limit (");
          Serial.print(lower_limit);
          Serial.println("). Moving to lower limit...");
          stepper.moveTo(lower_limit);    
        }
        else if (argument.toInt() > lower_limit) 
        {
          Serial.print("Setpoint above upper limit (");
          Serial.print(upper_limit);
          Serial.println("). Moving to upper limit...");
          stepper.moveTo(upper_limit);    
        }
        
        break;
      case 'p':
        Serial.println("Stopping movement...");
        stepper.stop();
        break;
      case 'g':
        switch (argument.charAt(0)) 
        {
          case 's':
            Serial.print("Current max speed: ");
            Serial.print(stepper.maxSpeed() * 60.0 / steps_per_revolution);
            Serial.print(" RPM (");
            Serial.print(stepper.maxSpeed());
            Serial.println(" steps/s)");
            break;
           case 'a':
            Serial.print("Current acceleration: ");
            Serial.print(stepper.acceleration() * 60.0 / steps_per_revolution);
            Serial.print(" RPM/s (");
            Serial.print(stepper.acceleration());
            Serial.println(" steps/s^2");
            break;
        }
        break;
    }
  }
}
