#include <AccelStepper.h>

// Define a stepper and the pins it will use
AccelStepper stepper(1, 8, 9);
const float initial_max_speed     = 500.0;      // In RPM
const float initial_acceleration  = 2000.0;      // In RPM/s
const int steps_per_revolution  = 400;      // Before altering this value, check the controller and alter the value there

int upper_limit = 800;
int lower_limit = -800;

const byte upper_endstop_pin = 2;
const byte lower_endstop_pin = 3;
int endstops_identified = 0;
bool upper_endstop_identified = false;
bool lower_endstop_identified = false;
bool calibration_started = false;
bool unstuck_procedure_started = false; // Just for testing when motor doesn't work
float target_position;
unsigned long initial_calibration_time;

void setup()
{
  stepper.setMaxSpeed(steps_per_revolution * initial_max_speed / 60.0);
  stepper.setAcceleration(steps_per_revolution * initial_acceleration / 60.0);
  stepper.moveTo(0);

  pinMode(upper_endstop_pin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(upper_endstop_pin), upper_endstop_interrupt, HIGH);
  pinMode(lower_endstop_pin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(lower_endstop_pin), lower_endstop_interrupt, HIGH);

  Serial.begin(115200);
}

void loop()
{
  stepper.run();
  
  if (Serial.available() > 0 && !calibration_started)
  {
    String request = Serial.readStringUntil('\n');
    String argument = request.substring(2);
    Serial.println(request);
    switch (request.charAt(0))
    {
      case 't':
        unstuck_procedure_started = true;
        initial_calibration_time = millis();
        break;
      case 'i':
        Serial.println("Starting calibration, wait for it to finish...");
        upper_endstop_identified = false;
        lower_endstop_identified = false;
        calibration_started = true;
        initial_calibration_time = millis();
        break;
      case 'c':
        for (int i = 0; i < 10; i++)
        {
          delay(100);
          Serial.print(argument);
          Serial.print('\n');
        }
        break;
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
      case 'm':
        target_position = lower_limit + (upper_limit - lower_limit) * argument.toFloat();
      
        if (target_position >= lower_limit && target_position <= upper_limit)
        {
          Serial.print("Moving to: ");
          Serial.println(target_position);
          stepper.moveTo(target_position);
        }
        else {
          Serial.println("Target should be between 0 and 1");
        }

        break;
      case 'p':
        Serial.println("Stopping movement...");
        stepper.stop();
        break;
     case 'y':
        stepper.move(100);
        Serial.println("moving 100");
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
  
  if (unstuck_procedure_started) {
    if (millis() - initial_calibration_time < 2000) {
      stepper.move(10);
    }
    else if (millis() - initial_calibration_time < 4000) {
      stepper.move(-10);
    }
    else if (millis() - initial_calibration_time > 4000) {
      unstuck_procedure_started = false;
    }
  }
  if (calibration_started) {
    if (millis() - initial_calibration_time > 20000) {
      calibration_started = false;
      endstops_identified = 0;
      stepper.moveTo(0);
      Serial.println("201: Calibration failed");
    }
    
    if (!upper_endstop_identified) {
      stepper.move(10);
    } else if (upper_endstop_identified && !lower_endstop_identified) {
      stepper.move(-10);
    } else if (upper_endstop_identified && lower_endstop_identified) {
      calibration_started = false;
      endstops_identified = 0;
      stepper.moveTo(int((lower_limit + upper_limit)/2));
      Serial.println("201: Calibration Finished");
    }
    delay(1);
  }
}

void upper_endstop_interrupt() {
  
  if (!upper_endstop_identified) {
    stepper.stop();
    upper_endstop_identified = true;
    Serial.print("Upper endstop: ");
    upper_limit = stepper.currentPosition() - 20;
    Serial.println(upper_limit);
  }
}

void lower_endstop_interrupt() {
  if (!lower_endstop_identified) {
    stepper.stop();
    lower_endstop_identified = true;
    Serial.print("Lower endstop: ");
    lower_limit = stepper.currentPosition() + 20;
    Serial.println(lower_limit);
    
  }
}
