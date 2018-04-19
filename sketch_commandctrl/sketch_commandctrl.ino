/*
 * Command Interface: <CommandSet>
 * Command Set: strings for CMD and PARAMETERS
 *  Stepper Motor Set Speed:  <stepspeed [motor# 1-M1M2/2-M3M4] [speed]
 *  Stepper Motor Move:       <stepper [motor#] [steps] [F-Forward/B-Backward] [S-Single/D-Double/I-Interleave/M-Microstep]
 *  Stepper Motor Release:    <steprel [motor#]
 * Example Stepper Commands: 
<stepspeed 1 100>
<stepper 1 1000 F S>
<steprel 1>
 * 
 */

#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include <string.h>
 
const byte numChars = 100;
char receivedChars[numChars];
boolean newData = false;
char *cmd[10];
int argc = 0;

// Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 
// Or, create it with a different I2C address (say for stacking)
// Adafruit_MotorShield AFMS = Adafruit_MotorShield(0x61); 

// Connect a stepper motor with 200 steps per revolution (1.8 degree)
// to motor port #2 (M3 and M4)
Adafruit_StepperMotor *motor1 = AFMS.getStepper(200, 1);
Adafruit_StepperMotor *motor2 = AFMS.getStepper(200, 2);

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Serial.println("<Arduino is ready>");

  AFMS.begin();  // create with the default frequency 1.6KHz
  //AFMS.begin(1000);  // OR with a different frequency, say 1KHz
  motor1->setSpeed(10);
  motor2->setSpeed(10);
}

void loop() {
  readCommand(); 
  parseCommand();
}

void parseCommand() {
  if( argc > 0 ) {
     echo();
     if(strcmp(cmd[0], "stepper") == 0) {
        // step(#steps, direction, steptype) procedure. #steps is how many steps you'd like it to take. direction is either FORWARD or BACKWARD and the step type is SINGLE, DOUBLE, INTERLEAVE or MICROSTEP.
        
        String motor(cmd[1]);
        String steps(cmd[2]);

        int dir = FORWARD;
        if( strcmp(cmd[3], "B")==0) dir = BACKWARD;

        int style = SINGLE;
        if( strcmp(cmd[4], "D")==0) style = DOUBLE;
        if( strcmp(cmd[4], "I")==0) style = INTERLEAVE;
        if( strcmp(cmd[4], "M")==0) style = MICROSTEP;

        Serial.print("Step ");
        Serial.print( motor );
        Serial.print( " "  );
        Serial.print( steps );
        Serial.print( " "  );
        Serial.print( String(dir) );
        Serial.print( " "  );
        Serial.println( String(style) );

        if( motor.toInt() == 1 ) {
          motor1->step(steps.toInt(), dir, style);
        } else {
          motor2->step(steps.toInt(), dir, style);
        }
     } else if(strcmp(cmd[0], "stepspeed") == 0) {
        String motor(cmd[1]);
        String speed(cmd[2]);
      
        if( motor.toInt() == 1 ) {
          motor1->setSpeed(speed.toInt());
        } else {
          motor2->setSpeed(speed.toInt());
        }
     } else if(strcmp(cmd[0], "steprel") == 0) {
        String motor(cmd[1]);
      
        if( motor.toInt() == 1 ) {
          motor1->release();
        } else {
          motor2->release();
        }
     }
  }
  argc = 0;
  //Serial.flush();
}

void readCommand() {
  recvWithStartEndMarkers();
  if (newData == true) {
    argc = 0;  
    cmd[argc] = strtok(receivedChars, " |");
   
    // Keep printing tokens while one of the
    // delimiters present in str[].
    while (cmd[argc] != NULL)
    {
      argc ++;
      cmd[argc] = strtok(NULL, " |");
    }
    newData = false;
  }
}

void recvWithStartEndMarkers() {
    static boolean recvInProgress = false;
    static byte ndx = 0;
    char startMarker = '<';
    char endMarker = '>';
    char rc;
 
    while (Serial.available() > 0 && newData == false) {
        rc = Serial.read();

        if (recvInProgress == true) {
            if (rc != endMarker) {
                receivedChars[ndx] = rc;
                ndx++;
                if (ndx >= numChars) {
                    ndx = numChars - 1;
                }
            }
            else {
                receivedChars[ndx] = '\0'; // terminate the string
                recvInProgress = false;
                ndx = 0;
                newData = true;
            }
        }

        else if (rc == startMarker) {
            recvInProgress = true;
        }
    }
}

void echo() {
    Serial.print("Received Command: ");
    for(int i=0; i<argc; i++) {
      Serial.print("[");
      Serial.print( cmd[i] );
      Serial.print( "]" );
    }
    Serial.println("");
}
