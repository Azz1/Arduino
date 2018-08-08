/*
 * Command Interface: <CommandSet>
 * Command Set: strings for CMD and PARAMETERS
 *  Stepper Motor Set Speed:  <stepspeed [motor# 1-M1M2/2-M3M4] [speed]>
 *  Stepper Motor Move:       <stepper [motor#] [steps] [F-Forward/B-Backward] [S-Single/D-Double/I-Interleave/M-Microstep]>
 *  Stepper Motor Release:    <steprel [motor#]>
 *  Enable/Disable Touch Sensor:<touch 0-disable 1-enable>
 * Example Stepper Commands: 
<stepspeed 1 100>
<stepper 1 1000 F S>
<steprel 1>
 * 
 */
#include <Adafruit_MotorShield.h>
#include <Wire.h>
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
Adafruit_StepperMotor *motor1 = AFMS.getStepper(1, 1);  // set steps to 1 for Microstep only for speed control: 
                                                            //    speed 1-10 60s/r, 20 36s/r, 40 50s/r, 50 43s/r, 100 28s/r, 200 20s/r, max 13s/r
Adafruit_StepperMotor *motor2 = AFMS.getStepper(200, 2);

boolean enableTouchSensor = false;
const int M1FTouch = 2;
const int M1BTouch = 3;
const int M2FTouch = 4;
const int M2BTouch = 5;
const int SEG_STEPS = 10;

void setup() {
  // put your setup code here, to run once:
  pinMode(M1FTouch, INPUT);
  pinMode(M1BTouch, INPUT);
  pinMode(M2FTouch, INPUT);
  pinMode(M2BTouch, INPUT);
  
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

        int segs = steps.toInt() / SEG_STEPS;
        int remsteps = steps.toInt() % SEG_STEPS;

        for(int i=0; i<segs; i++) {
          if( motor.toInt() == 1 ) {
            int state = checkState(1, dir);
            if(state != HIGH)
              motor1->step(SEG_STEPS, dir, style);
            else
              break;
          } else {
            int state = checkState(2, dir);
            if(state != HIGH)
              motor2->step(SEG_STEPS, dir, style);
            else
              break;
          }
        }
        if( motor.toInt() == 1 ) {
          int state = checkState(1, dir);
          if(state != HIGH)
            motor1->step(remsteps, dir, style);
        } else {
          int state = checkState(2, dir);
          if(state != HIGH)
            motor2->step(remsteps, dir, style);
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
     } else if(strcmp(cmd[0], "touch") == 0) {
        String val(cmd[1]);
      
        if( val.toInt() == 1 ) {
          enableTouchSensor = true;
        } else {
          enableTouchSensor = false;
        }
     }
  }
  argc = 0;
  //Serial.flush();
}

int checkState(int motor, int dir) 
{
  int state = LOW;
  if(enableTouchSensor)  {
    if( motor == 1 ) {
      if(dir == FORWARD) {
        state = digitalRead(M1FTouch);
      } else {
        state = digitalRead(M1BTouch);
      }
    } else {
      if(dir == FORWARD) {
        state = digitalRead(M2FTouch);
      } else {
        state = digitalRead(M2BTouch);
      }
    }
    if( state == HIGH ) {
      Serial.print( "Reached Limit: [" );
      Serial.print( String(motor) );
      Serial.print( "," );
      Serial.print( String(dir) );
      Serial.print( "," );
      Serial.print( String(state) );
      Serial.println( "]" );
    }
  }
  return state;
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
