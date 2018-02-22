#include <Servo.h>

// states for state machine
#define PARK        0
#define START_CAR   1
#define NEUTRAL_RC  2
#define DRIVE_RC    3
#define DRIVE_AI    4
#define REVERSE_RC  5
#define STOP_CAR    6

// Arduino pins
#define RC_CH_2     6 // throttle/brake (left stick, y axis)
#define RC_CH_1     2 // steering (right stick, x axis)

#define Gear_Pot      A7
#define Steering_Pot  A0
#define Brake_Pot     A5

#define Ignition_Relay  9
#define Battery_Relay   8
#define Jetson_Boot     39
#define Throttle_Out    7
#define Brake_Dir_Out   34
#define Brake_PWM_Out   36
#define Gear_Dir_Out    52
#define Gear_PWM_Out    48
#define Steering_Out    4

// gear constants
#define GEAR_P 0
#define GEAR_R 1
#define GEAR_N 2
#define GEAR_D 3

#define GEAR_TOLERANCE  5

// brake constants
#define BRAKE_ZERO  0
#define BRAKE_MAX   180
#define BRAKE_TOLERANCE 5

// throttle constants
#define THROTTLE_ZERO     170
#define THROTTLE_LOW      160
#define THROTTLE_SENSIBLE 120
#define THROTTLE_MAX      90

// steering constants
#define STEER_HARD_LEFT     330
#define STEER_MIDDLE        630
#define STEER_HARD_RIGHT    1000
#define STEER_TOLERANCE     20

// misc constants
#define CRANK_TIME 1000

// init actuators
Servo throttleServo;

// init state machine
int state = PARK;

// misc vars
int gear_lever_state = GEAR_P;
int brake_state = BRAKE_ZERO;
int steering_state = STEER_MIDDLE;
int start_delay_counter = 0;

// RC vars
int rc_throttle = THROTTLE_ZERO;
int rc_brake = BRAKE_MAX;
int rc_gear = GEAR_P;
int rc_steering = STEER_MIDDLE;
int rc_stop = 0;

// jetson vars
int jetson_throttle = THROTTLE_ZERO;
int jetson_brake = BRAKE_MAX;
int jetson_gear = GEAR_P;
int jetson_steer = STEER_MIDDLE;
int jetson_stop = 0;

void setup()
{
    Serial.begin(9600);   // debug serial
    Serial1.begin(9600);  // Jetson serial
    Serial2.begin(38400); // Small RoboClaw serial (brake and gear shift)
    Serial3.begin(38400); // Large RoboClaw serial (steering)

    // set up pins
    pinMode(Ignition_Relay, OUTPUT);
    pinMode(Battery_Relay, OUTPUT);
    pinMode(Jetson_Boot, OUTPUT);
    pinMode(Brake_Pot, INPUT);
    pinMode(Gear_Pot, INPUT);
    pinMode(Steering_Pot, INPUT);

    // attach servos
    throttleServo.attach(Throttle_Out);
    
    // init actuators
    throttleServo.write(THROTTLE_ZERO);
    brake_state = BRAKE_ZERO;
    gear_lever_state = GEAR_P;
    steering_state = STEER_MIDDLE;

    // swtich car power on
    digitalWrite(Battery_Relay, HIGH);
}


void loop()
{  
    // read throttle and brake (left stick) from RC receiver
    int ch_2_stick = pulseIn(RC_CH_2, HIGH, 25000);
    delay(5);
    int ch_2_PWM = RCToThrottle(ch_2_stick);
    
    // stick down, brakes on
    if (ch_2_PWM > 100)
    {
        rc_throttle = THROTTLE_ZERO;
        rc_brake = map(ch_2_PWM, 101, 255, 0, 1023);
    }
    // stick up, throttle on
    else if (ch_2_PWM < -100)
    {
        rc_throttle = map(ch_2_PWM, -100, -255, THROTTLE_ZERO, THROTTLE_MAX);
        rc_brake = BRAKE_ZERO;
    }
    // not braking or throttle
    else if (ch_2_PWM < 100 && ch_2_PWM > -100)
    {
        rc_throttle = THROTTLE_ZERO;
        rc_brake = BRAKE_ZERO;
    }

    // read steering (right stick) from RC receiver
    int ch_1_steering = pulseIn(RC_CH_1, HIGH, 25000);
    delay(5);
    rc_steering = RCToSteering(ch_1_steering);
    
//    Serial.print("steering: ");
//    Serial.print(ch_1_PWM);
//    Serial.print("\t");
//    Serial.println(ch_1_steering);


    // gear and brake loops select
    setGear(gear_lever_state);
    setBrake(brake_state);
    setSteering(steering_state);

    // listen for Jetson
    if (Serial1.read() == 0xFF)
    {
//        Serial.println("buffer");
        
        // wait for data
        while (!(Serial1.available() > 3)) Serial.println("wait");
        
        jetson_stop = Serial1.read();
        jetson_steer = Serial1.read();
        jetson_throttle = Serial1.read();
        jetson_brake = Serial1.read();
//
//        Serial.print(" stop: ");
//        Serial.print(jetson_stop);
//        Serial.print(" steer: ");
//        Serial.println(jetson_steer);
//        Serial.print(" jetson_throttle: ");
//        Serial.print(jetson_throttle);
//        Serial.print(" jetson_brake: ");
//        Serial.println(jetson_brake);
    }

    // main state machine
    switch(state)
    {
        case PARK:
        {
            // actuate brakes on, throttle as RC
            brake_state = rc_brake;
            throttleServo.write(rc_throttle);
            steering_state = rc_steering;

            // wait for 200 cycles then start the car
            start_delay_counter++;
            if (start_delay_counter > 200) state = START_CAR;

            break;
        }
        case START_CAR:
        {          
            // brakes on, throttle off
            brake_state = BRAKE_MAX;
            throttleServo.write(THROTTLE_ZERO);
            steering_state = rc_steering;
            
            // start engine and put in gear
            startEngine();
            delay(1000);
            gear_lever_state = GEAR_D;

            // boot Jetson - consider whether these delays are safe - might need a counter to make it asynch
//            delay(500);
//            digitalWrite(Jetson_Boot, HIGH);
//            delay(500);
//            digitalWrite(Jetson_Boot, LOW);
            
            state = DRIVE_RC;
            break;
        }
        case DRIVE_RC:
        {
            // set/actuate all outputs
            brake_state = rc_brake;
            throttleServo.write(rc_throttle);
            steering_state = rc_steering;

            // if request AI transition
                // setBrakes(BRAKE_ZERO);
                // setThrottle(THROTTLE_ZERO);
                // setSteering(STEER_MIDDLE);
                // gear_lever_state = (GEAR_D);
                // state = DRIVE_AI;

            // if request stop car
                // set to stop state

            // if request reverse
                // set to reverse state
            break;
        }
        case DRIVE_AI:
        {
            // if stop command, switch off and go back to park mode
//            if (jetson_stop == 1)
//            {
//                state = STOP_CAR;
//                break;
//            }

//            // set/actuate all outputs
//            if (jetson_brake > 0) brake_state = BRAKE_MAX;
//            else brake_state = BRAKE_ZERO;
            brake_state = rc_brake;

//            if (jetson_throttle > 0) throttleServo.write(THROTTLE_LOW);
//            else throttleServo.write(THROTTLE_ZERO);
            throttleServo.write(rc_throttle);
            
//            steeringServo.write(jetson_steer);
            steering_state = jetson_steer;

                          
            break;
        }
//        case NEUTRAL_RC:  // neutral state, currently unused
//        {
//            setBrake.write(rc_brake);
//            throttleServo.write(rc_throttle);
//
//            // if RC asks for DRIVE_RC
//                // setBrakes(BRAKE_ZERO);
//                // setThrottle(THROTTLE_ZERO);
//                // gear_lever_state = (GEAR_D);
//                // state = DRIVE_RC;
//
//            // if RC asks for DRIVE_AI
//                // setBrakes(BRAKE_ZERO);
//                // setThrottle(THROTTLE_ZERO);
//                // setSteering(STEER_MIDDLE);
//                // gear_lever_state = (GEAR_D);
//                // state = DRIVE_AI;
//            break;
//        }
//        case REVERSE_RC:  // reverse state, currently unused.
//        {
//            // put gear in reverse
//            brake_state = rc_brake;
//            throttleServo.write(rc_throttle);
//            steering_state = rc_steering;
//
//            // if request drive forward, put in DRIVE_RC state
//            break;
//        }
//        case STOP_CAR:
//        {
//            brake_state = BRAKE_MAX;
//            
//            stopEngine();
//            delay(500);
//            gear_lever_state = PARK;
//
//            state = PARK;
//            break;
//        }
        default:
        {
            // included for safety, should not ever end up in this state
            // setBrakes(BRAKE_MAX);
            // setThrottle(THROTTLE_ZERO);
            break;
        }
    }

}

void startEngine()
{
    digitalWrite(Ignition_Relay, HIGH);
    delay(CRANK_TIME);
    digitalWrite(Ignition_Relay, LOW);
}

void stopEngine()
{
    digitalWrite(Battery_Relay, LOW);
    delay(1000);
    digitalWrite(Battery_Relay, HIGH);
}

void setBrake(int newpos)
{
    // RoboClaw Serial2 control, channel 2
    // 128 = full reverse
    // 192 = stop
    // 255 = full forward
    
    int oldpos = analogRead(Brake_Pot);
    
    if ( newpos <= (oldpos - BRAKE_TOLERANCE) ) 
    {
        // actuate brakes off
        Serial2.write(128);
    } 
    else if ( newpos > (oldpos + BRAKE_TOLERANCE) ) 
    {
        // actuate brakes on
        Serial2.write(255);
    }
    else Serial2.write(192);  // else stop brake motor
}

void setGear(int gearState)
{
    // calibrate gear positions with the numbers below
    switch (gearState) 
    {
        case GEAR_P:
            //Parking
            moveGearActuator(700);
            break;
        case GEAR_R:
            //Reverse
            moveGearActuator(555);
            break;
        case GEAR_N:
            //Neutral
            moveGearActuator(477);
            break;
        case GEAR_D:
            //Drive
            moveGearActuator(400);
        default:
            // statements
            return;
     }
}

void moveGearActuator(int newpos)
{
    // RoboClaw Serial2 control, channel 1
    // 1 = full reverse
    // 65 = stop
    // 127 = full forward
    
    int oldpos = analogRead(Gear_Pot);

    if ( newpos <= (oldpos - GEAR_TOLERANCE) )
    {
        // pull shifter back
        Serial2.write(1);
    } 
    else if ( newpos > (oldpos + GEAR_TOLERANCE) ) 
    {
        // push shifter forward
        Serial2.write(127);
    }
    else Serial2.write(64);   // else stop shifter motor
}

void setSteering(int newpos)
{
    // RoboClaw Serial3 control, channel 2
    // 128 = full reverse
    // 192 = stop
    // 255 = full forward

    int oldpos = analogRead(Steering_Pot);
    int rate = abs(newpos - oldpos)/8;      // crude proportional steering rate control (makes it less twitchy)
    if ( rate > 63 ) rate = 63;             // then clamp value to max possible steering rate
  
    if ( newpos <= (oldpos - STEER_TOLERANCE) )
    {
        // steer right
        Serial3.write(192 + rate);
    } 
    else if ( newpos > (oldpos + STEER_TOLERANCE) ) 
    {
        // steer left
        Serial3.write(192 - rate);
    }
    else Serial3.write(192);  // else stop steering motor

    Serial.println();
}

int RCToThrottle(int pulse)
{
    if ( pulse > 1000 ) {
      pulse = map(pulse, 1000, 2000, -500, 500);
      pulse = constrain(pulse, -255, 255);
    } else {
      pulse = 0;
    }

    // Anything in deadzone should stop the motor
    return pulse;
}

int RCToSteering(int pulse)
{
    if ( pulse > 1000 ) {
      pulse = map(pulse, 1110, 1930, STEER_HARD_LEFT, STEER_HARD_RIGHT);
    } else {
      pulse = 0;
    }

    // Anything in deadzone should stop the motor
    return pulse;
}

int parseJetson(String data, char separator, int index)
{
    int found = 0;
    int strIndex[] = { 0, -1 };
    int maxIndex = data.length() - 1;

    for (int i = 0; i <= maxIndex && found <= index; i++) {
        if (data.charAt(i) == separator || i == maxIndex) {
            found++;
            strIndex[0] = strIndex[1] + 1;
            strIndex[1] = (i == maxIndex) ? i+1 : i;
        }
    }
    String val_string = found > index ? data.substring(strIndex[0], strIndex[1]) : "";
    int val_integer = val_string.toInt();

    return val_integer;
}
