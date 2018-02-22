// Run this code on the Arduino Uno which is connected to the car ignition relay (the main battery power, 
// not starter motor relay). It listens to the RC receiver and allows a remote kill no matter what the 
// Jetson or main Arduino Mega are doing.

#define KILL_IN   3
#define GEAR_IN   10

#define RELAY_OUT 8

#define DRIVE_OUT   4
#define REVERSE_OUT 5

void setup() {
    Serial.begin(9600);
    
    pinMode(RELAY_OUT, OUTPUT);
    pinMode(DRIVE_OUT, OUTPUT);
    pinMode(REVERSE_OUT, OUTPUT);
    
    digitalWrite(RELAY_OUT, HIGH);
    digitalWrite(DRIVE_OUT, LOW);
    digitalWrite(REVERSE_OUT, LOW);
}

void loop() {
    // igntion from RC controller
    int kill_input = pulseIn(KILL_IN, HIGH, 25000);    
    
    if (kill_input > 1750) digitalWrite(RELAY_OUT, LOW);
    Serial.print("kill: ");
    Serial.print(kill_input);
    
    int gear_input = pulseIn(GEAR_IN, HIGH, 25000);
//    if (gear_input    
    
    Serial.print("\tgear: ");
    Serial.println(gear_input);

    delay(5);
}
