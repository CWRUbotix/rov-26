#include <Wire.h>

const uint8_t SDA_PIN = 4 //Probably not 4
const uint8_t SCL_PIN = 5 //Probably not 5
const uint8_t LEFT_PIN = 1 //Definietely not 1
const uint8_t RIGHT_PIN = 2 //Definitely not 2

void setup() {
    pinMode(LEFT_PIN, OUTPUT);
    pinMode(RIGHT_PIN, OUTPUT);
    Serial.begin(115200);
    delay(200);
    Wire.begin(SDA_PIN, SCL_PIN);

    Serial.println("Initializing manipulators ...");
    Wire.onRecieve(recieveState);

}

void loop() {
    delay(100);
}

void recieveState(int howMany) {
    Serial.println("Recieved");
    Serial.println(howMany);
    if(howMany == 2) {
        Wire.read();
        uint8_t data = Wire.read();
        Serial.println("Data was 2 bytes")
        Serial.println(data);
        if((data >> 3) & 1) {
            Serial.println("Writing high to left");
            digitalWrite(LEFT_PIN, HIGH);
            data -= 8; 
        } else {
            Serial.println("Writing low to left");
            digitalWrite(LEFT_PIN, LOW);
        }

        if((data >> 2) & 1) {
            Serial.println("Writing high to right");
            digitalWrite(RIGHT_PIN, HIGH);
        } else {
            Serial.println("Writing low to right");
            digitalWrite(RIGHT_PIN, LOW);
        }
        Serial.println("At end of if");
    }
    Serial.println("End of manipulator callback");
}