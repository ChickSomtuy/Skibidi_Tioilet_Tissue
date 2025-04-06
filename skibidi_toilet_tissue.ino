#include <Adafruit_MCP23X17.h>

Adafruit_MCP23X17 mcp;

// Motor 1 pins (GPA)
#define MOTOR1_IN1 7  // GPA7
#define MOTOR1_IN2 6  // GPA6
#define MOTOR1_IN3 5  // GPA5
#define MOTOR1_IN4 4  // GPA4

// Motor 2 pins (GPB)
#define MOTOR2_IN1 11  // GPB3
#define MOTOR2_IN2 10  // GPB2
#define MOTOR2_IN3 9   // GPB1
#define MOTOR2_IN4 8   // GPB0

// Stepper motor sequence (half-step)
const uint8_t stepSequence[8] = {
  0b1000, 0b1100, 0b0100, 0b0110,
  0b0010, 0b0011, 0b0001, 0b1001
};

int currentStep1 = 0;
int currentStep2 = 0;
// =============================Ultra sonic sensor==================
const int trigPin = 18;    //Trig Ultrasonic
const int echoPin = 19;    //Echo Ultrasonic
const int sensorPin = 33;  //Pin IR sensor

//define sound speed in cm/uS
#define SOUND_SPEED 0.034
//#define CM_TO_INCH 0.393701

long duration;
float distanceCm;
float distanceInch;

void oneStep(int motorNumber, bool reverse = false) {
  int* currentStep = (motorNumber == 1) ? &currentStep1 : &currentStep2;

  if (reverse) {
    *currentStep = (*currentStep - 1 + 😎 % 8;
  } else {
    *currentStep = (*currentStep + 1) % 8;
  }

  uint8_t pattern = stepSequence[*currentStep];

  if (motorNumber == 1) {
    mcp.digitalWrite(MOTOR1_IN1, (pattern & 0b1000) ? HIGH : LOW);
    mcp.digitalWrite(MOTOR1_IN2, (pattern & 0b0100) ? HIGH : LOW);
    mcp.digitalWrite(MOTOR1_IN3, (pattern & 0b0010) ? HIGH : LOW);
    mcp.digitalWrite(MOTOR1_IN4, (pattern & 0b0001) ? HIGH : LOW);
  } else {
    mcp.digitalWrite(MOTOR2_IN1, (pattern & 0b1000) ? HIGH : LOW);
    mcp.digitalWrite(MOTOR2_IN2, (pattern & 0b0100) ? HIGH : LOW);
    mcp.digitalWrite(MOTOR2_IN3, (pattern & 0b0010) ? HIGH : LOW);
    mcp.digitalWrite(MOTOR2_IN4, (pattern & 0b0001) ? HIGH : LOW);
  }
}

void setup() {
  Serial.begin(115200);      // Starts the serial communication
  pinMode(trigPin, OUTPUT);  // Sets the trigPin as an Output
  pinMode(echoPin, INPUT);   // Sets the echoPin as an Input



  Wire.begin();

  // Initialize the MCP23017
  if (!mcp.begin_I2C()) {
    Serial.println("Error initializing MCP23017. Check your connections.");
    while (1)
      ;
  }

  // Set the motor pins as outputs
  // Motor 1
  mcp.pinMode(MOTOR1_IN1, OUTPUT);
  mcp.pinMode(MOTOR1_IN2, OUTPUT);
  mcp.pinMode(MOTOR1_IN3, OUTPUT);
  mcp.pinMode(MOTOR1_IN4, OUTPUT);

  // Motor 2
  mcp.pinMode(MOTOR2_IN1, OUTPUT);
  mcp.pinMode(MOTOR2_IN2, OUTPUT);
  mcp.pinMode(MOTOR2_IN3, OUTPUT);
  mcp.pinMode(MOTOR2_IN4, OUTPUT);

  Serial.println("Dual stepper motor test ready!");
}
int now1 = 0;
int last1 = 0;
int now2 = 0;
int last2 = 0;
void loop() {
  now1 = millis();
  if (now1 - last1 >= 500) {
    last1 = now1;
    // Clears the trigPin
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    // Sets the trigPin on HIGH state for 10 micro seconds
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);

    // Reads the echoPin, returns the sound wave travel time in microseconds
    duration = pulseIn(echoPin, HIGH);

    // Calculate the distance
    distanceCm = duration * SOUND_SPEED / 2;

    // // Convert to inches
    // distanceInch = distanceCm * CM_TO_INCH;

    // Prints the distance in the Serial Monitor
    Serial.print("Distance from Ultrasonic (cm): ");
    Serial.println(distanceCm);
    // Serial.print("Distance (inch): ");
    // Serial.println(distanceInch);

    int sensorValue = analogRead(sensorPin);

    // Convert the analog reading to voltage
    float voltage = sensorValue * (3.3 / 4095.0);

    // Convert voltage to distance (in cm)
    // This formula is an approximation and may need calibration
    float distance = 12.08 * pow(voltage, -1.058);

    // Print the results
    // Serial.print("Analog reading: ");
    // Serial.print(sensorValue);
    // Serial.print(", Voltage: ");
    // Serial.print(voltage);
    Serial.print("Distance from IR (cm) : ");
    Serial.println(distance);
    Serial.println("------------------------------------------------------------");
  }

  if (distanceCm < 10) {
    now2 = millis();
    if (now2 - last2 >= 5000) {
      last2 = now2;
      for (int i = 0; i < 4096; i++) {  // 4096 half-steps for a typical 28BYJ-48 stepper motor
        oneStep(1, false);              // Motor 1 forward
        oneStep(2, false);              // Motor 2 forward
        delay(2);                       // Adjust delay for desired speed
      }
    }
  }
}
