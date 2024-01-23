#include "MD_MAX72xx.h"
#include "pitches.h"
#include "SPI.h"

// Turn on debug statements to the serial output
#define DEBUG 1

#if DEBUG
#define PRINT(s, x) \
  { \
    Serial.print(F(s)); \
    Serial.print(x); \
  }
#define PRINTS(x) Serial.print(F(x))
#define PRINTD(x) Serial.println(x, DEC)

#else
#define PRINT(s, x)
#define PRINTS(x)
#define PRINTD(x)

#endif

// Define MD_MAX72 pins
#define HARDWARE_DEVICE MD_MAX72XX::FC16_HW
#define MAX_DEVICES 1
#define CS_PIN 11

// Define SENSORs pins
#define LDR_PIN_INPUT A5
#define SSS_PIN_INPUT 1
#define TIL_PIN_INPUT 2
#define HC_SR04_TRIG_PIN 3
#define HC_SR04_ECHO_PIN 4

// Define MOTOR pins
#define MOTOR_INTA_PIN1 5
#define MOTOR_INTA_PIN2 6
#define MOTOR_INTB_PIN1 7
#define MOTOR_INTB_PIN2 8
#define MOTOR_ENA_PIN 9
#define MOTOR_ENB_PIN 10

// Define SPEAKER pins
#define SPEAKER_PIN 12


MD_MAX72XX mx = MD_MAX72XX(HARDWARE_DEVICE,
                           CS_PIN, MAX_DEVICES);

class IRES {
private:
  uint8_t _brightVal;   // Value from LDR sensor
  uint8_t _distance;    // Value from HC_SR04
  bool _soundDetected;  // Value from Sound Sensor
  bool _tiltDetected;   // Value form Tilt_Ball_Switch sensor
  //--------------------------------------------------------------
  /** \name Methods for setter.
   * @{
   */
  void setBrightVal(int value) {
    this->_brightVal = map(value, 0, 1023, 0, 255);
  }

  void setDistance(int value) {
    if (value > 255) { value = 255; }
    if (value < 0) {
      value = 0;
      PRINTS("\nDistance Sensor broken");
    }
    this->_distance = (uint8_t)value;
  }

  void setSoundDetected(bool value) {
    this->_soundDetected = value;
  }

  void setTiltDetected(bool value) {
    this->_tiltDetected = value;
  }
public:
  /**
   * Class Constructor
   *
   * Instantiate a new instance of the class.
   * 
   */
  IRES() {}
  /**
   * Initialize the object.
   *
   * Initialize the object data. This needs to be called during setup() 
   * to initialize new data for the class
   * that cannot be done during the object creation.
   *
   * \return true if initialized with no error, false otherwise.
   *
   */
  bool begin(void) {
    // Initialize MD_MAX72XX
    if (!mx.begin()) {
      PRINTS("\nMD_MAX72XX initialization failed");
      return false;
    }
    mx.control(MD_MAX72XX::INTENSITY, 0);

    // Configure SENSORs pins
    pinMode(SSS_PIN_INPUT, INPUT);
    pinMode(TIL_PIN_INPUT, INPUT);

    // Configure HC_SR04 pins
    pinMode(HC_SR04_TRIG_PIN, OUTPUT);
    pinMode(HC_SR04_ECHO_PIN, INPUT);

    // Configure MOTOR pins
    pinMode(MOTOR_INTA_PIN1, OUTPUT);
    pinMode(MOTOR_INTA_PIN2, OUTPUT);
    pinMode(MOTOR_INTB_PIN1, OUTPUT);
    pinMode(MOTOR_INTB_PIN2, OUTPUT);
    pinMode(MOTOR_ENA_PIN, OUTPUT);
    pinMode(MOTOR_ENB_PIN, OUTPUT);
    digitalWrite(MOTOR_INTA_PIN1, LOW);
    digitalWrite(MOTOR_INTA_PIN2, LOW);
    digitalWrite(MOTOR_INTB_PIN1, LOW);
    digitalWrite(MOTOR_INTB_PIN2, LOW);
  }

  //--------------------------------------------------------------
  /** \name Methods for setter.
   * @{
   */
  uint8_t getBrightVal(void) {
    return this->_brightVal;
  }

  uint8_t getDistance(void) {
    return this->_distance;
  }

  bool isSoundDetected(void) {
    return this->_soundDetected;
  }

  bool isTiltDetected(void) {
    return this->_tiltDetected;
  }

  /** \name Methods for object and hardware control.
   * @{
   */
  void readLightAmbient(void) {
    this->setBrightVal(analogRead(LDR_PIN_INPUT));
    PRINTS("\nRead Light Ambient: ");
    PRINTD(this->getBrightVal());
  }

  void readDistanceToObstacle(void) {
    // The sensor is triggered by a HIGH pulse of 10 or more microseconds.
    // Give a short LOW pulse beforehand to ensure a clean HIGH pulse:
    digitalWrite(HC_SR04_TRIG_PIN, LOW);
    delayMicroseconds(5);
    digitalWrite(HC_SR04_TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(HC_SR04_TRIG_PIN, LOW);

    // Read the signal from the sensor: a HIGH pulse whose
    // duration is the time (in microseconds) from the sending
    // of the ping to the reception of its echo off of an object.
    long dur = pulseIn(HC_SR04_ECHO_PIN, HIGH);
    long dis = (dur / 2) * 0.0343;  // speed of sound 343m/s
    this->setDistance(dis);
    PRINTS("\nRead Distance: ");
    PRINTD(this->getDistance());
  }

  void readSoundDectector(void) {
    this->setSoundDetected(digitalRead(SSS_PIN_INPUT));
    PRINTS("\nRead Sound Detector: ");
    PRINTD(this->isSoundDetected());
  }

  void readTiltDetector(void) {
    this->setTiltDetected(digitalRead(TIL_PIN_INPUT));
    PRINTS("\nRead Tilt Detector: ");
    PRINTD(this->isTiltDetected());
  }

  void speak() {
    tone(SPEAKER_PIN,
         NOTE_C5, 500);
    PRINTS("\nPLAYSOUND");
  }
};


IRES jarvis = IRES();

void setup() {
#if DEBUG
  Serial.begin(57600);
#endif
  PRINTS("\n[START PROGRAM]");

  jarvis.begin();
}

void loop() {

  jarvis.readLightAmbient();
  jarvis.readDistanceToObstacle();
  jarvis.readSoundDectector();
  jarvis.readTiltDetector();
  if (jarvis.getBrightVal() > 200) {
    jarvis.speak();
  }
  delay(1000);

  int i = jarvis.getBrightVal();

  if (i > 100 && i < 200) {

    Serial.println("Content");

    mx.clear();

    // EYE_NORMAL
    mx.setRow(1, 102);
    mx.setRow(2, 102);

    // MOUTH_NORMAL
    mx.setRow(5, 66);
    mx.setRow(6, 60);
  } else if (i < 10) {

    Serial.println("Sad");

    mx.clear();

    // EYE_SAD
    mx.setRow(1, 36);
    mx.setRow(2, 102);

    // MOUTH_SAD
    mx.setRow(5, 60);
    mx.setRow(6, 66);
  }

  else if (i >= 10 && i <= 100) {

    Serial.println("OK");

    mx.clear();

    // EYE_NORMAL
    mx.setRow(1, 36);

    // MOUTH_CONFUSED
    mx.setRow(5, 126);
  }

  else if (i >= 200) {

    Serial.println("Angry");

    mx.clear();

    mx.setRow(1, 66);
    mx.setRow(2, 36);
    mx.setRow(5, 60);
    mx.setRow(6, 66);
  }

  delay(500);
}
