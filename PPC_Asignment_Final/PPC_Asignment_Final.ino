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
#define TIL_PIN_INPUT A1
#define HC_SR04_TRIG_PIN 3
#define HC_SR04_ECHO_PIN 4

// Define MOTOR pins
#define MOTOR_ENA_PIN 5
#define MOTOR_INTA_PIN1 6
#define MOTOR_INTA_PIN2 7

#define MOTOR_ENB_PIN A2
#define MOTOR_INTB_PIN1 A3
#define MOTOR_INTB_PIN2 A4



// Define SPEAKER pins
#define SPEAKER_PIN 12

// Define MOTOR VIB pin
#define MOTOR_VIB_PIN 13


MD_MAX72XX mx = MD_MAX72XX(HARDWARE_DEVICE,
                           CS_PIN, MAX_DEVICES);

class SERI {
private:

  enum emoState_t  // Value for emotional state.
  {
    ANGRY = 0,
    EXCITED = 1,
    HAPPY = 2,
    SAD = 3,
    SCARED = 4,
    SPEECHL = 5
  };
  uint8_t _brightVal;   // Value from LDR sensor
  uint8_t _distance;    // Value from HC_SR04
  bool _soundDetected;  // Value from Sound Sensor
  bool _tiltDetected;   // Value form Tilt_Ball_Switch sensor
  emoState_t _emoState; // Variabe state.
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

  void setEmoState(emoState_t mode) {
    this->_emoState = mode;
  }

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

  void blink(void) {
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

    // Configure SPEAKER pin
    pinMode(SPEAKER_PIN, OUTPUT);

    // Configure MOTOR pins
    pinMode(MOTOR_INTA_PIN1, OUTPUT);
    pinMode(MOTOR_INTA_PIN2, OUTPUT);
    pinMode(MOTOR_INTB_PIN1, OUTPUT);
    pinMode(MOTOR_INTB_PIN2, OUTPUT);
    pinMode(MOTOR_ENA_PIN, OUTPUT);
    pinMode(MOTOR_ENB_PIN, OUTPUT);
    pinMode(MOTOR_VIB_PIN, OUTPUT);
    digitalWrite(MOTOR_INTA_PIN1, LOW);
    digitalWrite(MOTOR_INTA_PIN2, LOW);
    digitalWrite(MOTOR_INTB_PIN1, LOW);
    digitalWrite(MOTOR_INTB_PIN2, LOW);
  }

  uint8_t getBrightVal(void) {
    return this->_brightVal;
  }

  uint8_t getDistance(void) {
    return this->_distance;
  }

  emoState_t getEmoState(void) {
    return this->_emoState;
  }

  bool isSoundDetected(void) {
    return this->_soundDetected;
  }

  bool isTiltDetected(void) {
    return this->_tiltDetected;
  }

  void sense(void) {
    this->readDistanceToObstacle();
    this->readLightAmbient();
    this->readSoundDectector();
    this->readTiltDetector();
  }

  void speak(long durationMilli = 100) {
    tone(SPEAKER_PIN,
         NOTE_C5, durationMilli);
    PRINTS("\nPLAYSOUND");
  }

  void goFoward(uint8_t speed = 255) {

    digitalWrite(MOTOR_INTA_PIN1, LOW);
    digitalWrite(MOTOR_INTA_PIN2, HIGH);
    digitalWrite(MOTOR_INTB_PIN1, LOW);
    digitalWrite(MOTOR_INTB_PIN2, HIGH);

    analogWrite(MOTOR_ENA_PIN, speed);
    analogWrite(MOTOR_ENB_PIN, speed);

    PRINTS("\n[GO_FOWARD]");
  }

  void goBackward(uint8_t speed = 255) {

    digitalWrite(MOTOR_INTA_PIN1, HIGH);
    digitalWrite(MOTOR_INTA_PIN2, LOW);
    digitalWrite(MOTOR_INTB_PIN1, HIGH);
    digitalWrite(MOTOR_INTB_PIN2, LOW);

    analogWrite(MOTOR_ENA_PIN, speed);
    analogWrite(MOTOR_ENB_PIN, speed);

    PRINTS("\n[GO_BACKWARD]");
  }

  void stop(void) {
    PRINTS("\n[STOP]");
    digitalWrite(MOTOR_INTA_PIN1, LOW);
    digitalWrite(MOTOR_INTA_PIN2, LOW);
    digitalWrite(MOTOR_INTB_PIN1, LOW);
    digitalWrite(MOTOR_INTB_PIN2, LOW);
  }

  void vibrate(uint8_t interval_second = 0) {
    digitalWrite(MOTOR_VIB_PIN, HIGH);
    if (interval_second) {
      // Interval function go here.
      PRINTS("\n Interval vibrating...");
    } else PRINTS("\n vibrating...");
  }

  void stopVibrate(void) {
    digitalWrite(MOTOR_VIB_PIN, LOW);
    PRINTS("\nStop vibrate");
  }

  void showHappy(void) {
    mx.clear();
    // EYE_NORMAL
    mx.setRow(1, 102);
    mx.setRow(2, 102);
    // MOUTH_NORMAL
    mx.setRow(5, 66);
    mx.setRow(6, 60);
    this->setEmoState(SERI::HAPPY);
    PRINTD(this->getEmoState());
  }

  void showSad(void) {
    mx.clear();
    // EYE_SAD
    mx.setRow(1, 36);
    mx.setRow(2, 102);
    // MOUTH_SAD
    mx.setRow(5, 60);
    mx.setRow(6, 66);
    this->setEmoState(SERI::SAD);
    PRINTD(this->getEmoState());
  }

  void showScared(void) {
    mx.clear();

    // Code for display facial

    this->setEmoState(SERI::SCARED);
    PRINTD(this->getEmoState());
  }

  void showAngry(void) {
    mx.clear();
    mx.setRow(1, 66);
    mx.setRow(2, 36);
    mx.setRow(5, 60);
    mx.setRow(6, 66);
    this->setEmoState(SERI::ANGRY);
    PRINTD(this->getEmoState());
  }

  void showExcited(void) {
    mx.clear();

    // Code for display facial

    this->setEmoState(SERI::EXCITED);
    PRINTD(this->getEmoState());
  }

  void showSpeechless(void) {
    mx.clear();
    // EYE_DOT
    mx.setRow(1, 36);
    // MOUTH_CONFUSED
    mx.setRow(5, 126);
    this->setEmoState(SERI::SPEECHL);
    PRINTD(this->getEmoState());
  }
};


SERI mySeri = SERI();

void setup() {
#if DEBUG
  Serial.begin(57600);
#endif
  PRINTS("\n[START PROGRAM]");

  mySeri.begin();
}

void loop() {

  uint8_t brightness = mySeri.getBrightVal();
  uint8_t distance = mySeri.getDistance();

  mySeri.sense();
  if (distance < 20) {
    mySeri.speak();
  }

  if (brightness <= 50) mySeri.showAngry();
  else if (brightness > 50 && brightness <= 150) mySeri.showHappy();
  else if (brightness > 150 && brightness <= 200) mySeri.showSpeechless();
  else if (brightness > 200) mySeri.showSad();

  if (mySeri.getEmoState() == 1) {
    mySeri.vibrate();
    mySeri.stopVibrate();
  }

  if (!mySeri.isTiltDetected()) {
    mySeri.vibrate();
    mySeri.showScared();
  } else {
    mySeri.stopVibrate();
  }
}
