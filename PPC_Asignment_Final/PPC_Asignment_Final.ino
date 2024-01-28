/**
\mainpage Arduino SERI main code.
--------------------------------

\DESCRIPTION
-----------
SERI, or Support Emotional Robot Intelligence, 
is an innovative robot designed to interact seamlessly with its surroundings. 
Equipped with numerous types of sensors, SERI is adept at perceiving 
environmental nuances as well as physical movements occurring both 
in and around it. Upon processing this data, SERI expresses emotions 
through a digital facial display or responds physically, 
enhancing user engagement. Each reaction is accompanied by a brief sound 
signal, resembling a voice, which users can personalize to their preference.

\CREDIT
Written and developed by Quy An Tran
Library and extenal sources used
  - MD_MAX72xx.h by majicDesigns

---------------------------------
  \brief 
    - Class SERI
*/

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
#define DATA_PIN 11
#define CS_PIN 3
#define CLK_PIN 13

// Define SENSORs pins
#define LDR_PIN_INPUT A5
#define TIL_PIN_INPUT A1
#define HC_SR04_TRIG_PIN 2
#define HC_SR04_ECHO_PIN 1

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

// Rate for the filter to avoid noisy light sensor, value from 0 - 5
#define LIGHT_FILTER_RATE 2

MD_MAX72XX mx = MD_MAX72XX(HARDWARE_DEVICE,
                           CS_PIN, MAX_DEVICES);

class SERI {
public:
  enum emoState_t  // Value for emotional state.
  {
    ANGRY = 0,
    EXCITED = 1,
    HAPPY = 2,
    SAD = 3,
    SCARED = 4,
    SPEECHL = 5
  };
private:
  uint8_t _brightVal;                    // Value from LDR sensor
  uint8_t _distance;                     // Value from HC_SR04
  bool _isVerticalStand = false;         // Value form Tilt_Ball_Switch sensor
  unsigned long _preBlinkMillis = 0;     // Stores last time LED was updated
  unsigned long _blinkDuration = 0;      // Duration of the current blink
  unsigned long _blinkInterval = 1000;   // Interval at which to blink (millisec)
  bool _isBlinking = false;              // LED blinking state
  uint8_t _savedColumnData[2];           // To save the column data
  unsigned long _preLightSenMillis = 0;  // Stores last time Light data was read
  unsigned long _preTiltSenMillis = 0;   // Stores last time Tilt data was read
  emoState_t _emoState;                  // Variabe state.
  int _readLight[LIGHT_FILTER_RATE];     // Reading from the analog input from LDR (Light Depend Resistor)
  uint8_t readLightIndex = 0;            // The index of the current reading LDR
  int _totalReadLight = 0;               // The running total of reading LDR
  uint8_t intensity = 0;                 // Value for intensity of the LED
  //--------------------------------------------------------------
  /** \name Methods for setter.
   * @{
   */

  /**
     * Sets the brightness value based on the sensor input.
     * @param value The raw sensor reading from the LDR (Light Dependent Resistor).
     */
  void setBrightVal(int value) {
    this->_brightVal = map(value, 0, 1023, 0, 255);
  }

  /**
     * Sets the distance value based on the sensor input.
     * @param value The raw sensor reading from the HC-SR04 ultrasonic sensor.
     */
  void setDistance(int value) {
    if (value > 255) { value = 255; }
    if (value < 0) {
      value = 0;
      PRINTS("\nDistance Sensor broken");
    }
    this->_distance = (uint8_t)value;
  }

  /**
     * Sets the tilt detector status.
     * @param value The state read from the tilt switch sensor.
     */
  void setTiltDetector(bool value) {
    this->_isVerticalStand = value;
  }

  /**
     * Sets the current emotional state of SERI.
     * @param mode The emotional state to be set.
     */
  void setEmoState(emoState_t mode) {
    this->_emoState = mode;
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
   * @return True if initialized with no error, False otherwise.
   *
   */
  bool begin(void) {
    // Initialize MD_MAX72XX
    if (!mx.begin()) {
      PRINTS("\nMD_MAX72XX initialization failed");
      return false;
    }
    mx.clear();
    mx.control(MD_MAX72XX::INTENSITY, 0);

    // Configure SENSORs pins
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

  /**
     * Calibrates the sensors to ensure accurate readings.
     * It's recommended to call this at startup.
     */
  void calibrateSensor(void) {
    for (int i = 0; i < LIGHT_FILTER_RATE; i++) {
      this->readLightAmbient(0);
    }
    this->readTiltDetector(0);
  }

  /**
     * Performs a start-up animation on the LED matrix.
     */
  void startAnimation(void) {
    mx.clear();
    for (int i = COL_SIZE - 1; i >= 0; i--) {
      mx.setColumn(i, 0xff);
      delay(50);
    }
    for (int i = COL_SIZE - 1; i >= 0; i--) {
      mx.setColumn(i, 0);
      delay(50);
    }
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

  bool isVerticalStand(void) {
    return this->_isVerticalStand;
  }

  /**
     * Reads ambient light intensity and filters the reading to smooth out noise.
     * @param readSensorInterval Time interval between successive readings.
     */
  void readLightAmbient(long readSensorInterval = 1000) {
    unsigned long _curMillis = millis();

    if (_curMillis - this->_preLightSenMillis > readSensorInterval) {
      this->_preLightSenMillis = _curMillis;

      // Subtract the last reading:
      _totalReadLight -= _readLight[readLightIndex];
      // Read from the sensor
      _readLight[readLightIndex] = analogRead(LDR_PIN_INPUT);
      // Add the reading to the total
      _totalReadLight += _readLight[readLightIndex];
      // Advance to the next position in the array
      readLightIndex++;

      // Wrap around at the end array
      if (readLightIndex >= LIGHT_FILTER_RATE) {
        readLightIndex = 0;
      }

      // Calculate and update the brightness value
      this->setBrightVal(_totalReadLight / LIGHT_FILTER_RATE);

      // Adjust the screen with the light value.
      uint8_t temp_intensity = map(
        this->getBrightVal(), 0, 255, MAX_INTENSITY, 0);

      // Animation to gradually adjust intensity of the screen
      if (intensity < temp_intensity) {
        intensity++;
        mx.control(MD_MAX72XX::INTENSITY, intensity);
      } else if (intensity > temp_intensity) {
        intensity--;
        mx.control(MD_MAX72XX::INTENSITY, intensity);
      }


      PRINTS("\nRead Light Ambient: ");
      PRINTD(this->getBrightVal());
    } else {
      return;
    }
  }

  /**
     * Reads the distance to an obstacle using the HC-SR04 ultrasonic sensor.
     * @param readingSensor True to enable reading, false to skip.
     */
  void readDistanceToObstacle(bool readingSensor) {

    if (readingSensor) {
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
    } else {
      return;
    }
  }

  /**
     * Reads the tilt sensor status.
     * @param readSensorInterval Time interval between successive readings.
     */
  void readTiltDetector(long readSensorInterval = 2500) {
    unsigned long _curMillis = millis();

    if (_curMillis - this->_preTiltSenMillis > readSensorInterval) {
      this->_preTiltSenMillis = _curMillis;
      this->setTiltDetector(digitalRead(TIL_PIN_INPUT));
      PRINTS("\nRead Tilt Detector: ");
      PRINTD(this->isVerticalStand());
    } else {
      return;
    }
  }

  /**
     * Produces a sound for a specified duration.
     * @param durationMilli The duration of the sound in milliseconds.
     */
  void speak(long durationMilli = 100) {
    tone(SPEAKER_PIN,
         NOTE_C5, durationMilli);
    PRINTS("\nPLAYSOUND");
  }

  /**
     * Moves SERI forward at a specified speed.
     * @param speed The speed of the motors (0 to 255).
     */
  void goFoward(uint8_t speed = 255) {

    digitalWrite(MOTOR_INTA_PIN1, LOW);
    digitalWrite(MOTOR_INTA_PIN2, HIGH);
    digitalWrite(MOTOR_INTB_PIN1, LOW);
    digitalWrite(MOTOR_INTB_PIN2, HIGH);

    analogWrite(MOTOR_ENA_PIN, speed);
    analogWrite(MOTOR_ENB_PIN, speed);

    PRINTS("\n[GO_FOWARD]");
  }

  /**
     * Moves SERI backward at a specified speed.
     * @param speed The speed of the motors (0 to 255).
     */
  void goBackward(uint8_t speed = 255) {

    digitalWrite(MOTOR_INTA_PIN1, HIGH);
    digitalWrite(MOTOR_INTA_PIN2, LOW);
    digitalWrite(MOTOR_INTB_PIN1, HIGH);
    digitalWrite(MOTOR_INTB_PIN2, LOW);

    analogWrite(MOTOR_ENA_PIN, speed);
    analogWrite(MOTOR_ENB_PIN, speed);

    PRINTS("\n[GO_BACKWARD]");
  }

  /**
     * Stops all motor movement.
     */
  void stop(void) {
    PRINTS("\n[STOP]");
    digitalWrite(MOTOR_INTA_PIN1, LOW);
    digitalWrite(MOTOR_INTA_PIN2, LOW);
    digitalWrite(MOTOR_INTB_PIN1, LOW);
    digitalWrite(MOTOR_INTB_PIN2, LOW);
  }

  /**
     * Activates the vibration motor for a set interval.
     * @param interval_second The duration of the vibration in seconds.
     */
  void vibrate(uint8_t interval_second = 0) {
    digitalWrite(MOTOR_VIB_PIN, HIGH);
    if (interval_second) {
      // Interval function go here.
      PRINTS("\n Interval vibrating...");
    } else PRINTS("\n vibrating...");
  }

  /**
     * Stops the vibration motor.
     */
  void stopVibrate(void) {
    digitalWrite(MOTOR_VIB_PIN, LOW);
    PRINTS("\nStop vibrate");
  }

  /**
     * Updates the blinking animation on the LED matrix in a non-blocking manner.
     */
  void updateBlink() {
    unsigned long _curMillis = millis();

    if (!this->_isBlinking
        && _curMillis - this->_preBlinkMillis >= this->_blinkInterval) {
      // Start blinking
      this->_isBlinking = true;
      this->_blinkDuration = random(50, 101);
      this->_preBlinkMillis = _curMillis;

      // Save the current column data
      // and set the column to 0 (off)
      _savedColumnData[0] = mx.getColumn(1);
      _savedColumnData[1] = mx.getColumn(2);

      // Blinking animation display
      mx.setColumn(1, 0);
      delay(20);
      mx.setColumn(2, 0);

      // Set blinking time to random
      this->_blinkInterval = random(1000, 2601);
    }

    else if (this->_isBlinking
             && _curMillis - this->_preBlinkMillis >= this->_blinkDuration) {
      // Stop blinking
      this->_isBlinking = false;
      this->_preBlinkMillis = _curMillis;

      // Restore the original display
      mx.setColumn(2, _savedColumnData[1]);
      delay(20);
      mx.setColumn(1, _savedColumnData[0]);
    }
  }

  /**
     * Displays a 'Happy' emotion on the LED matrix.
     */
  void showHappy(void) {
    if (this->getEmoState() != SERI::HAPPY) {
      mx.clear();
      // EYE_NORMAL
      mx.setColumn(1, 102);
      mx.setColumn(2, 102);
      // MOUTH_NORMAL
      mx.setColumn(5, 66);
      mx.setColumn(6, 60);
      this->setEmoState(SERI::HAPPY);
      PRINTD(this->getEmoState());
    } else {
      return;
    }
  }

  /**
     * Displays a 'Sad' emotion on the LED matrix.
     */
  void showSad(void) {
    if (this->getEmoState() != SERI::SAD) {
      mx.clear();
      // EYE_SAD
      mx.setColumn(1, 36);
      mx.setColumn(2, 102);
      // MOUTH_SAD
      mx.setColumn(5, 60);
      mx.setColumn(6, 66);
      this->setEmoState(SERI::SAD);
      PRINTD(this->getEmoState());
    } else {
      return;
    }
  }

  /**
     * Displays a 'Scared' emotion on the LED matrix.
     */
  void showScared(void) {
    if (this->getEmoState() != SERI::SCARED) {
      mx.clear();
      // EYE_SAD
      mx.setColumn(1, 36);
      mx.setColumn(2, 102);
      // MOUTH_SCARED
      mx.setColumn(5, 52);
      mx.setColumn(6, 74);
      this->setEmoState(SERI::SCARED);
      PRINTD(this->getEmoState());
    } else {
      return;
    }
  }

  /**
     * Displays an 'Angry' emotion on the LED matrix.
     */
  void showAngry(void) {
    if (this->getEmoState() != SERI::ANGRY) {
      mx.clear();
      // EYE_ANGRY
      mx.setColumn(1, 66);
      mx.setColumn(2, 36);
      // MOUTH_ANGRY
      mx.setColumn(5, 60);
      mx.setColumn(6, 66);
      this->setEmoState(SERI::ANGRY);
      PRINTD(this->getEmoState());
    }
  }

  /**
     * Displays an 'Excited' emotion on the LED matrix.
     */
  void showExcited(void) {
    if (this->getEmoState() != SERI::EXCITED) {
      mx.clear();
      // EYE_LINE
      mx.setColumn(1, 102);
      // MOUTH_EXCITED
      mx.setColumn(3, 126);
      mx.setColumn(4, 66);
      mx.setColumn(5, 66);
      mx.setColumn(6, 60);
      this->setEmoState(SERI::EXCITED);
      PRINTD(this->getEmoState());
    }
  }

  /**
     * Displays a 'Speechless' emotion on the LED matrix.
     */
  void showSpeechless(void) {
    if (this->getEmoState() != SERI::SPEECHL) {
      mx.clear();
      // EYE_DOT
      mx.setColumn(1, 36);
      // MOUTH_CONFUSED
      mx.setColumn(5, 126);
      this->setEmoState(SERI::SPEECHL);
      PRINTD(this->getEmoState());
    }
  }

  /**
     * Resets the LED matrix display.
     */
  void reset(void) {
    mx.clear();
    mx.control(MD_MAX72XX::INTENSITY, 0);
  }
};

/**
------------------
main function go here.
*/

SERI mySeri = SERI();
bool animated = false;

void setup() {
#if DEBUG
  Serial.begin(57600);
#endif
  PRINTS("\n[START PROGRAM]");

  mySeri.begin();
  mySeri.calibrateSensor();
}

void loop() {

  // Smooth out the transition for reset.
  if (!animated) {
    mySeri.startAnimation();
    animated = true;
    delay(500);
  }

  if (mySeri.isVerticalStand()) {
    if (mySeri.getBrightVal() <= 5) {
      mySeri.showAngry();
    } else if (mySeri.getBrightVal() > 5 && mySeri.getBrightVal() <= 75) {
      mySeri.showExcited();
    } else if (mySeri.getBrightVal() > 75 && mySeri.getBrightVal() <= 175) {
      mySeri.showHappy();
    } else if (mySeri.getBrightVal() > 175 && mySeri.getBrightVal() <= 220) {
      mySeri.showSad();
    } else if (mySeri.getBrightVal() > 220) {
      mySeri.showScared();
    }
  }

  if (mySeri.getEmoState() != SERI::SCARED && !mySeri.isVerticalStand()) {
    mySeri.showScared();
    mySeri.vibrate(2000);
  } else {
    mySeri.stopVibrate();
  }

  mySeri.readDistanceToObstacle(false);
  mySeri.updateBlink();

  mySeri.readLightAmbient();
  mySeri.readTiltDetector();
}
