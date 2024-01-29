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
  - DHT.h, DHT_U.h and Adafruit_Sensor.h by Adafruit.

---------------------------------
  \brief 
    - Class SERI
*/
#include "Adafruit_Sensor.h"
#include "DHT_U.h"
#include "DHT.h"
#include "MD_MAX72xx.h"
#include "pitches.h"
#include "SPI.h"

// Turn on debug statements to the serial output
/*
 * WARNING, TURN ON DEBUG MODE WILL TURN OFF SENDING DATA
 * TO PROCESSING.
*/
#define DEBUG 1

#if DEBUG
#define PRINT(s, x) \
  { \
    Serial.print(F(s)); \
    Serial.print(x); \
  }
#define PRINTS(x) Serial.print(F(x))
#define PRINTD(x) Serial.println(x, DEC)
#define SEND_DATA 0

#else
#define PRINT(s, x)
#define PRINTS(x)
#define PRINTD(x)
#define SEND_DATA 1

#endif

// Define MD_MAX72 pins
#define HARDWARE_DEVICE MD_MAX72XX::FC16_HW
#define MAX_DEVICES 1
#define DATA_PIN 11
#define CS_PIN 3
#define CLK_PIN 13

// Define SENSORs pins
#define TEMP_PIN 4
#define LDR_PIN_INPUT A5
#define TIL_PIN_INPUT A1
#define HC_SR04_TRIG_PIN 2
#define HC_SR04_ECHO_PIN 8

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

DHT_Unified dht(TEMP_PIN, DHT11);
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
    SPEECHL = 5,
    COLD = 6,
    HOT = 7,
    SATISFIED = 8,
    DRY = 9
  };
private:
  uint8_t _brightVal;             // Value from LDR sensor
  float _temperatureVal;          // Value temperature from the DHT11 sensor
  float _humidityVal;             // Value for humidity
  int _distance;                  // Value from HC_SR04
  bool _isVerticalStand = false;  // Value form Tilt_Ball_Switch sensor
  emoState_t _emoState;           // State of emotional face
  bool _isMoving = true;          // State of turn on the motor driver
  bool _interuptEvent = false;    // State checking for interupted event.

  unsigned long _preBlinkMillis = 0;    // Stores last time LED was updated
  unsigned long _blinkDuration = 0;     // Duration of the current blink
  unsigned long _blinkInterval = 1000;  // Interval at which to blink (millisec)
  bool _isBlinking = false;             // LED blinking state
  uint8_t _savedColumnData[2];          // To save the column data

  unsigned long _preLightSenMillis = 0;        // Stores last time Light data was read
  unsigned long _preTempSenMillis = 0;         // Stores last time Temp data was read
  unsigned long _preHumiSenMillis = 0;         // Stores last time Humidity data was read
  unsigned long _preTiltSenMillis = 0;         // Stores last time Tilt data was read
  unsigned long _preDistSenMillis = 0;         // Stores last time Distance data was read
  unsigned long _preColdAnimatedMillis = 0;    // Stores last time Animated Cold mouth
  unsigned long _preHotAnimatedMillis = 0;     // Stores last time Animated Hot mouth
  unsigned long _lastEmotionChangeMillis = 0;  // Store last time Change emotion Speechless
  unsigned long _preSatisfiedMillis = 0;       // Store last time Animated Satisfied face

  int _readLight[LIGHT_FILTER_RATE];  // Reading from the analog input from LDR (Light Depend Resistor)
  uint8_t readLightIndex = 0;         // The index of the current reading LDR
  int _totalReadLight = 0;            // The running total of reading LDR
  uint8_t intensity = 0;              // Value for intensity of the LED

  unsigned long _animationDelay = 0;  // Store last time was animated
  bool _switchAnimation = false;      // Store the state to switch Animation
  bool _tempSwitchAnimation = false;  // store the state for later compare

  emoState_t _previousEmoState;  // To store the previous emotion state
  unsigned long _speechlessDuration = 5000;
  bool _isShowingSpeechless = false;
  bool _satisfiedAnimationState = false;

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
   * Sets the temperature value based on the sensor input.
   * @param value The temperature reading from the DHT sensor.
   */
  void setTemperatureVal(float value) {
    this->_temperatureVal = value;
  }

  /**
   * Sets the humidity value based on the sensor input.
   * @param value The humidity reading from the DHT sensor.
   */
  void setHumidityVal(float value) {
    this->_humidityVal = value;
  }

  /**
     * Sets the distance value based on the sensor input.
     * @param value The raw sensor reading from the HC-SR04 ultrasonic sensor.
     */
  void setDistance(int value) {
    this->_distance = value;
  }

  /**
     * Sets the tilt detector status.
     * @param value The state read from the tilt switch sensor.
     */
  void setTiltDetector(bool value) {
    this->_isVerticalStand = value;
  }

  /**
     * Sets the moving status.
     * @param value The state to be set.
     */
  void setIsMoving(bool value) {
    this->_isMoving = value;
  }

  /**
     * Sets the current emotional state of SERI.
     * @param mode The emotional state to be set.
     */
  void setEmoState(emoState_t mode) {
    this->_emoState = mode;
  }

  /**
     * Helper function to convert the emotional state enum to a string.
     * @return A string representing the current emotional state.
     */
  String getEmotionalStateAsString() {
    switch (_emoState) {
      case ANGRY:
        return "Angry";
      case EXCITED:
        return "Excited";
      case HAPPY:
        return "Happy";
      case SAD:
        return "Sad";
      case SCARED:
        return "Scared";
      case SPEECHL:
        return "Speechless";
      default:
        return "Unknown";
    }
  }

  /**
   * Handles the animation of the mouth for the 'Cold' emotion.
   *
   * This function toggles the mouth's animation state between two
   * patterns to simulate shivering or teeth chattering. It is called
   * by showCold function based on the specified animation interval.
   */
  void animateMouthCold() {
    // Toggle animation state
    this->_switchAnimation = !this->_switchAnimation;

    // Update mouth based on animation state
    if (this->_switchAnimation) {
      mx.setColumn(5, 84);
      mx.setColumn(6, 42);
    } else {
      mx.setColumn(5, 42);
      mx.setColumn(6, 84);
    }
  }

  /**
   * Handles the animation of the mouth for the 'Hot' emotion.
   *
   * This function toggles the mouth's animation state between two
   * patterns to simulate panting or being overheated. It is called
   * by showHot function based on the specified animation interval.
   */
  void animateMouthHot() {
    // Toggle animation state
    _switchAnimation = !_switchAnimation;

    // Update mouth based on animation state
    if (_switchAnimation) {
      mx.setColumn(4, 60);
      mx.setColumn(5, 126);
      mx.setColumn(6, 126);
    } else {
      mx.setColumn(4, 0);
      mx.setColumn(5, 24);
      mx.setColumn(6, 24);
    }
  }

  /**
   * Handles the animation of the satisfied emotion.
   *
   * This function toggles the mouth and eyes's animation state between two
   * patterns to simulate smilling happy. It is called
   * by showSatisfied function based on the specified animation interval.
   */
  void animateSatisfied() {
    _satisfiedAnimationState = !_satisfiedAnimationState;

    if (_satisfiedAnimationState) {
      mx.setColumn(1, 102);
      mx.setColumn(2, 102);
      mx.setColumn(5, 66);
      mx.setColumn(6, 60);
    } else {
      mx.setColumn(1, 90);
      mx.setColumn(2, 90);
      mx.setColumn(5, 102);
      mx.setColumn(6, 36);
    }
  }

  /**
   * Displays the 'Speechless' emotion randomly.
   * This function is called periodically to randomly trigger the 'Speechless' emotion.
   * The chance of triggering and the duration are controlled by internal parameters.
   */
  void showSpeechlessRandomly() {
    unsigned long curMillis = millis();

    if (!_isShowingSpeechless
        && (curMillis - _lastEmotionChangeMillis > _speechlessDuration)) {
      // Randomly decide whether to show 'Speechless'
      if (random(0, 100) < 5) {
        this->showSpeechless();
        _isShowingSpeechless = true;
        _lastEmotionChangeMillis = curMillis;
      }
    } else if (_isShowingSpeechless
               && (curMillis - _lastEmotionChangeMillis > _speechlessDuration)) {
      resetEmotionState();
      _isShowingSpeechless = false;
      _lastEmotionChangeMillis = curMillis;
    }
  }

  /**
   * Resets the emotion state of SERI to the previous emotion.
   * This function is typically called after displaying a temporary emotion (like 'Speechless')
   * to revert back to the ongoing emotion state.
   */
  void resetEmotionState() {
    switch (_previousEmoState) {
      case EXCITED:
        showExcited();
        break;
      case HAPPY:
        showHappy();
        break;
      case SAD:
        showSad();
        break;
      case SATISFIED:
        showSatisfied();
        break;
      default:
        break;
    }
  }

public:
  /**
   * Class Constructor
   *
   * Instantiate a new instance of the class.
   * Pass the oneWire object from One Wire library to control 
   * the temperature sensor
   *
   */
  SERI() {}
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
    dht.begin();
    this->readTemperature(0);
    this->readHumidity(0);
  }

  /**
     * Waiting for Processing to send back
     * Handshake protocol with the Processing sketch
     */
  void establishContact() {
    uint8_t col_num = 0;
    while (true) {

      // If Processing send "B", finish handshake protocol
      if (Serial.available() <= 0 && Serial.find("B")) {
        Serial.println("Established connection");
        break;
      }

      // If nothing is send, send request to connect.
      Serial.println("A");

      // Waiting for connection animation
      if (col_num > COL_SIZE) {
        col_num = 0;
      }
      mx.setColumn(col_num, 255);
      col_num++;
      mx.setColumn(col_num, 0);
    }
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

  /**
     * Sets the interupt event state of SERI.
     * @param value The state to be set.
     */
  void setInteruptEvent(bool value) {
    this->_interuptEvent = value;
  }

  uint8_t getBrightVal(void) {
    return this->_brightVal;
  }

  float getTemperatureVal(void) {
    return this->_temperatureVal;
  }

  float getHumidityVal(void) {
    return this->_humidityVal;
  }

  int getDistance(void) {
    return this->_distance;
  }

  bool isMoving(void) {
    return this->_isMoving;
  }

  emoState_t getEmoState(void) {
    return this->_emoState;
  }

  bool isVerticalStand(void) {
    return this->_isVerticalStand;
  }

  bool isInteruptEvent(void) {
    return this->_interuptEvent;
  }

  /**
     * Reads ambient light intensity and filters the reading to smooth out noise.
     * @param readSensorInterval Time interval between successive readings.
     */
  void readLightAmbient(unsigned long readSensorInterval = 1000) {
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
     * Reads temperature around using DHT11 temperature sensor.
     * @param readSensorInterval Time interval between successive readings.
     */
  void readTemperature(unsigned long readSensorInterval = 60000) {
    unsigned long _curMillis = millis();

    if (_curMillis - this->_preTempSenMillis > readSensorInterval) {
      this->_preTempSenMillis = _curMillis;

      sensors_event_t event;
      dht.temperature().getEvent(&event);
      if (isnan(event.temperature)) {
        Serial.println("Error reading temperature!");
        setTemperatureVal(-1);
      } else {
        setTemperatureVal(event.temperature);
      }
      PRINTS("\nRead Temperature in *C: ");
      PRINTD(this->getTemperatureVal());
    }
  }

  /**
     * Reads humidity around using DHT11 temperature sensor.
     * @param readSensorInterval Time interval between successive readings.
     */
  void readHumidity(unsigned long readSensorInterval = 60000) {
    unsigned long _curMillis = millis();

    if (_curMillis - this->_preHumiSenMillis > readSensorInterval) {
      this->_preHumiSenMillis = _curMillis;

      sensors_event_t event;
      dht.humidity().getEvent(&event);
      if (isnan(event.relative_humidity)) {
        Serial.println("Error humidity temperature!");
        setHumidityVal(-1);
      } else {
        setHumidityVal(event.relative_humidity);
      }
      PRINTS("\nRead Humidity in %: ");
      PRINTD(this->getHumidityVal());
    }
  }

  /**
     * Reads the distance to an obstacle using the HC-SR04 ultrasonic sensor.
     * @param readSensorInterval Time interval between successive readings.
     */
  void readDistanceToObstacle(unsigned long readSensorInterval = 1500) {
    unsigned long _curMillis = millis();

    if (_curMillis - this->_preDistSenMillis > readSensorInterval) {
      this->_preDistSenMillis = _curMillis;

      digitalWrite(HC_SR04_TRIG_PIN, LOW);
      delayMicroseconds(2);
      digitalWrite(HC_SR04_TRIG_PIN, HIGH);
      delayMicroseconds(10);
      digitalWrite(HC_SR04_TRIG_PIN, LOW);

      unsigned long duration = pulseIn(HC_SR04_ECHO_PIN, HIGH);
      unsigned long distance = (duration / 2) / 29.1;

      this->setDistance(distance);
      PRINTS("\nRead Distance in cm: ");
      PRINTD(this->getDistance());

    } else {
      return;
    }
  }

  /**
     * Reads the tilt sensor status.
     * @param readSensorInterval Time interval between successive readings.
     */
  void readTiltDetector(unsigned long readSensorInterval = 2500) {
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
     * Sends the robot's current data (emotional state, brightness) over Serial.
     */
  void sendRobotData() {
    String emotionalState = getEmotionalStateAsString();
    int brightness = this->getBrightVal();
    float temperature = this->getTemperatureVal();
    bool isStanding = this->isVerticalStand();
    bool isMoving = this->isMoving();
    // Add more data if needed

    // Create a comma-separated string of values
    String dataString =
      emotionalState + "," + brightness + "," + temperature + "," + isStanding + "," + isMoving;

    // Send the data string over serial
    Serial.println(dataString);
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
    // PRINTS("\n[STOP]");
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
    // PRINTS("\nStop vibrate");
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
      this->_blinkInterval = random(1000, 2301);
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
      PRINT("\nEmotional State: ", this->getEmoState());
      _previousEmoState = SERI::HAPPY;
      _lastEmotionChangeMillis = millis();  // Update time
    } else {
      this->showSpeechlessRandomly();
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
      PRINT("\nEmotional State: ", this->getEmoState());
      _previousEmoState = SERI::SAD;
      _lastEmotionChangeMillis = millis();  // Update time
    } else {
      this->showSpeechlessRandomly();
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
      PRINT("\nEmotional State: ", this->getEmoState());
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
      PRINT("\nEmotional State: ", this->getEmoState());
    } else {
      return;
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
      PRINT("\nEmotional State: ", this->getEmoState());
      _previousEmoState = SERI::EXCITED;
      _lastEmotionChangeMillis = millis();  // Update time
    } else {
      this->showSpeechlessRandomly();
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
      PRINT("\nEmotional State: ", this->getEmoState());
    } else {
      return;
    }
  }

  /**
   * Displays a 'Cold' emotion on the LED matrix with an animated mouth.
   *
   * This function changes the emotional state to 'COLD' and initiates
   * an animation for the mouth, giving an impression of shivering or
   * teeth chattering. The eyes are set to a fixed expression, and the
   * mouth's animation toggles between two states at regular intervals
   * defined by animatedInterval.
   *
   * @param animatedInterval Duration between animation states for the mouth
   *                         in milliseconds. The default is 200ms.
   */
  void showCold(unsigned long animatedInterval = 200) {
    // Check and update emotional state if not already COLD
    if (this->getEmoState() != SERI::COLD) {
      this->setEmoState(SERI::COLD);
      PRINT("\nEmotional State: ", this->getEmoState());
      mx.clear();
      // EYE_COLD
      mx.setColumn(1, 36);
      mx.setColumn(2, 36);
    }

    // Animation control using time intervals
    unsigned long _curMillis = millis();
    if (_curMillis - this->_preColdAnimatedMillis > animatedInterval) {
      this->_preColdAnimatedMillis = _curMillis;
      animateMouthCold();
    }
  }

  /**
   * Displays a 'Hot' emotion on the LED matrix with an animated mouth.
   *
   * This function changes the emotional state to 'HOT' and initiates
   * an animation for the mouth to give an impression of panting or
   * being overheated. The eyes are set to a fixed expression, and the
   * mouth's animation toggles between two states at regular intervals
   * defined by animatedInterval.
   *
   * @param animatedInterval Duration between animation states for the mouth
   *                         in milliseconds. The default is 200ms.
   */
  void showHot(unsigned long animatedInterval = 600) {
    // Check and update emotional state if not already HOT
    if (this->getEmoState() != SERI::HOT) {
      this->setEmoState(SERI::HOT);
      PRINT("\nEmotional State: ", this->getEmoState());
      mx.clear();
      // EYE_HOT
      mx.setColumn(1, 36);
      mx.setColumn(2, 36);
    }

    // Animation control using time intervals
    unsigned long _curMillis = millis();
    if (_curMillis - this->_preHotAnimatedMillis > animatedInterval) {
      this->_preHotAnimatedMillis = _curMillis;
      this->animateMouthHot();
    }
  }

  /**
   * Displays a 'Satisfied' emotion on the LED matrix with animated eyes and mouth.
   * The animation changes the shape of the eyes and mouth over time.
   * @param animatedInterval Duration between animation states for the mouth and eyes
   *                         in milliseconds. The default is 1000ms.
   */
  void showSatisfied(unsigned long animatedInterval = 1000) {
    if (this->getEmoState() != SERI::SATISFIED) {
      mx.clear();
      this->setEmoState(SERI::SATISFIED);
      PRINT("\nEmotional State: ", this->getEmoState());
      _previousEmoState = SERI::EXCITED;
      _lastEmotionChangeMillis = millis();  // Update time
    } else {
      this->showSpeechlessRandomly();
    }

    // Animation control using time intervals
    unsigned long _curMillis = millis();
    if (_curMillis - this->_preSatisfiedMillis > animatedInterval) {
      this->_preSatisfiedMillis = _curMillis;
      this->animateSatisfied();
    }
  }

  /**
   * Displays a 'Dry' emotion on the LED matrix.
   * This function uses the LED matrix to create a face representation
   * of the 'Dry' emotional state.
   */
  void showDry() {
    if (this->getEmoState() != SERI::DRY) {
      mx.clear();
      // EYE_DRY
      mx.setColumn(1, 36);
      mx.setColumn(2, 36);
      // MOUTH_DRY
      mx.setColumn(5, 60);
      mx.setColumn(6, 60);

      this->setEmoState(SERI::DRY);
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

// Duration to display temp/humidity emotion
unsigned long emotionDisplayTime = 5000;
unsigned long lastEmotionChange = -5000;
bool tempHumidityEmotionDisplayed = true;
bool tempEmotionDisplayed = true;  // Start with temperature emotion

SERI mySeri = SERI();
bool animated = false;

void setup() {
  Serial.begin(9600);
  PRINTS("\n[START PROGRAM]");

  mySeri.begin();
  mySeri.calibrateSensor();
  randomSeed(analogRead(0));  // Ensure random number generator is properly seeded.

#if SEND_DATA
  mySeri.establishContact();
#endif
}

void loop() {

  // Smooth out the transition for reset.
  if (!animated) {
    mySeri.startAnimation();
    animated = true;
    delay(500);
  }

  unsigned long curMillis = millis();

  // Revert to light-based emotions after a delay
  if (tempHumidityEmotionDisplayed
      && curMillis - lastEmotionChange >= emotionDisplayTime) {
    tempHumidityEmotionDisplayed = false;
    if (mySeri.isVerticalStand() && !mySeri.isInteruptEvent()) {
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
  }

  if (mySeri.getEmoState() != SERI::SCARED && !mySeri.isVerticalStand()) {
    mySeri.showScared();
    mySeri.vibrate(2000);
  } else {
    mySeri.stopVibrate();
  }

  mySeri.readDistanceToObstacle(500);
  mySeri.updateBlink();
  mySeri.readLightAmbient();
  mySeri.readTemperature();
  mySeri.readHumidity();
  mySeri.readTiltDetector();

  if (mySeri.getDistance() < 5) {
    mySeri.goBackward();
  } else {
    mySeri.stop();
  }

  // Alternate between temperature and humidity-based emotions
  if (curMillis - lastEmotionChange >= emotionDisplayTime) {

    // Randomly choose to display temperature or humidity emotion
    if (random(2) == 0) {
      tempEmotionDisplayed = !tempEmotionDisplayed;
    }

    // Display the chosen emotion
    if (tempEmotionDisplayed) {
      if (mySeri.getTemperatureVal() < 10) {
        mySeri.showCold();
      } else if (mySeri.getTemperatureVal() > 30) {
        mySeri.showHot();
      }
    } else {
      if (mySeri.getHumidityVal() < 30) {
        mySeri.showDry();
      } else if (mySeri.getHumidityVal() > 70) {
        mySeri.showSatisfied();
      }
    }
    lastEmotionChange = curMillis;
    tempHumidityEmotionDisplayed = true;
  }

#if SEND_DATA
  mySeri.sendRobotData();
#endif
}
