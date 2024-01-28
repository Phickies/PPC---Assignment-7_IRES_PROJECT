# PPC---Assignment-7_SERI_PROJECT
Description
SERI, or Support Emotional Robot Intelligence, is an innovative robot designed to interact seamlessly with its surroundings. Equipped with numerous types of sensors, SERI is adept at perceiving environmental nuances as well as physical movements occurring both in and around it. Upon processing this data, SERI expresses emotions through a digital facial display or responds physically, enhancing user engagement. Each reaction is accompanied by a brief sound signal, resembling a voice, which users can personalize to their preference.

Furthermore, SERI boasts wireless connectivity with electronic devices such as computers and phones. This feature allows users to monitor the robot’s status, control and customize its functions remotely.

The primary objective of SERI is to assist individuals with mental health challenges and those seeking support in overcoming detrimental habits. It aims to contribute significantly to healthcare, enhancing the quality of life. Users have the flexibility to modify SERI’s functions to align with their specific requirements using Arduino IDE.
Product

Specification

Problems and limitations
The current Arduino chip is a big limitation in processing cycles, only 16MHz clock cycle and there are only 32kB storage capability. Therefore, we have to limit the cycle of checking sensing data, and reduce the processing time for unnecessary calculations. This also reduces the responsive performance of the robot with the environment around. 

Furthermore, Arduino only limits it to 5V, this is not enough current to control the actuator and multiple LEDs. Therefore we need to have 2 different power supplies. One with higher voltage and current, 9V battery to control all the actuators, LED, speaker and the other one with a lower in voltage and current to microcontroller and sensors.

Code
Arduino
Processing
Schematics
