# ServiceBot

Developed by Karanraj Chauhan and Anand Sanmukhani

Working components:
1. MSP432P401R microcontroller
2. Pixy (CMUcam5) cam
3. HC-SR04 ultrasonic sensor
4. TB6612FNGG motor driver
5. Adafruit Bluefruit LE UART Friend module 

This code is for the developed device, the primary purpose of which is to serve the purpose of a service dog, to help people who have disabilities such as visual impairment, hearing impairments, mental illnesses (such as posttraumatic stress disorder (PTSD)), seizure disorder, mobility impairment, and diabetes.This device would track and follow the user from a safe distance, as they walk around. If the patient collapses, the device would then take some action.

The secondary purpose is to function as a remotely controlled device, so that it can be sent to places where it might be unsafe for humans to go. We control the bot using the accelerometer on an Android phone (using the application Bluefruit LE). The bot moves forward, backward, left and right when the phone is tilted forward, backward, left and right respectively.
