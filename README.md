# Steering a House Boat with CAN-bus Linear Actuators 

## Overview
This repository contains Python code written to steer Brandon's houseboat. The steering system consists of the following components:
- A Raspberry pi running the main application, boat-steering.py in a Poetry virtual environment
- Two Thomson Linear Electrak HD actuators with SAE J1939 CAN bus interfaces, one each for the port and starboard rudders.
- Two control stations, one on the upper deck and another on the lower deck, each with the following:
  - A momentary switch to allow the pilot at a station to take control from the other station
  - An LED to indicate whether the station is in control
  - A Rotary encoder to receive the pilot's steering inputs
  - A 24 segment LED bargraph to indicate the servo's feedback position
   
## Origins
The application code can trace its origins to prior work by two really talented developers:
- Michael Anderson's early prototype from circa 2019, which is the only thing we were able to locate following the accidental destruction of the RPi SD card on which the only copy of the code was stored. Don't judge. Maybe Github didn't exist back in 2019.
- Andrew Dupont's speaker volume control application (https://gist.github.com/savetheclocktower), whose code to interface with the rotary encoder I used as a starting point for this application. I forked from his repository.  

## Code Structure
1. First, the application instantiates actuator objects, one for each physical actuator on the boat.  It has getters and setters for both the incoming feedback position and outgoing position commands. Actuator objects are stored in a dictionary whose keys are the actuator CAN IDs. At this stage, both states are initialized to placeholder values or None, as we have yet to establish communication with the actuators via CAN.   
2. Next, the application instantiates a CAN object, which starts separate threads for the CAN Rx and CAN Tx loops. The Rx loop looks at the CAN ID of the message sender and after parsing the actuator feedback message, will set the feedback position property for the actuator that has the matching CAN ID. The Tx loop sends to each CAN ID in the dictionary, actuator control messages at 10 Hz.  Note that I don't start separate Tx threads for each actuator, as it seemed to degrade the actuator's responsiveness to encoder inputs.  Instead I send consecutive messages to all the actuators in the dictionary within one iteration cycle. Messaging to disparate actuators are synchronized in this fashion.
3. After a 0.5s pause to give the actuators a chance to start responding, the application instantiates a rudder object, which is just an abstraction to map linear actuator positions in mm to/from rudder position in degrees, which in turn is what the event handler (below) sets when the pilot turns the encoder knob.  Each time this happens, the rudder object calls the actuator position command setters for both actuator objects, which move in unison for the most part. The rudder mapping math does make provisions for a fixed toe in angle between the port and starboard rudders.  We implemented this in hopes of reducing wander during cruise.
4. The application starts an event handler thread to process any events that the encoder object adds to the queue. Processing events this way ensures that the rudder setter doesn't miss consecutive steps, even if they happen in quick succession.  I took #4 and #5 below from Andrew Dupont's volume control application.  
5. The application then instantiates a "take control switch" object, which is just a wrapper around the encoder object. Each time the wrapper detects a button press to transfer control from one station to another, it destroys the old encoder object and creates a new one.  I decided to do it this way because I didn't want the RPi to be listening for events on 4 GPIO pins (a pair for each encoder) simultaneously as I thought it would degrade performance.  Creating an encoder object ad hoc with the 2 GPIO pins specific to that encoder effectively reduces the number of event detection threads to 2.
6. Lastly, the application creates an LED bar graph object, which starts a thread to continuously get rudder position to paint a 24-segment LED display.

We deployed this system last weekend in preparation for an upcoming houseboat trip.  I verified that it handles gracefully all the obvious failure modes including single actuator operation, temporary actuator power loss and communication dropout, and actuators being powered off during application startup.  I've created github issues for all the known issues and vulnerabilities.

Happy boating!