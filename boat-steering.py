#!/usr/bin python3

"""
The daemon responsible for moving the actuator in response to a turn or press
of the rotary encoder.  It turns infinitely in either direction.

The knob uses two GPIO pins and we need some extra logic to decode it. The
button we can just treat like an ordinary button. Rather than poll
constantly, we use threads and interrupts to listen on all three pins in one
script.

Yuto Shinagawa forked Andrew Dupont's speaker volume control project
(https://gist.github.com/savetheclocktower) and modified it for controlling
CAN bus-controlled servo actuators
"""

import os
import signal
import subprocess
import sys
import threading
import time
import can
import datetime

from RPi import GPIO
from queue import Queue

DEBUG = False

# SETTINGS
# ========

# Pins for station 1 steering encoder (BCM numbering).
GPIO_1S = 17 # take control switch
GPIO_1A = 23 # encoder ch A
GPIO_1B = 24 # encoder ch B

# Pins for station 2 steering encoder (BCM numbering).
GPIO_2S = 27 # take control switch
GPIO_2A = 5 # encoder ch A
GPIO_2B = 6 #encoder ch B

# The pin that the knob's button is hooked up to. If you have no button, set
# this to None.
GPIO_BUTTON = None

# The minimum and maximum steering angle in mm
STROKE_MIN = 0 #mm
STROKE_MAX = 150 #mm
STROKE_RESET = 75 #mm

# The amount you want one click of the knob to increase or decrease the command stroke
STROKE_INCREMENT = 1 #mm

# Controller will command the servo to move if the feedback position deviates from the
# command reference by more than the threshold below
MOTION_THRESH = 2 #mm

# Send SAEJ1939 actuator control messages (ACM) at fixed rate even if the
# actuator isn't being actively moved so that we continue to get periodic
# actuator feedback messages (AFM).
# Set the motion bit within the ACM to 1 only when the actuator needs to be
# moved to a different position; keep 0 all other times
CAN1_ID = 0x18ef1300 #18=priority 6; ef=actuator cmd msg; 13=destination addr; 00=source (pi) addr
CAN2_ID = 0x18ef1400
CAN_SEND_HZ = 10

# CAN MSG bit to engineering unit scaling factors
MM_BIT = 0.1
AMP_BIT = 0.1
PERC_BIT = 5

# CONFIGURABLE ACTUATOR SETTINGS
CURRENT_AMPS = 20
SPEED_PERC = 25


# (END SETTINGS)


# When the knob is turned, the callback happens in a separate thread. If
# those turn callbacks fire erratically or out of order, we'll get confused
# about which direction the knob is being turned, so we'll use a queue to
# enforce FIFO. The callback will push onto a queue, and all the actual
# actuator moving will happen in the main thread.
QUEUE = Queue()

# When we put something in the queue, we'll use an event to signal to the
# main thread that there's something in there. Then the main thread will
# process the queue and reset the event. If the knob is turned very quickly,
# this event loop will fall behind, but that's OK because it consumes the
# queue completely each time through the loop, so it's guaranteed to catch up.
EVENT = threading.Event()

GPIO.setmode(GPIO.BCM)

def debug(str):
  if not DEBUG:
    return
  print(str)

class TakeControlSwitch:

  def __init__(self, gpio1S, gpio1A, gpio1B, gpio2S, gpio2A, gpio2B, callback):
    self.station_controlling = 1
    self.callback = callback
    self.gpio1S = gpio1S
    self.gpio1A = gpio1A
    self.gpio1B = gpio1B
    self.gpio2S = gpio2S
    self.gpio2A = gpio2A
    self.gpio2B = gpio2B
    self.encoder = RotaryEncoder(gpio1A, gpio1B, callback=callback)

    GPIO.setup(self.gpio1S, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    GPIO.setup(self.gpio2S, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    GPIO.add_event_detect(self.gpio1S, GPIO.FALLING, self._give_S1_control)
    GPIO.add_event_detect(self.gpio2S, GPIO.FALLING, self._give_S2_control)

  def _give_S1_control(self, channel):
    if self.station_controlling != 1:
      self.encoder.remove_events()
      self.encoder = RotaryEncoder(self.gpio1A, self.gpio1B, callback=self.callback)
      self.station_controlling = 1
      print('station 1 took control')

  def _give_S2_control(self, channel):
    if self.station_controlling != 2:
      self.encoder.remove_events()
      self.encoder = RotaryEncoder(self.gpio2A, self.gpio2B, callback=self.callback)
      self.station_controlling = 2
      print('station 2 took control')

  def destroy(self):
    GPIO.setmode(GPIO.BCM)
    GPIO.remove_event_detect(self.gpio1S)
    GPIO.remove_event_detect(self.gpio2S)
    GPIO.cleanup()


class RotaryEncoder:
  """
  A class to decode mechanical rotary encoder pulses.
  Ported to RPi.GPIO from the pigpio sample here: 
  http://abyz.co.uk/rpi/pigpio/examples.html
  """
  
  def __init__(self, gpioA, gpioB, callback=None, buttonPin=None, buttonCallback=None):
    """
    Instantiate the class. Takes three arguments: the two pin numbers to
    which the rotary encoder is connected, plus a callback to run when the
    switch is turned.
    
    The callback receives one argument: a `delta` that will be either 1 or -1.
    One of them means that the dial is being turned to the right; the other
    means that the dial is being turned to the left. I'll be damned if I know
    yet which one is which.
    """
    
    self.lastGpio = None
    self.gpioA    = gpioA
    self.gpioB    = gpioB
    self.callback = callback
    
    self.gpioButton     = buttonPin
    self.buttonCallback = buttonCallback
    
    self.levA = 0
    self.levB = 0
    
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(self.gpioA, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    GPIO.setup(self.gpioB, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    
    GPIO.add_event_detect(self.gpioA, GPIO.BOTH, self._callback)
    GPIO.add_event_detect(self.gpioB, GPIO.BOTH, self._callback)
    
    if self.gpioButton:
      GPIO.setup(self.gpioButton, GPIO.IN, pull_up_down=GPIO.PUD_UP)
      GPIO.add_event_detect(self.gpioButton, GPIO.FALLING, self._buttonCallback, bouncetime=500)
    
  def remove_events(self):
    GPIO.remove_event_detect(self.gpioA)
    GPIO.remove_event_detect(self.gpioB)

  def destroy(self):
    self.remove_events()
    GPIO.cleanup()
    
  def _buttonCallback(self, channel):
    self.buttonCallback(GPIO.input(channel))
    
  def _callback(self, channel):
    level = GPIO.input(channel)
    if channel == self.gpioA:
      self.levA = level
    else:
      self.levB = level
      
    # Debounce.
    if channel == self.lastGpio:
      return
    
    # When both inputs are at 1, we'll fire a callback. If A was the most
    # recent pin set high, it'll be forward, and if B was the most recent pin
    # set high, it'll be reverse.
    self.lastGpio = channel
    if channel == self.gpioA and level == 1:
      if self.levB == 1:
        self.callback(1)
    elif channel == self.gpioB and level == 1:
      if self.levA == 1:
        self.callback(-1)

class ActuatorError(Exception):
  pass

class Actuator:
  """
  A wrapper API for interacting with the actuator.
  """
  MIN = STROKE_MIN
  MAX = STROKE_MAX
  INCREMENT = STROKE_INCREMENT
  
  def __init__(self):
    # TODO grab actuator position at startup
    self.last_pos = STROKE_RESET
    self.pos_cmd = self.last_pos
    self._motion_enable = 1

  def extend(self):
    """
    Extends the actuator by one increment.
    """
    return self.change(self.INCREMENT)

  def retract(self):
    """
    Retracts the actuator by one increment.
    """
    return self.change(-self.INCREMENT)

  def change(self, delta):
    p = self.pos_cmd + delta
    p = self._constrain(p)
    return self.set_actuator(p)

  def set_actuator(self, p):
    """
    Sets actuator position to a specific value.
    """
    self.pos_cmd = self._constrain(p)
    # self._motion_enable = 1
    return self.pos_cmd

  def reset(self, p):
    """
    Reset actuator to predefined value
    """
    self.pos_cmd = self.constrain(p)
    return self.pos_cmd

  # Limit the position command to between our minimum and maximum.
  def _constrain(self, p):
    if p < self.MIN:
      return self.MIN
    if p > self.MAX:
      return self.MAX
    return p

  @property
  def motion_enable(self):
    return self._motion_enable

  @motion_enable.setter
  def motion_enable(self, value):
    self._motion_enable = value
  
def process_encoder_events():
  while True:
    # This is the best way I could come up with to ensure that this script
    # runs indefinitely without wasting CPU by polling. The main thread will
    # block quietly while waiting for the event to get flagged. When the knob
    # is turned we're able to respond immediately, but when it's not being
    # turned we're not looping at all.
    #
    # The 1200-second (20 minute) timeout is a hack; for some reason, if I
    # don't specify a timeout, I'm unable to get the SIGINT handler above to
    # work properly. But if there is a timeout set, even if it's a very long
    # timeout, then Ctrl-C works as intended. No idea why.
    EVENT.wait(1200)
    
    consume_queue()
    EVENT.clear()

def can_rx_loop():
  while True:
    for msg in bus:
      frame = ''
      for data in msg.data:
        frame += bin(data)[2:].zfill(8)[::-1]
      bin_pos = frame[0:14] #first 14 bits is measured position [mm]
      bin_curr = frame[14:23] #9 bits, measured current [A]
      bin_speed = frame[23:28] #5 bits, running speed [%]
      bin_volterr = frame[28:30] #2 bits, voltage, 00=in range, 01=too lo, 10=too hi
      bin_temperr = frame[30:32] #2 bits, temp, 00=in range, 01=too lo, 10=too hi
      bin_motflg = frame[32:33] #1 bit, actuator in motion flag
      bin_overflg = frame[33:34] #1 bit, current exceeds limit set
      bin_bkdrvflg = frame[34:35] #1 bit, servo backdrive detected
      bin_paramflg = frame[35:36] #1 bit, commanded param out of range
      bin_satflag = frame[36:37] #1 bit, actuator exceeding 90% max capability
      bin_fatalflag = frame[37:38] #1 bit, actuator needs servicing
      pos_mm = int(bin_pos[::-1], 2)*MM_BIT
      curr_amps = int(bin_curr[::-1], 2)*AMP_BIT
      speed_perc = int(bin_speed[::-1], 2)*PERC_BIT

      pos_cmd = a.pos_cmd
      timestamp = datetime.datetime.now().strftime("%H:%M:%S.%f")[:-4]
      #TODO print id of the actuator the message came from
      print("{}, cmd={:.1f}mm, pos={:.1f}mm, curr={:.1f}A, duty={}%, " \
            "voltage err={}, temp error={}, " \
            "motion={}, overload={}, backdrive={}, " \
            "param={}, saturation={}, fatal={}".format(
              timestamp, pos_cmd, pos_mm, curr_amps, speed_perc,
              bin_volterr, bin_temperr, bin_motflg, bin_overflg, 
              bin_bkdrvflg, bin_paramflg, bin_satflag, bin_fatalflag))

      # Note, a.motion_enable is written in this thread and accessed in another
      # If actuator is overloaded, reset by setting motion_enable = 0
      if abs(pos_cmd - pos_mm) > MOTION_THRESH and int(bin_overflg) == 0:
        a.motion_enable = 1
      else:
        a.motion_enable = 0
    time.sleep(1)

def on_press(value):
  a.center()
  print("Reset actuator to: {}".format(a.pos_cmd))
  EVENT.set()

# This callback runs in the background thread. All it does is put turn
# events into a queue and flag the main thread to process them. The
# queueing ensures that we won't miss anything if the knob is turned
# extremely quickly.
def on_turn(delta):
  QUEUE.put(delta)
  EVENT.set()
  
def consume_queue():
  while not QUEUE.empty():
    delta = QUEUE.get()
    handle_delta(delta)

def handle_delta(delta):
  if delta == 1:
    pos = a.extend()
  else:
    pos = a.retract()

def on_exit(a, b):
  print("Exiting...")
  tcs.destroy()
  sys.exit(0)


if __name__ == "__main__":
  
  # gpioA = GPIO_A
  # gpioB = GPIO_B
  # gpioButton = GPIO_BUTTON
  
  # TODO: create a bus instance using 'with' statement,
  # this will cause bus.shutdown() to be called on the block exit;
  # many other interfaces are supported as well (see documentation)
  bus = can.interface.Bus(channel='can0', bustype='socketcan')

  encoder_thread = threading.Thread(target=process_encoder_events)
  encoder_thread.start()

  can_rx_thread = threading.Thread(target=can_rx_loop)
  can_rx_thread.start()

  a = Actuator()

  # define control transfer switch class;
  # in int, create event handlers for station 1 & 2  "take control switch"
  # define callback functions that will destroy and recreate new RotaryEncoder object upon control transfer


  # debug("Reading rotary encoder from pins {} and {}".format(gpioA, gpioB))

  # if gpioButton != None:
  #   debug("Reading actuator reset position command from pin {}".format(gpioButton))

  tcs = TakeControlSwitch(GPIO_1S, GPIO_1A, GPIO_1B, GPIO_2S, GPIO_2A, GPIO_2B, callback=on_turn)

  signal.signal(signal.SIGINT, on_exit)

  bin_current = bin(int(CURRENT_AMPS/AMP_BIT))[2:].zfill(9)[::-1]
  bin_speed = bin(int(SPEED_PERC/PERC_BIT))[2:].zfill(5)[::-1]

  time_after_loop = time.time() # initialization
  while True:
    time_before_loop = time.time()
    if time_before_loop - time_after_loop >= (1./CAN_SEND_HZ):
      real_frequency = time_before_loop - time_after_loop
      bin_pos = bin(int(a.pos_cmd/MM_BIT))[2:].zfill(14)[::-1]

      # TODO if overload bit is high in actuator status message, then
      # we have to reset motion_enable false before back true
      motion_enable = bin(int(a.motion_enable))[2:].zfill(1)[::-1]

      # add to this the bits to define current, speed, movement.
      command = bin_pos + bin_current + bin_speed + motion_enable + \
                "00000000000000000000000000000000000"

      com_hex = ''

      for i in range(0,8):
        #reverse order of bits
        com = command[i*8:(i+1)*8][::-1]
        # Convert bytes into one big string of hex
        com_hex+=hex(int(com,2))[2:].zfill(2)

      message1 = can.Message(arbitration_id=CAN1_ID, is_extended_id=True,
                            data=bytearray.fromhex(com_hex))
      message2 = can.Message(arbitration_id=CAN2_ID, is_extended_id=True,
                            data=bytearray.fromhex(com_hex))
      bus.send(message1, timeout=0.2)
      bus.send(message2, timeout=0.2)

      time_after_loop = time.time()
