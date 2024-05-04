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
import board
from adafruit_ht16k33.bargraph import Bicolor24
import copy

from RPi import GPIO
from queue import Queue

DEBUG = False

# SETTINGS
# ========

# Pins for station 1 steering encoder (BCM numbering).
GPIO_1S = 26 # take control switch
GPIO_1A = 5 # encoder ch A
GPIO_1B = 6 # encoder ch B
GPIO_1LED = 22 #station 1 in control LED

# Pins for station 2 steering encoder (BCM numbering).
GPIO_2S = 16 # take control switch
GPIO_2A = 23 # encoder ch A
GPIO_2B = 24 #encoder ch B
GPIO_2LED = 27 #station 2 in control LED

# The pin that the knob's button is hooked up to. If you have no button, set
# this to None.
GPIO_BUTTON = None

# The minimum and maximum actuator stroke in mm
STROKE_MIN = 10 #mm
STROKE_MAX = 130 #mm
STROKE_RESET = 70 #mm

# Controller will command the servo to move if the feedback position deviates from the
# command reference by more than the threshold below
MOTION_THRESH = 2 #mm

# Send SAEJ1939 actuator control messages (ACM) at fixed rate even if the
# actuator isn't being actively moved so that we continue to get periodic
# actuator feedback messages (AFM).
# Set the motion bit within the ACM to 1 only when the actuator needs to be
# moved to a different position; keep 0 all other times
CAN1_ID = 0x13
CAN2_ID = 0x14
CAN_SEND_HZ = 10

# CAN MSG bit to engineering unit scaling factors
MM_BIT = 0.1
AMP_BIT = 0.1
PERC_BIT = 5

# CONFIGURABLE ACTUATOR SETTINGS
CURRENT_AMPS = 20 # max allowable current draw
SPEED_PERC = 25 # target speed as a % of max speed

# The amount you want one click of the knob to increase or decrease the rudder angle
RUD_INCR = 1 #deg
RUD_MIN = -45 #deg
RUD_MAX = 45 #deg
TOE_IN = 0 #deg

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
  GPIO_1S = GPIO_1S # take control switch
  GPIO_1A = GPIO_1A # encoder ch A
  GPIO_1B = GPIO_1B # encoder ch B
  GPIO_1LED = GPIO_1LED # station 1 in control LED

  # Pins for station 2 steering encoder (BCM numbering).
  GPIO_2S = GPIO_2S # take control switch
  GPIO_2A = GPIO_2A # encoder ch A
  GPIO_2B = GPIO_2B # encoder ch B
  GPIO_2LED = GPIO_2LED # station 2 in control LED

  def __init__(self, callback):

    self.callback = callback

    GPIO.setup(self.GPIO_1S, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    GPIO.setup(self.GPIO_2S, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    GPIO.setup(self.GPIO_1LED, GPIO.OUT)
    GPIO.setup(self.GPIO_2LED, GPIO.OUT)
    GPIO.add_event_detect(self.GPIO_1S, GPIO.FALLING, self._give_S1_control, bouncetime=200)
    GPIO.add_event_detect(self.GPIO_2S, GPIO.FALLING, self._give_S2_control, bouncetime=200)

    # Initialize with station 1 in control
    self.station_controlling = 1
    self.encoder = RotaryEncoder(self.GPIO_1A, self.GPIO_1B, callback=callback)
    GPIO.output(self.GPIO_1LED, 0)
    GPIO.output(self.GPIO_2LED, 1)

  def _give_S1_control(self, channel):
    if self.station_controlling != 1:
      self.encoder.remove_events()
      self.encoder = RotaryEncoder(self.GPIO_1A, self.GPIO_1B, callback=self.callback)
      self.station_controlling = 1
      # Relay turns on LED when channel is driven low
      GPIO.output(self.GPIO_1LED, 0)
      GPIO.output(self.GPIO_2LED, 1)
      print('station 1 in control')

  def _give_S2_control(self, channel):
    if self.station_controlling != 2:
      self.encoder.remove_events()
      self.encoder = RotaryEncoder(self.GPIO_2A, self.GPIO_2B, callback=self.callback)
      self.station_controlling = 2
      # Relay turns on LED when channel is driven low
      GPIO.output(self.GPIO_1LED, 1)
      GPIO.output(self.GPIO_2LED, 0)
      print('station 2 in control')

  def destroy(self):
    self.encoder.destroy()
    GPIO.remove_event_detect(self.GPIO_1S)
    GPIO.remove_event_detect(self.GPIO_2S)

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

class CAN:

  def __init__(self, actuatorMap):
    self.actuatorMap = actuatorMap
    # TODO: create a bus instance using 'with' statement,
    # this will cause bus.shutdown() to be called on the block exit;
    # many other interfaces are supported as well (see documentation)
    self.bus = can.interface.Bus(channel='can0', bustype='socketcan')
    self.can_rx_thread = threading.Thread(target=self.can_rx_loop)
    self.can_rx_thread.start()
    self.can_tx_thread = threading.Thread(target=self.can_tx_loop)
    self.can_tx_thread.start()

  def can_rx_loop(self):
    try:
      print('starting can_rx_loop thread')
      while True:
        for msg in self.bus:
          source_id = msg.arbitration_id & 0XFF
          if source_id not in self.actuatorMap:
            continue
          
          timestamp = datetime.datetime.now()
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

          #return actuator object corresponding to the sender's id
          a = self.actuatorMap[source_id]
          a.pos_mm = pos_mm
          a.overflg = int(bin_overflg)

          if a.lst_update is None:
            a.pos_cmd = pos_mm
          a.lst_update = timestamp
          timestr = timestamp.strftime("%H:%M:%S.%f")[:-4]
          print("{}, id={}, cmd={:.1f}mm, pos={:.1f}mm, curr={:.1f}A, duty={}%, " \
                "voltage err={}, temp error={}, " \
                "motion={}, overload={}, backdrive={}, " \
                "param={}, saturation={}, fatal={}".format(
                  timestr, hex(msg.arbitration_id), a.pos_cmd, pos_mm, curr_amps, speed_perc,
                  bin_volterr, bin_temperr, bin_motflg, bin_overflg,
                  bin_bkdrvflg, bin_paramflg, bin_satflag, bin_fatalflag))

        time.sleep(1)
    except Exception as e:
      print(f"Exception in can_rx_loop: {type(e).__name__}, {e}")

  def can_tx_loop(self):
    print('starting can_tx_loop thread')
    bin_current = bin(int(CURRENT_AMPS/AMP_BIT))[2:].zfill(9)[::-1]
    bin_speed = bin(int(SPEED_PERC/PERC_BIT))[2:].zfill(5)[::-1]

    time_after_loop = time.time() # initialization
    while True:
      try:
        time_before_loop = time.time()
        #TODO: consider creating separate tx threads for each actuator
        if time_before_loop - time_after_loop >= (1./CAN_SEND_HZ):
          for id,a in self.actuatorMap.items():
            # If overload bit is high in actuator status message, then
            # we have to reset motion_enable false before back true
            motion_enable = bin(int(0))[2:].zfill(1)[::-1]
            if a.pos_mm is not None:
              if abs(a.pos_cmd - a.pos_mm) > MOTION_THRESH and \
                a.overflg == 0:
                  motion_enable = bin(int(1))[2:].zfill(1)[::-1]

            bin_pos = bin(int(a.pos_cmd/MM_BIT))[2:].zfill(14)[::-1]

            # add to this the bits to define current, speed, movement.
            command = bin_pos + bin_current + bin_speed + motion_enable + \
                  "00000000000000000000000000000000000"

            com_hex = ''

            for i in range(0,8):
              #reverse order of bits
              com = command[i*8:(i+1)*8][::-1]
              # Convert bytes into one big string of hex
              com_hex+=hex(int(com,2))[2:].zfill(2)

            #18=priority 6; ef=actuator cmd msg; 13=destination addr; 00=source (pi) addr
            ARBIT_ID = id << 8 | 0x18ef0000

            message = can.Message(arbitration_id=ARBIT_ID, is_extended_id=True,
                                    data=bytearray.fromhex(com_hex))
            self.bus.send(message, timeout=0.2)

          time_after_loop = time.time()
      except can.CanError as e:
        print(f"CAN Error: {e}, retrying...")
        time.sleep(1)  # Wait a bit before retrying
      except Exception as e:
        print(f"Exception in can_tx_loop: {type(e).__name__}, {e}")

class ActuatorError(Exception):
  pass


class Actuator:
  """
  A wrapper API for interacting with the actuator.
  """
  MIN = STROKE_MIN
  MAX = STROKE_MAX
  RESET = STROKE_RESET

  def __init__(self, nodeID):
    # TODO grab actuator position at startup
    self.nodeID = nodeID
    self._pos_cmd = self.RESET #target position to drive actuator to
    self._pos_mm = None # actual position reported in can msg
    self._overflg = 0
    self._lst_update = None

  def extend(self, delta):
    """
    Extends the actuator by one increment.
    """
    return self.change(delta)

  def retract(self, delta):
    """
    Retracts the actuator by one increment.
    """
    return self.change(-delta)

  def change(self, delta):
    p = self._pos_cmd + delta
    p = self._constrain(p)
    return self.set_actuator(p)

  def set_actuator(self, p):
    """
    Sets actuator position to a specific value.
    """
    self._pos_cmd = self._constrain(p)
    return self._pos_cmd

  def reset(self, p):
    """
    Reset actuator to predefined value
    """
    self._pos_cmd = self.constrain(p)
    return self._pos_cmd

  # Limit the position command to between our minimum and maximum.
  def _constrain(self, p):
    if p < self.MIN:
      return self.MIN
    if p > self.MAX:
      return self.MAX
    return p

  @property
  def pos_cmd(self):
    return self._pos_cmd

  @pos_cmd.setter
  def pos_cmd(self, value):
    self._pos_cmd = value

  @property
  def pos_mm(self):
    return self._pos_mm

  @pos_mm.setter
  def pos_mm(self, value):
    self._pos_mm = value

  @property
  def overflg(self):
    return self._overflg

  @overflg.setter
  def overflg(self, value):
    self._overflg = value

  @property
  def lst_update(self):
    return self._lst_update

  @lst_update.setter
  def lst_update(self, value):
    self._lst_update = value


class Rudder:
  RUD_MIN = RUD_MIN
  RUD_MAX = RUD_MAX
  TOE_IN = TOE_IN

  def __init__(self, a1, a2):
    self.a1 = a1 # port actuator
    self.a2 = a2 # starboard actuator
    self._pos_deg = self.get_rudder(a1, a2)
    self._pos_cmd = self._pos_deg
    print('Based on current actuator position, initialized ' 
          'rudder command at: {}'.format(self.pos_deg))

  def change(self, delta):
    pos_cmd = copy.copy(self._pos_cmd)
    if pos_cmd is not None:
      p = self._pos_cmd + delta
      p = self._constrain(p)
      self.set_rudder(p)

  def get_rudder(self, a1, a2):
    self.pos1_deg = None
    self.pos2_deg = None
    self._pos_deg = None
    if a1 is not None:
      pos1_mm = copy.copy(self.a1.pos_mm)
      if pos1_mm is not None:
        self.pos1_deg = (self.RUD_MAX - self.RUD_MIN) / (self.a1.MAX - self.a1.MIN) * (self.a1.pos_mm - self.a1.MIN) + self.RUD_MIN
        self._pos_deg = self.pos1_deg
    if a2 is not None:
      pos2_mm = copy.copy(self.a2.pos_mm)
      if pos2_mm is not None:
        self.pos2_deg = (self.RUD_MAX - self.RUD_MIN) / (self.a2.MAX - self.a2.MIN) * (self.a2.pos_mm - self.a2.MIN) + self.RUD_MIN
        self._pos_deg = self.pos2_deg
    if a1 is not None and a2 is not None:
      if self.pos1_deg is not None and self.pos2_deg is not None:
        self._pos_deg = (self.pos1_deg + self.pos2_deg) / 2

    return self._pos_deg

  def set_rudder(self, p):
    """
    Set the rudder angle to specified valued after constraining it
    Return constrained value
    """
    self._pos_cmd = self._constrain(p)
    pos1_mm, pos2_mm = self._calc_actuator_pos()
    self.a1.set_actuator(pos1_mm)
    self.a2.set_actuator(pos2_mm)
    return self._pos_cmd

  def _constrain(self, p):
    if p < self.RUD_MIN:
      return self.RUD_MIN
    if p > self.RUD_MAX:
      return self.RUD_MAX
    return p

  def _calc_actuator_pos(self):
    #TODO actual mapping between rudder angle and actuator position is defined by kinematics, which is likely trigonometric
    pos1_mm = (self.a1.MAX - self.a1.MIN) / (self.RUD_MAX - self.RUD_MIN) * (self._constrain(self._pos_cmd - self.TOE_IN) - self.RUD_MIN) + self.a1.MIN
    pos2_mm = (self.a2.MAX - self.a2.MIN) / (self.RUD_MAX - self.RUD_MIN) * (self._constrain(self._pos_cmd + self.TOE_IN) - self.RUD_MIN) + self.a2.MIN
    return pos1_mm, pos2_mm

  @property
  def pos_deg(self):
    return self.get_rudder(self.a1, self.a2)


class LEDBarGraph:
  NUMBARS = 24 #even #s only

  def __init__(self, r, actuatorMap):
    # start 2 Hz thread
    i2c = board.I2C()
    self.bc24 = Bicolor24(i2c, address=0x70)
    self.bc24.brightness = 1
    self.bc24.blink_rate = 0
    self.r = r
    self.actuatorMap = actuatorMap
    self.interval = (r.RUD_MAX - r.RUD_MIN) / self.NUMBARS
    self.bagraph_thread = threading.Thread(target=self._update_bargraph)
    self.bagraph_thread.start()

  def _update_bargraph(self):
    carray = [self.bc24.LED_OFF] * 24
    curr_pos = copy.copy(r.pos_deg)
    while True:
      old_carray = carray
      prev_pos = curr_pos
      curr_pos = copy.copy(r.pos_deg)
      time.sleep(0.25)
      #r.pos_deg will be None if we haven't received feedback msgs from both actuators
      if curr_pos is None or prev_pos is None:
        self.bc24.fill(self.bc24.LED_RED)
        carray = [self.bc24.LED_RED] * self.NUMBARS
        continue

      i_pos = int((curr_pos - r.RUD_MIN) // self.interval)
      i_pos = self.NUMBARS - 1 if i_pos >= self.NUMBARS else i_pos
      carray = [self.bc24.LED_OFF] * self.NUMBARS
      #TODO handle odd # of BARS
      if i_pos <= (self.NUMBARS // 2 - 1):
        i_cen = self.NUMBARS // 2
        carray[i_pos+1:i_cen] = [self.bc24.LED_GREEN] * (i_cen-i_pos-1)
      else:
        i_cen = self.NUMBARS // 2 - 1
        carray[i_cen+1:i_pos] = [self.bc24.LED_GREEN] * (i_pos - i_cen -1)
      carray[i_pos] = self.bc24.LED_RED
      carray[i_cen] = self.bc24.LED_YELLOW
      if curr_pos > prev_pos:
        updateseq = range(len(carray))
      else:
        updateseq = reversed(range(len(carray)))
      for i in updateseq:
        if carray[i] != old_carray[i]:
          self.bc24[i] = carray[i]

  def cleanup(self):
    self.bc24.fill(self.bc24.LED_OFF)

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

# def on_press(value):
#   a.center()
#   print("Reset actuator to: {}".format(a._pos_cmd))
#   EVENT.set()

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
    # positive increment is starboard
    r.change(RUD_INCR)
  else:
    # negative increment is port
    r.change(-RUD_INCR)

def on_exit(a, b):
  print("Exiting...")
  tcs.destroy()
  GPIO.cleanup()
  led.cleanup()
  sys.exit(0)


if __name__ == "__main__":
  
  signal.signal(signal.SIGINT, on_exit)

  a1 = Actuator(CAN1_ID) # Port actuator
  a2 = Actuator(CAN2_ID) # Starboard actuator
  
  actuatorMap = {
    CAN1_ID: a1,
    CAN2_ID: a2,
  }

  c  = CAN(actuatorMap)

  # debug("Reading rotary encoder from pins {} and {}".format(gpioA, gpioB))

  # if gpioButton != None:
  #   debug("Reading actuator reset position command from pin {}".format(gpioButton))

  # Actuator will respond within 0.1s, which will allow us to initialize the rudder
  # at the startup position
  time.sleep(0.5)

  r = Rudder(a1,a2)

  encoder_thread = threading.Thread(target=process_encoder_events)
  encoder_thread.start()

  tcs = TakeControlSwitch(callback=on_turn)

  led = LEDBarGraph(r,actuatorMap)
