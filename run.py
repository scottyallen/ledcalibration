import json
import sys
import threading
import time

import numpy as np

import cv2
import serial
import gflags


gflags.DEFINE_integer('min_led_size', 20,
                     'The minimum bounding box size of an led contour as seen by the camera, in pixels. '
                     'Any contours smaller than this will be discarded.')
gflags.DEFINE_integer('led_start', 10, 'The index of the first led to include in the leds to calibrate.')
gflags.DEFINE_integer('led_max', 48, 'The index of the last led to include in the leds to calibrate.')
gflags.DEFINE_float('blink_delay', 0.05, 'The length of a blink of an led during calibration, in seconds.')
gflags.DEFINE_integer('num_blinks', 3, 'The number of times to blink an led during calibration.')
gflags.DEFINE_integer('warmup_time', 1,
                      'How long to blink all of the leds prior to starting calibration, to allow for '
                      'alignment of the camera.')
gflags.DEFINE_integer('led_blink_intensity', 255,
                     'The brightness of the led blinks during calibration. Lower values may help with '
                     'camera blowout. Number between 0-255.')
gflags.DEFINE_string('arduino_serial_device', '/dev/tty.usbmodem1411',
                     'The path to the serial device for the arduino')
gflags.DEFINE_string('serial_baud', 115200,
                     'The baud rate for the serial port.')
gflags.DEFINE_integer('serial_write_delay', 3,
                      'Number of milliseconds to wait between sending commands to the arduino. '
                      'Without this, my arduino seems to get overwhelmed, and drops commands.')
gflags.DEFINE_bool('incremental_intensity', True,
                   'Blink in increasing intensity, to try and cope with camera blowout.')
gflags.DEFINE_string('save', '', 'Filename to save led positions to.')
gflags.DEFINE_string('load', '', 'Filename to load led positions from.')

FLAGS = gflags.FLAGS

class LEDArray(object):
  '''An indidvually addressable array of LEDs.'''

  def __init__(self, device, baud=FLAGS.serial_baud):
    self.serial = serial.Serial(device, baud)

  def setLed(self, led, hue, saturation, value):
    '''Set a given led to a given value.'''
    self.serial.write("%s\n" % ''.join(map(chr, [led, hue, saturation, value])))
    time.sleep(FLAGS.serial_write_delay / 1000.0)

  def clear(self):
    '''Turn off all leds in the array.'''
    for led in xrange(0, FLAGS.led_max):
      self.setLed(led, 0, 0, 0)


class LEDCalibrator(object):

  def __init__(self, detector, led_array):
    self.detector = detector
    self.led_array = led_array
    self.thread = threading.Thread(target=self._run)
    self.thread.daemon = True

  def start(self):
    '''Start calibration. Should be called in the main thread.'''
    self.leds = {}
    self.thread.start()
    self.detector.run()
    self.thread.join()

  def groupPoints(self, points, tolerance):
    '''Groups points that are within a given pixel distance from each other (manhattan distance).'''
    if not points:
      return []

    # Start a single group with the first point
    groups = [ [points[0]] ]

    # Go through the rest of the points, and see if they belong to an existing group
    for point in points[1:]:
      matched = False
      for group in groups:
        if all([abs(point[0] - groupPt[0]) < tolerance and
                abs(point[1] - groupPt[1]) < tolerance for groupPt in group]):
          group.append(point)
          matched = True
          break
      if not matched:
        groups.append([point])

    return groups

  def warmup(self, warmup_time):
    '''Blink all the leds for a given number of seconds.'''
    for i in xrange(int(warmup_time / (FLAGS.blink_delay * 2))):
      for led in xrange(FLAGS.led_start, FLAGS.led_max + 1):
        self.led_array.setLed(led, 0, 0, 64)
      time.sleep(FLAGS.blink_delay)

      for led in xrange(FLAGS.led_start, FLAGS.led_max + 1):
        self.led_array.setLed(led, 0, 0, 0)
      time.sleep(FLAGS.blink_delay)

    self.led_array.clear()

  def _run(self):
    '''Run the actual calibration.'''
    # wait for the detector to start up
    while not self.detector.running:
      time.sleep(0.1)

    self.led_array.clear()

    # blink everything, so we can point the camera in the right direction
    self.warmup(FLAGS.warmup_time)

    self.led_array.clear()

    # Cycle through each led
    for led in xrange(FLAGS.led_start, FLAGS.led_max + 1):
      # throw initial points away - they might be contaminated
      time.sleep(FLAGS.blink_delay)
      self.detector.getPoints()

      if FLAGS.incremental_intensity:
        intensity_increment = FLAGS.led_blink_intensity / FLAGS.num_blinks
        intensity = intensity_increment
      else:
        intensity_increment = 0
        intensity = FLAGS.led_blink_intensity

      self.led_array.setLed(led, 0, 0, 0)
      for i in xrange(FLAGS.num_blinks):
        self.led_array.setLed(led, 0, 0, intensity)
        intensity += intensity_increment
        time.sleep(FLAGS.blink_delay)

        self.led_array.setLed(led, 0, 0, 0)
        time.sleep(FLAGS.blink_delay)

      print "LED #%d" % led
      points = self.detector.getPoints()
      groups = self.groupPoints(points, 10)
      groups.sort(key=lambda g: len(g), reverse=True)

      for i, group in enumerate(groups):
        group.sort()
        print "Group #%d" % i
        print "\n".join(["%d, %d" % (x, y) for x, y in group])
        print

      if groups:
        center = avg([x for x, y in groups[0]]), avg([y for x, y in groups[0]])
        print "LED Center: %r" % (center, )
        self.leds[led] = center

      print "----------------"
      print

    self.detector.stop()


def led_detect():
  '''Run the calibration routine.'''
  detector = LEDDetector()
  led_array = LEDArray(FLAGS.arduino_serial_device)
  calibrator = LEDCalibrator(detector, led_array)
  calibrator.start()
  leds = calibrator.leds
  return leds

def animation(leds):
  '''Run some test animations.'''
  sorted_leds = sorted(leds.items(), key=lambda x: x[1][1])

  min_y = sorted_leds[0][1][1]
  max_y = sorted_leds[-1][1][1]
  
  led_array = LEDArray(FLAGS.arduino_serial_device)
  led_array.clear()

  # vertical fade
  while True:
    for y in xrange(max_y, min_y, -10):
      for led, position in leds.iteritems():
        scale = min(255, abs(y - position[1]) * 2)
        value = max(0, 128 - scale)
        led_array.setLed(led, 0, 0, value)

    for y in xrange(min_y, max_y, 10):
      for led, position in leds.iteritems():
        scale = min(255, abs(y - position[1]) * 2)
        value = max(0, 128 - scale)
        led_array.setLed(led, 0, 0, value)


class LEDDetector(object):
  '''Detects possible led positions using the camera.

  While running, grabs frames from the camera, looking for potential leds.  Call run() in the main
  thread to start it, which blocks.  Call getPoints() in a separate thread to get a list of 2d
  points where an led could potentially be since the last time it was called.  Call stop() to stop
  it.  Displays a gui window showing what the camera is seeing with overlays of where leds are
  detected, for debugging.
  '''

  def __init__(self):
    self.running = False
    self.lock = threading.Lock()
    self.point_queue = []
    self._should_stop = False

  def _addPoint(self, point):
    '''Add a point to the queue.'''
    with self.lock:
      self.point_queue.append(point)

  def getPoints(self):
    '''Get the list of points where an led was detected since the last call.

    Returns: A list of (x, y) tuples in pixels.
    '''
    with self.lock:
      out = self.point_queue
      self.point_queue = []
      return out

  def stop(self):
    '''Stop the LEDDetector running.'''
    self._should_stop = True

  def run(self):
    '''Start the LEDDetector.

    Starts the camera capture and gui display.
    '''
    video_capture = cv2.VideoCapture(0)

    previous = None

    rectangles = []
    points = []

    self.running = True

    while not self._should_stop:

      # Capture frame-by-frame
      ret, frame = video_capture.read()

      if frame is None:
        print "Skipping - no frame: %r" % ret
        continue
      
      if not frame.any():
        print "Skipping - zero frame"
        continue

      gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

      ret, thresh = cv2.threshold(gray, 254, 255, cv2.THRESH_BINARY)

      if previous is None:
        previous = gray
        continue

      diff = cv2.subtract(thresh, previous)

      previous = thresh

      display = frame.copy()

      contours, hierarchy = cv2.findContours(diff, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

      cv2.drawContours(display, contours, -1, (0,255,0), 3)

      regions = []
      for contour in contours:
        regions.append(cv2.boundingRect(contour))

      found_rectangles = []
      found_points = []
      for x, y, width, height in regions:
        if width < FLAGS.min_led_size or height < FLAGS.min_led_size:
          continue
        pt1 = x,y
        pt2 = x+width,y+height
        mid = x + width/2, y + height/2
        found_rectangles.append( (pt1, pt2) )
        found_points.append(mid)
        self._addPoint(mid)

      if found_rectangles:
        rectangles = found_rectangles
        points = found_points

      for pt1, pt2 in rectangles:
        color = (0,0,255,0)
        cv2.rectangle(display, pt1, pt2, color, 2)

      for pt in points:
        color = (255,0,0,0)
        cv2.circle(display, pt, 5, color, 5)

      # Display the resulting frame
      cv2.imshow('Video', display)

      if cv2.waitKey(1) & 0xFF == ord('q'):
        break

    # When everything is done, release the capture
    video_capture.release()
    cv2.destroyAllWindows()

def avg(nums):
  '''Returns the average of a list of numbers.

  Returns an integer if all inputs are integers.
  '''
  return sum(nums) / len(nums)

def main(argv):
  leds = None
  if FLAGS.load:
    print "Loading..."
    leds = json.load(open(FLAGS.load))
    leds = dict([(int(k), v) for k, v in leds.iteritems()])
  else:
    leds = led_detect()
    if FLAGS.save:
      json.dump(leds, open(FLAGS.save, 'w'))
  animation(leds)

if __name__ == '__main__':
  argv = gflags.FLAGS(sys.argv)
  main(argv)
