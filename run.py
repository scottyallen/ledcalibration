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
gflags.DEFINE_integer('serial_write_delay', 3,
                      'Number of milliseconds to wait between sending commands to the arduino. '
                      'Without this, my arduino seems to get overwhelmed, and drops commands.'
FLAGS = gflags.FLAGS

class LEDArray(object):

  def __init__(self, device, baud=115200):
    self.serial = serial.Serial(device, baud)

  def setLed(self, led, val):
    self.serial.write("%d %d\n" % (led, val))
    time.sleep(FLAGS.serial_write_delay/1000.0)

  def clear(self):
    for led in xrange(0, FLAGS.led_max):
      self.setLed(led, 0)

def blink():
  led_array = LEDArray(FLAGS.arduino_serial_device)

  while True:
    led_array.setLed(0, 128)
    time.sleep(0.1)
    led_array.setLed(0, 0)
    time.sleep(0.1)

class LEDCalibrator(object):

  def __init__(self, detector, led_array):
    self.detector = detector
    self.led_array = led_array
    self.thread = threading.Thread(target=self.run)
    self.thread.daemon = True

  def start(self):
    self.leds = {}
    self.thread.start()
    self.detector.run()
    self.thread.join()

  def groupPoints(self, points, tolerance):
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

  def warmup(self):
    for i in xrange(int(FLAGS.warmup_time / (FLAGS.blink_delay * 2))):
      for led in xrange(FLAGS.led_start, FLAGS.led_max + 1):
        self.led_array.setLed(led, 64)
      time.sleep(FLAGS.blink_delay)

      for led in xrange(FLAGS.led_start, FLAGS.led_max + 1):
        self.led_array.setLed(led, 0)
      time.sleep(FLAGS.blink_delay)

    self.led_array.clear()

  def run(self):
    # wait for the detector to start up
    while not self.detector.running:
      time.sleep(0.1)

    self.led_array.clear()

    # blink everything, so we can point the camera in the right direction
    self.warmup()

    # Cycle through each led
    for led in xrange(FLAGS.led_start, FLAGS.led_max + 1):
      # throw initial points away - they might be contaminated
      time.sleep(FLAGS.blink_delay)
      self.detector.getPoints()

      intensity_increment = FLAGS.led_blink_intensity / FLAGS.num_blinks
      intensity = intensity_increment
      for i in xrange(FLAGS.num_blinks):
        print "intensity:", intensity
        self.led_array.setLed(led, intensity)
        intensity += intensity_increment
        time.sleep(FLAGS.blink_delay)

        self.led_array.setLed(led, 0)
        time.sleep(FLAGS.blink_delay)

      print "LED #%d" % led
      points = self.detector.getPoints()
      groups = self.groupPoints(points, 10)
      groups.sort(key=lambda g: len(g), reverse=True)

      for i, group in enumerate(groups):
        group.sort()
        print "#%d" % i
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
  detector = LEDDetector()
  led_array = LEDArray(FLAGS.arduino_serial_device)
  calibrator = LEDCalibrator(detector, led_array)
  calibrator.start()
  leds = calibrator.leds

  sorted_leds = sorted(leds.items(), key=lambda x: x[1][1])

  min_y = sorted_leds[0][1][1]
  max_y = sorted_leds[-1][1][1]
  
  print "min_y: %r max_y: %r" % (min_y, max_y)

  led_array.clear()

  # vertical fade
  while True:
    for y in xrange(max_y, min_y, -10):
      for led, position in leds.iteritems():
        scale = min(255, abs(y - position[1]) * 2)
        value = max(0, 128 - scale)
        led_array.setLed(led, value)

    for y in xrange(min_y, max_y, 10):
      for led, position in leds.iteritems():
        scale = min(255, abs(y - position[1]) * 2)
        value = max(0, 128 - scale)
        led_array.setLed(led, value)


class LEDDetector(object):

  def __init__(self):
    self.running = False
    self.lock = threading.Lock()
    self.point_queue = []
    self._should_stop = False

  def _addPoint(self, point):
    with self.lock:
      self.point_queue.append(point)

  def getPoints(self):
    with self.lock:
      out = self.point_queue
      self.point_queue = []
      return out

  def stop(self):
    self._should_stop = True

  def run(self):
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
      #display = cv2.cvtColor(thresh, cv2.COLOR_GRAY2RGB)
      #display = cv2.cvtColor(diff, cv2.COLOR_GRAY2RGB)
      #display = np.zeros((720, 1280, 3), np.uint8)

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
  return sum(nums) / len(nums)

def main(argv):
  led_detect()
  #blink()

if __name__ == '__main__':
  argv = gflags.FLAGS(sys.argv)
  main(argv)
