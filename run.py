import sys
import threading
import time

import numpy as np

import cv2
import serial

MIN_SIZE = 20
LED_START = 35
LED_MAX = 48
LED_INCREMENT = 1
BLINK_DELAY = 0.1
NUM_BLINKS = 5
WARMUP_PASSES = 5

class LEDArray(object):

  def __init__(self, device, baud=9600):
    self.serial = serial.Serial(device, baud)

  def setLed(self, led, val):
    self.serial.write("%d %d\n" % (led, val))

def blink():
  led_array = LEDArray("/dev/tty.usbmodem1411")

  while True:
    led_array.setLed(0, 1)
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

  def run(self):
    # wait for the detector to start up
    while not self.detector.running:
      time.sleep(0.1)

    for i in xrange(WARMUP_PASSES):
      for led in xrange(LED_START, LED_MAX + 1, LED_INCREMENT):
        self.led_array.setLed(led, 1)
        time.sleep(BLINK_DELAY)

    # Cycle through each led
    for led in xrange(LED_START, LED_MAX + 1, LED_INCREMENT):
      for i in xrange(2):
        self.led_array.setLed(led, 1)
        time.sleep(BLINK_DELAY)

        self.led_array.setLed(led, 0)
        time.sleep(BLINK_DELAY)

      # throw initial points away - they might be contaminated
      self.detector.getPoints()

      for i in xrange(NUM_BLINKS):
        self.led_array.setLed(led, 1)
        time.sleep(BLINK_DELAY)

        self.led_array.setLed(led, 0)
        time.sleep(BLINK_DELAY)

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
  led_array = LEDArray("/dev/tty.usbmodem1411")
  calibrator = LEDCalibrator(detector, led_array)
  calibrator.start()
  leds = calibrator.leds

  sorted_leds = sorted(leds.items(), key=lambda x: x[1][1])

  for led, position in sorted_leds:
    print led, position

  while True:
    for led, position in sorted_leds:
      led_array.setLed(led, 1)
      time.sleep(BLINK_DELAY)


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
        if width < MIN_SIZE or height < MIN_SIZE:
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
  main(sys.argv)
