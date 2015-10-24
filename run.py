import sys
import time

import numpy as np

import cv2
import serial

MIN_SIZE = 20
LED_START = 46
LED_MAX = 49
LED_INCREMENT = 1

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

def led_detect():
  video_capture = cv2.VideoCapture(0)

  previous = None

  rectangles = []
  points = []

  led = LED_START

  led_array = LEDArray("/dev/tty.usbmodem1411")

  while True:
    # Cycle through each led
    led_array.setLed(led, 1)
    led += LED_INCREMENT
    if led > LED_MAX:
      led = LED_START

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

def main(argv):
  led_detect()
  #blink()

if __name__ == '__main__':
  main(sys.argv)
