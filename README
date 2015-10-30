# LED Calibrator

## Prerequisites

- [FastLED arduino library](http://fastled.io/)
- [opencv](http://opencv.org)
- [Python Gflags](https://pypi.python.org/pypi/python-gflags)
- [Pyserial](https://pypi.python.org/pypi/pyserial)

You can install python-gflags and pyserial by running:

pip install -r requirements.txt

It's often easier to install Opencv using a system package manager (I installed it on my mac using homebrew).

## Hardware

I used an UCS1903B addressable LED strand attached to pin 3 on an Arduino Uno for testing.  You easily use a
different led setup - just adjust the define statements in the arduino code accordingly.

## Getting started

- Load the serialled sketch onto your arduino.
- Run run.py.  This will run the calibration routine, and then a simple test animation.  If you pass it the --save=filename flag, it will save the led positions to a file.
