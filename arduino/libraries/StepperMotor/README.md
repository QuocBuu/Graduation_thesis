This is the Arduino AccelStepper library. It provides an object-oriented interface for pul, dir stepper driver..

Version 1.1:
- setSpeed(float):
  set constant speed
  Ex: mm/s
- setStepsPerUnit(float)
  set number of steps per Unit. 
  Ex: vitme T8-4mm, 1.8 degrees motor, 1/2 microstep driver
      stepsPerUnit = (1 / 4) * 360.0 * 2 / 1.8 = 100 step/mm
- setStartDirection(bool)
  set start direction 
- moveTo(float)
  Control motor to target
  Ex: moveTo(100) - move to position 100 mm

