from Motor import *
PWM=Motor()
from Ultrasonic import *
ultrasonic=Ultrasonic()

def test_USSpeed():
  try:
    data = ultrasonic.get_distance()
    while data >= 50:
      data=ultrasonic.get_distance()
      speed = 5*(data-40) #tried 5, 10, and 20 as k values
                          #also, only subtracted 40 here since the robot wouldn't go below a value of ~300, so couldn't decelerate all the way to 0
      print("Distance "+str(data)+"CM, speed")
      print(speed)
      PWM.setMotorModel(speed, speed, speed, speed)
      time.sleep(.1)
      
    PWM.setMotorModel(0,0,0,0)
  except KeyboardInterrupt:
    PWM.setMotorModel(0,0,0,0)
    
test_USSpeed()
