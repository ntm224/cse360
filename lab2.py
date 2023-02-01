import time
from Motor import *
from Led import *
from Buzzer import *
buzzer = Buzzer()
PWM = Motor()
led = Led()
def test_Square():
  try:
    PWM.setMotorModel(1000,1000,1000,1000)
    time.sleep(2)
    PWM.setMotorModel(2000,2000,-1500,-1500)
    time.sleep(.74)
    led.ledIndex(0x01,255,0,0)
    
    PWM.setMotorModel(1000,1000,1000,1000)
    time.sleep(2)
    PWM.setMotorModel(2000,2000,-1500,-1500)
    time.sleep(.74)
    led.ledIndex(0x02,0,0,255)
    
    PWM.setMotorModel(1000,1000,1000,1000)
    time.sleep(2)
    PWM.setMotorModel(2000,2000,-1500,-1500)
    time.sleep(.74)
    led.ledIndex(0x03,0,255,0)
    
    PWM.setMotorModel(1000,1000,1000,1000)
    time.sleep(2)
    PWM.setMotorModel(2000,2000,-1500,-1500)
    time.sleep(.74)
    PWM.setMotorModel(0,0,0,0)
    led.ledIndex(0x03,255,255,0)
    buzzer.run('1')
    time.sleep(1)
    buzzer.run('0')
    led.colorWipe(led.strip, Color(0,0,0))
  except KeyboardInterrupt:
    buzzer.run('0')
    led.colorWipe(led.strip, Color(0,0,0))
    PWM.setMotorModel(0,0,0,0)
    
test_square()
