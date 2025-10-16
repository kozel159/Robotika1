import brian.motors as motors   
import brian.sensors as sensors
import brian.uicontrol as unicontrol
import time

def lightUP(colour, animation):
    unicontrol.set_button_led(TL, animation, colour)
    unicontrol.set_button_led(TR, animation, colour)
    unicontrol.set_button_led(BL, animation, colour)
    unicontrol.set_button_led(BR, animation, colour)
    unicontrol.set_button_led(KN, animation, colour)

march = motors.EV3LargeMotor(motors.MotorPort.A)
descend = motors.EV3LargeMotor(motors.MotorPort.B)
spiiin = motors.EV3LargeMotor(motors.MotorPort.C)
march.wait_until_ready()
descend.wait_until_ready()
spiiin.wait_until_ready()

red = unicontrol.LedColor(256,0,0)
pink = unicontrol.LedColor(255,105,180)
colour = pink
animation = unicontrol.LedButtonAnimation(4)
TL = unicontrol.ButtonId(0)
TR = unicontrol.ButtonId(1)
BL = unicontrol.ButtonId(2)
BR = unicontrol.ButtonId(3)
KN = unicontrol.ButtonId(4)

#-------------------------------------------------------------------------

marchingDistance = 765
marchingSpeed = 180
grounded = 8        #FIXME - measure correct value for standing on ground
spinSpeed = 720
torqueTreshold = 5  #FIXME - measure correct value for detecting enemy

#-------------------------------------------------------------------------

ground = 0
currentSpinSpeed = 0
speedStep = spinSpeed/8
enemyDetect = 0
enemyDetected = False
enemyAlive = True
missCounter = []

lightUP(red, animation)

march.rotate_by_angle(marchingDistance,marchingSpeed)

while(ground < grounded):
    descend.rotate_by_angle(10,60)
    ground = descend.current_torque()

while(currentSpinSpeed < spinSpeed):
    spiiin.run_at_speed(currentSpinSpeed)
    time.sleep(0.5)
    currentSpinSpeed = currentSpinSpeed + speedStep

spiiin.run_at_speed(spinSpeed)

while True:
    enemyDetect = spiiin.current_torque()
    if(enemyDetect >= torqueTreshold):
        lightUP(red, animation)
        enemyDetected = True
    if(enemyDetected and (enemyDetect < torqueTreshold)):
        missCounter.append(10)
    else:
        missCounter.append(0)
    if(len(missCounter)>100):
        if((sum(missCounter)/len(missCounter)) < 1):
            break
        missCounter.pop(0)
    time.sleep(0.01)
    
