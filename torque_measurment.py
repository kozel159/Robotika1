import brian.motors as motors
import time

spiiin = motors.EV3LargeMotor(motors.MotorPort.B)

spiiin.run_at_speed(720)

while True:
    print(spiiin.current_torque)
    time.sleep(0.01)