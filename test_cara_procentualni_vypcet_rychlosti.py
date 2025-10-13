import brian.motors as motors   
import brian.sensors as sensors
import time

color = sensors.EV3.ColorSensorEV3(sensors.SensorPort.S4)

motor_a = motors.EV3LargeMotor(motors.MotorPort.A)
motor_a.wait_until_ready()
motor_d = motors.EV3LargeMotor(motors.MotorPort.D)
motor_d.wait_until_ready()

black = 16
white = 70
line = ((black + white)/2) - black
readingRange = (white - black)

errorBalance = 0
previousError = 0
dt = 0.05
Kp = 1.2
speed = 300

values = []

while True:
    measuredLinePosition = color.reflected_value()
    
    values.append(measuredLinePosition)
    measuredLinePositionAverage = (sum(values) / len(values)) - black

    if(len(values)>5):
        values.pop(0)
    
    error = ((line - measuredLinePositionAverage)/readingRange)
    print(error)
    errorPrediction = 0.1 * (error - previousError) / dt
    previousError = error

    output = Kp * (error)  + errorPrediction
    print('o:',output )
    speedLeft = max(min(speed * (1 + output), 1050), -1050)
    speedRight = max(min(speed * (1 - output), 1050), -1050)

    motor_d.run_at_speed(-speedRight)
    motor_a.run_at_speed(-speedLeft)

    time.sleep(dt)