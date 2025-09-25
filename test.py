import brian.motors as motors
import brian.sensors as sensors

color = sensors.EV3.ColorSensorEV3(sensors.SensorPort.S4)

motor_a = motors.EV3LargeMotor(motors.MotorPort.A)
motor_a.wait_until_ready()
motor_d = motors.EV3LargeMotor(motors.MotorPort.D)
motor_d.wait_until_ready()

idealLinePosition = 45
errorBalance = 0
previousError = 0
dt = 0.25
Kp = 1
Ki = 0.1
Kd = 0.05
speed = 800

while True:
    measuredLinePosition = color.reflected_value()
    error = measuredLinePosition - idealLinePosition
    errorBalance = (2/3) * errorBalance + error
    errorPrediction = (error - previousError) / dt
    previousError = error

    output = Kp * error + Ki * errorBalance + Kd * errorPrediction
    speedLeft = max(min(speed + output, 1050), -1050)
    speedRight = max(min(speed - output, 1050), -1050)

    motor_a.run_at_speed(speedRight)
    motor_d.run_at_speed(speedLeft)