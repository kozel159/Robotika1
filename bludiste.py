import brian.motors as motors
import brian.sensors as sensors
from collections import deque
import time

ultrasonic = sensors.EV3.UltrasonicSensorEV3(sensors.SensorPort.S2)
motor_b = motors.EV3LargeMotor(motors.MotorPort.B)  # scanner
motor_b.wait_until_ready()
motor_a = motors.EV3LargeMotor(motors.MotorPort.A)  # left wheel
motor_a.wait_until_ready()
motor_d = motors.EV3LargeMotor(motors.MotorPort.D)  # right wheel
motor_d.wait_until_ready()

# ===== Maze state =====
maze = {(0, 0): "visited"}  # start tile
x, y = 0, 0
facing = "N"

# ===== Movement tuning =====
FORWARD_ROT = 1440 
TURN_ROT = 180     
SPEED = 800

# ===== Helper functions =====
def direction_to_delta(direction):
    return {"N": (0, 1), "E": (1, 0), "S": (0, -1), "W": (-1, 0)}[direction]

def rotate_relative_dirs(facing):
    return {
        "N": {"front": "N", "right": "E", "left": "W"},
        "E": {"front": "E", "right": "S", "left": "N"},
        "S": {"front": "S", "right": "W", "left": "E"},
        "W": {"front": "W", "right": "N", "left": "S"}
    }[facing]

def should_scan(x, y, maze):
    for dx, dy in [(0,1),(1,0),(0,-1),(-1,0)]:
        if (x+dx, y+dy) not in maze:
            return True
    return False

def scan_surroundings(scan_motor, distance_sensor, facing, x, y, maze):
    dir_map = rotate_relative_dirs(facing)
    for local_dir, global_dir in dir_map.items():
        rotation_angle = 0 if local_dir == "front" else (-90 if local_dir == "right" else 90)
        if rotation_angle != 0:
            scan_motor.rotate_by_angle(rotation_angle, speed=300)
            time.sleep(0.2)
        distance = distance_sensor.distance_mm()
        is_wall = distance < 100
        dx, dy = direction_to_delta(global_dir)
        nx, ny = x + dx, y + dy
        maze[(nx, ny)] = "wall" if is_wall else maze.get((nx, ny), "free")
        if rotation_angle != 0:
            scan_motor.rotate_by_angle(-rotation_angle, speed=300)
            time.sleep(0.2)

def turn_left():
    global facing
    motor_a.rotate_by_angle(-TURN_ROT, SPEED, False)
    motor_d.rotate_by_angle(TURN_ROT, SPEED)
    facing = {"N": "W", "W": "S", "S": "E", "E": "N"}[facing]
    time.sleep(0.2)

def turn_right():
    global facing
    motor_a.rotate_by_angle(TURN_ROT, SPEED, False)
    motor_d.rotate_by_angle(-TURN_ROT, SPEED)
    facing = {"N": "E", "E": "S", "S": "W", "W": "N"}[facing]
    time.sleep(0.2)

def turn_around():
    global facing
    motor_a.rotate_by_angle(2 * TURN_ROT, SPEED, False)
    motor_d.rotate_by_angle(-2 * TURN_ROT, SPEED)
    facing = {"N": "S", "S": "N", "E": "W", "W": "E"}[facing]
    time.sleep(0.2)

def move_forward():
    global x, y
    motor_a.rotate_by_angle(FORWARD_ROT, SPEED, False)
    motor_d.rotate_by_angle(FORWARD_ROT, SPEED)
    dx, dy = direction_to_delta(facing)
    x += dx
    y += dy
    maze[(x, y)] = "visited"
    print(f"Moved to {(x, y)}, facing {facing}")
    time.sleep(0.3)

def get_unvisited_neighbors():
    options = []
    for d in ["N", "E", "S", "W"]:
        dx, dy = direction_to_delta(d)
        nx, ny = x + dx, y + dy
        if maze.get((nx, ny)) == "free":
            options.append((nx, ny))
    return options

def turn_toward(target):
    global facing
    dx, dy = target[0] - x, target[1] - y
    target_dir = {(0,1):"N", (1,0):"E", (0,-1):"S", (-1,0):"W"}[(dx,dy)]
    if target_dir == facing:
        return
    elif target_dir == {"N":"E","E":"S","S":"W","W":"N"}[facing]:
        turn_right()
    elif target_dir == {"N":"W","W":"S","S":"E","E":"N"}[facing]:
        turn_left()
    else:
        turn_around()

def bfs_shortest_path(start, goal):
    queue = deque([(start, [])])
    visited = {start}
    while queue:
        pos, path = queue.popleft()
        if pos == goal:
            return path
        for dx, dy in [(0,1), (1,0), (0,-1), (-1,0)]:
            neighbor = (pos[0]+dx, pos[1]+dy)
            if neighbor in maze and maze[neighbor] != "wall" and neighbor not in visited:
                queue.append((neighbor, path + [neighbor]))
                visited.add(neighbor)
    return []

def find_nearest_unvisited(start):
    queue = deque([start])
    visited = {start}
    while queue:
        pos = queue.popleft()
        for dx, dy in [(0,1), (1,0), (0,-1), (-1,0)]:
            nx, ny = pos[0]+dx, pos[1]+dy
            if (nx, ny) in visited:
                continue
            tile = maze.get((nx, ny))
            if tile == "free":
                return (nx, ny)
            if tile == "visited":
                queue.append((nx, ny))
                visited.add((nx, ny))
    return None

# ===== Main loop =====
while True:
    if should_scan(x, y, maze):
        scan_surroundings(motor_b, ultrasonic, facing, x, y, maze)

    unvisited = get_unvisited_neighbors()

    if unvisited:
        # Move to the nearest in front>right>left order
        priorities = [facing,
                      {"N": "E", "E": "S", "S": "W", "W": "N"}[facing],
                      {"N": "W", "W": "S", "S": "E", "E": "N"}[facing]]
        next_tile = None
        for direction in priorities:
            dx, dy = direction_to_delta(direction)
            nx, ny = x + dx, y + dy
            if (nx, ny) in unvisited:
                next_tile = (nx, ny)
                break
        if next_tile:
            turn_toward(next_tile)
            move_forward()

    else:
        # No local unvisited tiles → backtrack using BFS
        target = find_nearest_unvisited((x, y))
        if not target:
            print("Maze fully explored!")
            break

        path = bfs_shortest_path((x, y), target)
        if not path:
            print("No path found to", target)
            break

        # Physically drive tile by tile along the BFS path
        for step in path:
            turn_toward(step)
            move_forward()
            if should_scan(x, y, maze):
                scan_surroundings(motor_b, ultrasonic, facing, x, y, maze)
