from controller import Robot, DistanceSensor, Motor
import numpy as np
from collections import deque
import heapq # Import for Dijkstra's algorithm

# -------------------------------------------------------
# Grid Map Definition and Navigation (with weights)

corrected_weighted_grid = {
    # Parking spots (P nodes)
    "P1": {"S": ("A1", 2.0)},
    "P2": {"S": ("A2", 2.0)},
    "P3": {"S": ("A3", 2.0)},
    "P4": {"S": ("A4", 2.0)},
    "P5": {"N": ("E3", 2.0)},
    "P6": {"N": ("E4", 2.0)},
    "P7": {"N": ("E5", 2.0)},
    "P8": {"N": ("E6", 2.0)},

    # A row
    "A1": {"N": ("P1", 2.0), "E": ("A2", 1.0), "S": ("C1", 3.0)},
    "A2": {"N": ("P2", 2.0), "E": ("A3", 1.0), "W": ("A1", 1.0)},
    "A3": {"N": ("P3", 2.0), "E": ("A4", 1.0), "W": ("A2", 1.0)},
    "A4": {"N": ("P4", 2.0), "E": ("A5", 1.0), "W": ("A3", 1.0)},
    "A5": {"N": None, "E": ("A6", 1.0), "S": ("B1", 1.0), "W": ("A4", 1.0)},
    "A6": {"N": None, "E": None, "S": ("B2", 1.0), "W": ("A5", 1.0)},

    # B row
    "B1": {"N": ("A5", 1.0), "E": ("B2", 1.0), "S": ("C2", 1.0)},
    "B2": {"N": ("A6", 1.0), "E": None, "S": ("C3", 1.0), "W": ("B1", 1.0)},

    # C row
    "C1": {"N": ("A1", 3.0), "E": ("C2", 1.0), "S": ("D1", 1.0)},
    "C2": {"N": ("B1", 1.0), "E": ("C3", 1.0), "S": ("D2", 1.0), "W": ("C1", 1.0)},
    "C3": {"N": ("B2", 1.0), "E": None, "S": ("E6", 1.0), "W": ("C2", 1.0)},

    # D row
    "D1": {"N": ("C1", 1.0), "E": ("D2", 1.0), "S": ("E1", 1.0)},
    "D2": {"N": ("C2", 1.0), "E": None, "S": ("E2", 1.0), "W": ("D1", 5.0)},

    # E row
    "E1": {"N": ("D1", 1.0), "E": ("E2", 1.0), "S": None, "W": None},
    "E2": {"N": ("D2", 1.0), "E": ("E3", 1.0), "S": None, "W": ("E1", 1.0)},
    "E3": {"S": ("P5", 2.0), "E": ("E4", 1.0), "N": None, "W": ("E2", 1.0)},
    "E4": {"S": ("P6", 2.0), "E": ("E5", 1.0), "N": None, "W": ("E3", 1.0)},
    "E5": {"S": ("P7", 2.0), "E": ("E6", 1.0), "N": None, "W": ("E4", 1.0)},
    "E6": {"N": ("C3", 1.0), "E": None, "S": ("P8", 2.0), "W": ("E5", 1.0)},
}

def find_path_dijkstra(start_node, goal_node, grid_map_weighted, blocked_nodes=None):
    """
    Find shortest path between nodes using Dijkstra's algorithm with weighted edges.
    """
    if blocked_nodes is None:
        blocked_nodes = set()

    if start_node not in grid_map_weighted or goal_node not in grid_map_weighted:
        print(f"Error: Invalid nodes - Start: {start_node}, Goal: {goal_node}")
        return []

    # Distance from start_node to all other nodes
    distances = {node: float('infinity') for node in grid_map_weighted}
    distances[start_node] = 0

    # Priority queue to store (distance, node, path)
    # The smallest distance is always at the top
    priority_queue = [(0, start_node, [start_node])]

    while priority_queue:
        current_distance, current_node_pq, path_pq = heapq.heappop(priority_queue)

        # If we have found a shorter path to current_node_pq already, skip
        if current_distance > distances[current_node_pq]:
            continue

        # If we reached the goal, return the path
        if current_node_pq == goal_node:
            return path_pq

        # Explore neighbors
        for direction, neighbor_info in grid_map_weighted[current_node_pq].items():
            if neighbor_info: # Ensure neighbor_info is not None
                neighbor, weight = neighbor_info
                if neighbor not in blocked_nodes:
                    distance = current_distance + weight

                    # If this path is shorter, update distance and push to queue
                    if distance < distances[neighbor]:
                        distances[neighbor] = distance
                        new_path = path_pq + [neighbor]
                        heapq.heappush(priority_queue, (distance, neighbor, new_path))

    print(f"No path found from {start_node} to {goal_node}")
    return []

def get_direction_to_next_node(current_node, next_node, grid_map_weighted):
    """
    Get the direction (N, E, S, W) to move from current_node to next_node
    """
    if current_node not in grid_map_weighted:
        print(f"ERROR: get_direction_to_next_node: current_node '{current_node}' not in grid_map_weighted.")
        return None

    for direction, neighbor_info in grid_map_weighted[current_node].items():
        if neighbor_info and neighbor_info[0] == next_node: # Check the first element of the tuple (the neighbor name)
            return direction

    print(f"ERROR: get_direction_to_next_node: No direct connection found from '{current_node}' to '{next_node}'.")
    return None

def direction_to_robot_action(direction, current_heading):
    """
    Convert grid direction to robot action based on current heading
    Returns: (action_string, angle_difference_needed)
    """
    # Map directions to angles (in radians)
    # 0 = East, pi/2 = North, -pi/2 = South, pi = West
    direction_angles = {
        'N': np.pi/2,   
        'E': 0,         
        'S': -np.pi/2,  
        'W': np.pi      
    }
    
    if direction not in direction_angles:
        print(f"WARNING: Unknown direction '{direction}'. Defaulting to 'forward'.")
        return 'forward', 0.0 # Return 0 angle diff for forward action
    
    target_angle = direction_angles[direction]
    
    # Normalize current heading and target angle to [-pi, pi]
    current_heading = normalize_angle(current_heading)
    target_angle = normalize_angle(target_angle)

    print(f"DEBUG: In direction_to_robot_action: direction='{direction}', current_heading={np.degrees(current_heading):.1f}°, target_angle (from map)={np.degrees(target_angle):.1f}°")

    # Calculate angle difference
    # Use atan2 to get the shortest angular distance and proper sign for direction
    angle_diff = np.arctan2(np.sin(target_angle - current_heading), np.cos(target_angle - current_heading))
    
    print(f"DEBUG: angle_diff (normalized)={np.degrees(angle_diff):.1f}°")
    
    # Determine required action based on angle difference
    # Tighter tolerance for going straight
    if abs(angle_diff) < 0.1: # Within approx 5.7 degrees - go forward
        return 'forward', angle_diff
    elif abs(angle_diff - np.pi) < 0.2 or abs(angle_diff + np.pi) < 0.2: # Close to 180 degrees (pi or -pi)
        return 'turn_around', angle_diff
    elif angle_diff > 0:       # Need to turn left (positive angle diff)
        return 'turn_left', angle_diff
    else:                      # Need to turn right (negative angle diff)
        return 'turn_right', angle_diff

# -------------------------------------------------------
# Robot Controller Setup

MAX_SPEED = 6.28

robot = Robot()
timestep = int(robot.getBasicTimeStep())

print("Node-based robot controller with IMPROVED TURNING and DIJKSTRA PATHFINDING")

# Initialize devices
ps = [robot.getDevice(f'ps{i}') for i in range(8)]
for sensor in ps:
    sensor.enable(timestep)

gs = [robot.getDevice(f'gs{i}') for i in range(3)]
for sensor in gs:
    sensor.enable(timestep)

encoder = [robot.getDevice(name) for name in ['left wheel sensor', 'right wheel sensor']]
for enc in encoder:
    enc.enable(timestep)

leftMotor = robot.getDevice('left wheel motor')
rightMotor = robot.getDevice('right wheel motor')
leftMotor.setPosition(float('inf'))
rightMotor.setPosition(float('inf'))
leftMotor.setVelocity(0.0)
rightMotor.setVelocity(0.0)

# Robot parameters
R = 0.0205  # Wheel radius [m]
D = 0.057   # Distance between wheels [m]
delta_t = timestep / 1000.0

# Position tracking variables
# IMPORTANT: Adjust initial phi based on your robot's starting orientation in Webots.
# If robot starts facing East (default), phi = 0.
# If robot starts facing North, phi = np.pi/2.
# If robot starts facing South, phi = -np.pi/2. (This was the original assumption)
# If robot starts facing West, phi = np.pi.
phi = -np.pi/2  # Assuming robot starts facing South
oldEncoderValues = [0, 0]
encoderValues = [0, 0]

# Navigation setup
start_node = "P1"
goal_node = "P8"  # Example goal
robot_current_grid_node = start_node # Robot's logical position on the grid
last_confirmed_node = start_node # Last node robot definitively passed through
next_node_index = 1  # Index of the next node in the path the robot is moving towards

# Generate path using Dijkstra's algorithm
path = find_path_dijkstra(start_node, goal_node, corrected_weighted_grid)
print(f"Path from {start_node} to {goal_node}: {' -> '.join(path)}")

if len(path) < 2:
    print("ERROR: No valid path found or path is too short! Robot will stop.")
    path = [start_node] # Ensure path is not empty to avoid index errors
    next_node_index = 0 # Indicate no further nodes to target
    robot_state = 'stopping'
else:
    print(f"First target node in path (robot will move towards): {path[next_node_index]}")
    
# State machine variables
robot_states = ['line_following', 'at_intersection_drive_through', 'turning', 'post_turn_line_search', 'obstacle_avoidance', 'stopping']
robot_state = 'line_following' if len(path) >= 2 else 'stopping'

# Line following variables
line_threshold = 600.0  # Threshold for line detection
base_speed = MAX_SPEED * 0.5  # Base speed for line following
max_turn_speed_diff = MAX_SPEED * 0.4  # Maximum speed difference for turning

# PID parameters for smooth line following
kp = 2.0 # Proportional gain
ki = 0.2  # Integral gain
kd = 0.4  # Derivative gain

# Error tracking for PID
previous_error = 0.0
integral_error = 0.0
error_history = []
max_history_length = 5

# Line following state tracking
consecutive_no_line = 0
max_no_line_steps = 10
last_known_direction = 'center'

# Intersection detection variables
cross_counter = 0
CROSS_COUNTER_MIN = 8   # Reduced for faster intersection detection
INTERSECTION_DRIVE_THROUGH_DURATION = 35 # Steps to drive through after intersection detection
intersection_drive_through_counter = 0

# Enhanced turning variables with better durations
turn_action = 'forward' # Stores the action ('turn_left', 'turn_right', etc.)
expected_heading_change = 0.0 # Stores the precise angle difference needed for the turn

stop_counter = 0
turn_counter = 0  
post_turn_counter = 0  
line_search_counter = 0

STOP_DURATION = 15      # Steps to stop before turning (increased)
POST_TURN_PAUSE = 10    # Steps to pause after turn (reduced)
LINE_SEARCH_DURATION = 30 # Steps to search for line after turn

# Turn completion tracking (IMPROVED with closed-loop)
turn_start_heading = 0.0
heading_tolerance = 0.05 # Radians (approx 3 degrees) - MUCH TIGHTER for closed-loop
turn_kp = 2.0 # Proportional gain for turning (increased for snappier turns)

# Obstacle avoidance variables
obstacle_threshold = 85.0
obstacle_detected = False
obstacle_avoidance_states = ['normal', 'turning_180', 'driving_after_180_turn', 'reacquiring_line', 'replanning_path'] # Updated states
obstacle_avoidance_state = 'normal'
turn_start_angle = 0
reset_timer = 0
# RESET_DURATION is now used more generally for timers in obstacle avoidance
replanning_cooldown = 0
REPLANNING_COOLDOWN_TIME = 50
obstacle_blocked_node = None # Stores the node that was the target when the obstacle was hit

# -------------------------------------------------------
# Helper Functions

def get_wheels_speed(encoderValues, oldEncoderValues, delta_t):
    wl = (encoderValues[0] - oldEncoderValues[0]) / delta_t
    wr = (encoderValues[1] - oldEncoderValues[1]) / delta_t
    return wl, wr

def get_robot_speeds(wl, wr, R, D):
    u = R / 2.0 * (wr + wl)
    w = R / D * (wr - wl)
    return u, w

def update_heading(w, phi, delta_t):
    """Update robot heading based on angular velocity"""
    phi += w * delta_t
    
    # Wrap angle to [-π, π]
    while phi >= np.pi:
        phi -= 2*np.pi
    while phi < -np.pi:
        phi += 2*np.pi
    
    return phi

def normalize_angle(angle):
    """Normalize angle to [-π, π]"""
    while angle > np.pi:
        angle -= 2*np.pi
    while angle < -np.pi:
        angle += 2*np.pi
    return angle

def detect_intersection(gsValues):
    """
    Enhanced intersection detection with better filtering
    """
    line_right = gsValues[0] < line_threshold
    line_center = gsValues[1] < line_threshold  
    line_left = gsValues[2] < line_threshold
    
    # Strong intersection signal: all three sensors detect line
    strong_intersection = line_right and line_center and line_left
    
    return strong_intersection

def calculate_line_error(gsValues):
    """
    Calculate line following error using weighted sensor positions
    Returns error value: negative = line to left, positive = line to right, 0 = centered
    """
    # Normalize sensor values (invert so line gives high values)
    right_val = max(0, line_threshold - gsValues[0]) / line_threshold
    center_val = max(0, line_threshold - gsValues[1]) / line_threshold
    left_val = max(0, line_threshold - gsValues[2]) / line_threshold
    
    # Calculate weighted position (-1 to +1)
    total_activation = right_val + center_val + left_val
    
    if total_activation < 0.1:  # No line detected (adjust threshold if needed)
        return None
    
    # Weighted average position: -1 (left) to +1 (right)
    error = (right_val * 1.0 + center_val * 0.0 + left_val * (-1.0)) / total_activation
    
    return error

def advanced_line_following_control(gsValues):
    """
    Advanced PID-based line following with smooth control
    """
    global previous_error, integral_error, error_history
    global consecutive_no_line, last_known_direction
    
    # Calculate current error
    current_error = calculate_line_error(gsValues)
    
    if current_error is None:
        # No line detected - handle lost line situation
        consecutive_no_line += 1
        
        if consecutive_no_line < max_no_line_steps:
            # Use last known error direction to search for line
            if last_known_direction == 'left':
                current_error = -0.6  # Search left
            elif last_known_direction == 'right':
                current_error = 0.6   # Search right
            else:
                current_error = 0.0   # Go straight
        else:
            # Been lost too long, slow forward movement
            return base_speed * 0.3, base_speed * 0.3
    else:
        # Line found, reset lost counter
        consecutive_no_line = 0
        
        # Update last known direction
        if current_error < -0.2:
            last_known_direction = 'left'
        elif current_error > 0.2:
            last_known_direction = 'right'
        else:
            last_known_direction = 'center'
    
    # Update error history for derivative calculation
    error_history.append(current_error)
    if len(error_history) > max_history_length:
        error_history.pop(0)
    
    # Calculate PID components
    proportional = current_error
    
    # Integral (accumulated error)
    if consecutive_no_line == 0:
        integral_error += current_error
    else:
        integral_error *= 0.9  # Decay integral when line is lost
    integral_error = max(-2.0, min(2.0, integral_error))  # Clamp integral
    
    # Derivative (rate of change)
    derivative = 0.0
    if len(error_history) >= 2:
        derivative = error_history[-1] - error_history[-2]
    
    # PID output
    pid_output = kp * proportional + ki * integral_error + kd * derivative
    
    # Clamp PID output
    pid_output = max(-1.0, min(1.0, pid_output))
    
    # Convert PID output to motor speeds
    turn_adjustment = pid_output * max_turn_speed_diff
    
    leftSpeed = base_speed - turn_adjustment
    rightSpeed = base_speed + turn_adjustment
    
    # Ensure speeds are within bounds
    leftSpeed = max(base_speed * 0.2, min(MAX_SPEED, leftSpeed))
    rightSpeed = max(base_speed * 0.2, min(MAX_SPEED, rightSpeed))
    
    previous_error = current_error
    
    return leftSpeed, rightSpeed

def execute_improved_turn(action):
    """
    IMPROVED: Execute a more reliable turn with better completion detection
    Uses closed-loop control to achieve target heading based on the pre-calculated expected_heading_change.
    Returns (leftSpeed, rightSpeed, phase_completed, turn_fully_completed)
    """
    global stop_counter, turn_counter, post_turn_counter
    global turn_start_heading, expected_heading_change, phi # phi and expected_heading_change are global
    global heading_tolerance, turn_kp 

    turn_speed_base = MAX_SPEED * 0.5 # Base speed for turning

    # Phase 1: Stop completely
    if stop_counter < STOP_DURATION:
        if stop_counter == 0:
            turn_start_heading = phi
            # expected_heading_change is already set in the main loop before calling this function
            print(f"Starting turn sequence: {action}")
            print(f"Initial heading: {phi:.3f} rad ({np.degrees(phi):.1f}°)")
            print(f"Expected change (from direction_to_robot_action): {expected_heading_change:.3f} rad ({np.degrees(expected_heading_change):.1f}°)")

        stop_counter += 1
        return 0, 0, False, False

    # Phase 2: Execute turn with closed-loop heading control
    # Calculate target heading based on the start heading and the pre-calculated angle difference
    target_heading = normalize_angle(turn_start_heading + expected_heading_change)
    current_heading_normalized = normalize_angle(phi)
    
    # Calculate error: difference between target and current heading
    # Use atan2 for correct shortest angular distance
    heading_error = np.arctan2(np.sin(target_heading - current_heading_normalized), np.cos(target_heading - current_heading_normalized))

    # Check for turn completion using tighter tolerance
    if abs(heading_error) < heading_tolerance:
        print("Turn completed based on precise heading!")
        # Proceed to post-turn pause immediately
        
        # Phase 3: Brief pause after turn
        if post_turn_counter < POST_TURN_PAUSE:
            post_turn_counter += 1
            return 0, 0, False, False # Keep motors off during settle
        else:
            print(f"Turn sequence completed! Final heading change: {np.degrees(normalize_angle(phi - turn_start_heading)):.1f}°")
            return 0, 0, True, True # Turn fully completed, ready for next state

    # PID-like control for turning
    turn_adjustment = turn_kp * heading_error

    left_speed_turn = -turn_adjustment
    right_speed_turn = turn_adjustment
    
    # Apply base speed to ensure turning motion and clamp
    # The direction of turn (left/right) is determined by the sign of heading_error
    # We ensure the base speed is applied in the correct relative direction for spinning
    if heading_error > 0: # Need to turn left
        left_speed_turn = min(-turn_speed_base, left_speed_turn) # Ensure negative
        right_speed_turn = max(turn_speed_base, right_speed_turn) # Ensure positive
    else: # Need to turn right
        left_speed_turn = max(turn_speed_base, left_speed_turn) # Ensure positive
        right_speed_turn = min(-turn_speed_base, right_speed_turn) # Ensure negative

    # Ensure speeds don't exceed MAX_SPEED and are always moving in the correct direction
    left_speed_turn = np.clip(left_speed_turn, -MAX_SPEED, MAX_SPEED)
    right_speed_turn = np.clip(right_speed_turn, -MAX_SPEED, MAX_SPEED)
    
    print(f"Turning phase ({action}): Heading Error: {np.degrees(heading_error):.1f}°")
    print(f"Current heading: {np.degrees(phi):.1f}°, Target heading: {np.degrees(target_heading):.1f}°")
    print(f"Turn speeds L:{left_speed_turn:.2f}, R:{right_speed_turn:.2f}")

    turn_counter += 1  
    return left_speed_turn, right_speed_turn, False, False


def search_for_line_after_turn(gsValues):
    """
    IMPROVED: Enhanced line search after turning
    """
    global line_search_counter
    
    line_search_counter += 1
    
    # Check if we can see the line
    line_error = calculate_line_error(gsValues)
    
    if line_error is not None:
        print(f"Line found after turn! Error: {line_error:.3f}")
        return True, advanced_line_following_control(gsValues)
    
    # If no line found, do a slow search pattern
    if line_search_counter < LINE_SEARCH_DURATION:
        search_speed = base_speed * 0.3
        
        # Slight oscillation to find line
        if (line_search_counter // 5) % 2 == 0:
            return False, (search_speed * 0.8, search_speed * 1.2)  # Turn slightly right
        else:
            return False, (search_speed * 1.2, search_speed * 0.8)  # Turn slightly left
    else:
        print("Line search timeout - proceeding with forward motion")
        return True, (base_speed * 0.5, base_speed * 0.5)

def reset_turn_counters():
    """Reset all turn-related counters"""
    global stop_counter, turn_counter, post_turn_counter, line_search_counter
    stop_counter = 0
    turn_counter = 0
    post_turn_counter = 0
    line_search_counter = 0

def handle_obstacle_avoidance():
    """
    Handle obstacle avoidance with 180-degree turn, forward drive, line reacquisition, and replanning.
    """
    global obstacle_avoidance_state, turn_start_angle, reset_timer
    global path, next_node_index, robot_current_grid_node, goal_node, replanning_cooldown
    global obstacle_blocked_node # The node that was the target when the obstacle was hit
    global last_confirmed_node # The last node the robot definitively passed through

    turn_speed_180 = MAX_SPEED * 0.4
    forward_after_turn_duration = 30 # Steps to drive forward after 180 turn
    line_reacquire_timeout = 50 # Max steps to find line after forward move

    if obstacle_avoidance_state == 'turning_180':
        target_heading_180 = normalize_angle(turn_start_angle + np.pi)
        current_heading_normalized = normalize_angle(phi)
        heading_error_180 = np.arctan2(np.sin(target_heading_180 - current_heading_normalized), np.cos(target_heading_180 - current_heading_normalized))

        if abs(heading_error_180) < heading_tolerance * 2: # Turn completed
            print("180 degree turn completed! Starting forward move...")
            obstacle_avoidance_state = 'driving_after_180_turn'
            reset_timer = 0 # Re-purpose reset_timer for this new state's duration
            return 0, 0
        else:
            turn_adjustment = turn_kp * heading_error_180
            left_speed = -np.clip(turn_adjustment, -turn_speed_180, turn_speed_180)
            right_speed = np.clip(turn_adjustment, -turn_speed_180, turn_speed_180)
            
            if heading_error_180 > 0: # Need to turn left
                left_speed = min(-abs(left_speed), -turn_speed_180)
                right_speed = max(abs(right_speed), turn_speed_180)
            else: # Need to turn right
                left_speed = max(abs(left_speed), turn_speed_180)
                right_speed = min(-abs(right_speed), -turn_speed_180)
            return left_speed, right_speed
            
    elif obstacle_avoidance_state == 'driving_after_180_turn':
        if reset_timer < forward_after_turn_duration:
            print("Driving forward after 180 turn...")
            reset_timer += 1
            return base_speed * 0.4, base_speed * 0.4 # Drive slowly forward
        else:
            print("Finished driving forward. Now attempting to reacquire line...")
            obstacle_avoidance_state = 'reacquiring_line'
            reset_timer = 0 # Reset for line reacquisition timeout
            return 0, 0 # Stop momentarily to re-assess
            
    elif obstacle_avoidance_state == 'reacquiring_line':
        gsValues = [gs[i].getValue() for i in range(3)] # Get current sensor values
        line_error = calculate_line_error(gsValues)
        
        if line_error is not None:
            print(f"Line reacquired after obstacle avoidance! Error: {line_error:.3f}")
            obstacle_avoidance_state = 'replanning_path' # Move to replanning state
            reset_timer = 0 # Reset for replanning cooldown
            return 0, 0 # Stop briefly before replanning
        else:
            reset_timer += 1
            if reset_timer < line_reacquire_timeout:
                # Perform a slight search pattern for the line (e.g., oscillate)
                search_speed = base_speed * 0.3
                if (reset_timer // 5) % 2 == 0:
                    return search_speed * 0.8, search_speed * 1.2 # Turn slightly right
                else:
                    return search_speed * 1.2, search_speed * 0.8 # Turn slightly left
            else:
                print("Line reacquisition timed out. Proceeding to replan anyway, might be off-line.")
                obstacle_avoidance_state = 'replanning_path'
                reset_timer = 0
                return 0, 0 # Stop to replan

    elif obstacle_avoidance_state == 'replanning_path':
        print(f"Starting replanning phase from {last_confirmed_node}, blocking {obstacle_blocked_node}...")
        
        # Replan from the last *confirmed* node, explicitly blocking the node that was the target when obstacle hit
        new_path = find_path_dijkstra(last_confirmed_node, goal_node, corrected_weighted_grid, {obstacle_blocked_node} if obstacle_blocked_node else set())
        
        if len(new_path) > 1:
            path = new_path
            robot_current_grid_node = last_confirmed_node # Reset robot's logical position to the last confirmed one
            next_node_index = 1  # Path starts from last_confirmed_node, so next target is index 1
            print(f"New path: {' -> '.join(path)}")
        else:
            print("Could not find alternative path! Robot might be stuck.")
            path = [last_confirmed_node] # Reset path to prevent errors
            next_node_index = 0
            
        obstacle_avoidance_state = 'normal' # Return to normal operation
        replanning_cooldown = REPLANNING_COOLDOWN_TIME # Apply cooldown before next obstacle detection
        robot_state = 'line_following' # Return to line following after successful replan
        
        return 0, 0
            
    return 0, 0 # Default return, should ideally not be reached if states are managed properly

# -------------------------------------------------------
# Main Control Loop

print(f"Starting navigation from {start_node} to {goal_node}")
print(f"Planned path: {' -> '.join(path)}")

iteration_count = 0

while robot.step(timestep) != -1:
    iteration_count += 1
    
    # Update encoder readings and calculate angular velocity
    oldEncoderValues = encoderValues
    encoderValues = [encoder[i].getValue() for i in range(2)]
    wl, wr = get_wheels_speed(encoderValues, oldEncoderValues, delta_t)
    u, w = get_robot_speeds(wl, wr, R, D)
    phi = update_heading(w, phi, delta_t)
    
    # Read sensors
    gsValues = [gs[i].getValue() for i in range(3)]
    psValues = [ps[i].getValue() for i in range(8)]
    
    # Obstacle detection
    obstacle_detected = psValues[0] > obstacle_threshold or psValues[7] > obstacle_threshold
    
    # Handle obstacle avoidance (priority over normal navigation)
    if replanning_cooldown > 0:
        replanning_cooldown -= 1
    
    if obstacle_avoidance_state == 'normal':
        if obstacle_detected and replanning_cooldown <= 0:
            print("OBSTACLE DETECTED! Starting 180 degree turn...")
            obstacle_avoidance_state = 'turning_180'
            turn_start_angle = phi # Store current heading to calculate 180 turn target
            leftSpeed, rightSpeed = 0, 0 # Stop immediately
            
            # Store the node that was targeted when the obstacle was hit
            if next_node_index < len(path):
                obstacle_blocked_node = path[next_node_index]
            else:
                obstacle_blocked_node = None # Should not happen if path is valid
                
        # else: speeds will be set by main state machine if not in obstacle avoidance
    elif obstacle_avoidance_state in ['turning_180', 'driving_after_180_turn', 'reacquiring_line', 'replanning_path']:
        leftSpeed, rightSpeed = handle_obstacle_avoidance()
        leftMotor.setVelocity(leftSpeed)
        rightMotor.setVelocity(rightSpeed)
        if obstacle_avoidance_state != 'normal': # Only skip if still actively avoiding
            continue # Skip main state machine if avoiding obstacle
    
    # Check if mission is complete (if the robot_current_grid_node *is* the goal)
    # This check is done after obstacle avoidance, before main navigation logic
    if robot_current_grid_node == goal_node:  
        if robot_state != 'stopping': # Prevent repeated messages
            print(f"Mission completed! Reached goal node: {robot_current_grid_node}.")
            robot_state = 'stopping' # Transition to stopping state
        leftMotor.setVelocity(0)
        rightMotor.setVelocity(0)
        continue # Stop robot and end loop
    
    # Ensure path is valid before proceeding with navigation
    if not path or next_node_index >= len(path):
        print("Path exhausted or invalid. Stopping robot.")
        leftMotor.setVelocity(0)
        rightMotor.setVelocity(0)
        robot_state = 'stopping' # Ensure state is stopping
        continue

    # The node we are currently aiming to reach in this segment
    target_node_for_segment = path[next_node_index]
    
    # Main state machine for navigation
    if robot_state == 'line_following':
        # Follow line until intersection is detected
        intersection_detected = detect_intersection(gsValues)
        
        if intersection_detected:
            cross_counter += 1
            if cross_counter >= CROSS_COUNTER_MIN:
                print(f"Intersection detected! About to drive through. Target for this segment: {target_node_for_segment}")
                robot_state = 'at_intersection_drive_through'
                intersection_drive_through_counter = 0
                cross_counter = 0 # Reset for next intersection
                integral_error = 0.0 # Reset line following PID for clean start on next segment
                error_history = []
        else:
            cross_counter = 0
            
        # Line following with PID control
        leftSpeed, rightSpeed = advanced_line_following_control(gsValues)

    # State for driving through the intersection
    elif robot_state == 'at_intersection_drive_through':
        if intersection_drive_through_counter < INTERSECTION_DRIVE_THROUGH_DURATION:
            leftSpeed, rightSpeed = base_speed * 0.4, base_speed * 0.4 # Drive slowly straight
            intersection_drive_through_counter += 1
        else:
            print(f"Finished driving through intersection. Updating robot_current_grid_node from {robot_current_grid_node} to {target_node_for_segment}.")
            robot_current_grid_node = target_node_for_segment # Robot is now logically at this node
            last_confirmed_node = robot_current_grid_node # IMPORTANT: Update last_confirmed_node here
            
            # Check if the node we just arrived at is the goal
            if robot_current_grid_node == goal_node:
                print(f"Reached goal node {robot_current_grid_node} after drive-through. Stopping.")
                robot_state = 'stopping'
                leftSpeed, rightSpeed = 0, 0
            else:
                # Advance to the NEXT segment's target node
                next_node_index += 1
                if next_node_index >= len(path): # Check if we ran out of path unexpectedly
                    print("Path exhausted after advancing. Stopping robot.")
                    robot_state = 'stopping'
                    leftSpeed, rightSpeed = 0, 0
                else:
                    next_segment_target_node = path[next_node_index] # This is the NEW target node for the next movement

                    # Determine the action needed to get from the NEW robot_current_grid_node to the NEW target_node
                    direction_needed = get_direction_to_next_node(robot_current_grid_node, next_segment_target_node, corrected_weighted_grid)
                    
                    if direction_needed:
                        # Calculate the precise action and angle needed for the turn
                        turn_action, expected_heading_change = direction_to_robot_action(direction_needed, phi)
                        
                        print(f"At intersection: New robot_current_grid_node={robot_current_grid_node}, Next segment's target={next_segment_target_node}")
                        print(f"Direction needed: {direction_needed}, Action: {turn_action}, Calculated Angle Diff: {np.degrees(expected_heading_change):.1f}°")
                        
                        if turn_action == 'forward':
                            print(f"Decided to go straight from {robot_current_grid_node} to {next_segment_target_node}. Back to line following.")
                            robot_state = 'line_following'
                            # Reset line following parameters (already done entering drive-through, but good to reinforce)
                            integral_error = 0.0
                            error_history = []
                            consecutive_no_line = 0
                            leftSpeed, rightSpeed = advanced_line_following_control(gsValues)
                        else:
                            robot_state = 'turning'
                            reset_turn_counters() # Prepare turn counters for the new turn
                            leftSpeed = 0
                            rightSpeed = 0
                    else:
                        print(f"ERROR: No valid next direction from {robot_current_grid_node} to {next_segment_target_node}. Stopping.")
                        robot_state = 'stopping'
                        leftSpeed, rightSpeed = 0, 0
            
    elif robot_state == 'turning':
        # Execute the turn based on the pre-calculated turn_action and expected_heading_change
        leftSpeed, rightSpeed, phase_completed, turn_fully_completed = execute_improved_turn(turn_action)
        
        if turn_fully_completed:
            print(f"Turn sequence completed! Transitioning to line search...")
            robot_state = 'post_turn_line_search'
            line_search_counter = 0 # Reset counter for line search phase
            leftSpeed = 0
            rightSpeed = 0
            
    elif robot_state == 'post_turn_line_search':
        # Search for the line after completing a turn
        line_found, (leftSpeed, rightSpeed) = search_for_line_after_turn(gsValues)
        
        if line_found:
            print("Line re-acquired after search. Resuming line following.")
            robot_state = 'line_following'
            # Speeds are already set by search_for_line_after_turn
        # If not found, search_for_line_after_turn will continue to provide speeds
    
    elif robot_state == 'stopping':
        leftSpeed, rightSpeed = 0, 0
        
    else:
        print(f"WARNING: Unknown robot state: {robot_state}. Stopping.")
        leftSpeed, rightSpeed = 0, 0

    leftMotor.setVelocity(leftSpeed)
    rightMotor.setVelocity(rightSpeed)

