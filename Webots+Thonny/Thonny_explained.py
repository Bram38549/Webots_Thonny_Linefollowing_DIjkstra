# A_algorthim.py
"""
ESP32 MicroPython Controller for Path Planning Robot
Implements Dijkstra's algorithm and robot control logic
Communicates with Webots via serial for Hardware-in-the-Loop simulation

Author: Generated for Robotics Assignment
Compatible with: MicroPython v1.25.0 on ESP32
Required: ujson module (built into most MicroPython versions)
save on esp-32 as main.py
"""

from machine import Pin, UART # Imports Pin for GPIO control and UART for serial communication
from time import sleep, ticks_ms, ticks_diff # Imports sleep for delays, ticks_ms for millisecond timestamps, and ticks_diff for calculating time differences
import ujson # Imports ujson for JSON serialization/deserialization
import math # Imports math module for mathematical operations like pi

class PathPlannerESP32: # Defines the main class for the ESP32 path planner
    def __init__(self):
        """Initialize ESP32 controller with path planning capabilities"""
        
        # Hardware setup
        self.led_board = Pin(2, Pin.OUT) # Initializes GPIO pin 2 as an output for a general LED on the board
        self.led_yellow = Pin(4, Pin.OUT) # Initializes GPIO pin 4 as an output for a yellow LED
        self.led_blue = Pin(23, Pin.OUT) # Initializes GPIO pin 23 as an output for a blue LED
        self.led_green = Pin(22, Pin.OUT) # Initializes GPIO pin 22 as an output for a green LED
        self.led_red = Pin(21, Pin.OUT) # Initializes GPIO pin 21 as an output for a red LED
        self.button_left = Pin(34, Pin.IN, Pin.PULL_DOWN) # Initializes GPIO pin 34 as an input for the left button with a pull-down resistor
        self.button_right = Pin(35, Pin.IN, Pin.PULL_DOWN) # Initializes GPIO pin 35 as an input for the right button with a pull-down resistor
        
        # Serial communication (initially UART0, will switch to UART1)
        self.uart = None # Initializes the UART object to None, to be set up later
        
        # Robot parameters
        self.MAX_SPEED = 6.28 # Defines the maximum speed of the robot
        self.base_speed = self.MAX_SPEED * 0.5 # Defines the base speed for line following
        self.line_threshold = 600 # Defines the threshold for ground sensor readings to detect a line
        
        # Navigation parameters
        self.current_node = "P4"  # Sets the initial current node (starting position)
        self.goal_node = "P6"     # Sets the goal node (destination)
        self.path = [] # Initializes an empty list to store the calculated path
        self.next_node_index = 1 # Initializes the index for the next node in the path
        self.last_confirmed_node = "P1" # Stores the last physically confirmed node the robot was at
        self.blocked_nodes_for_replan = set() # Initializes a set to store nodes blocked due to obstacles, for replanning
        
        # Robot state machine
        self.states = ['line_following', 'at_intersection', 'turning', 
                      'post_turn_search', 'obstacle_avoidance', 'turning_180', 'stopping'] # Defines the possible states of the robot's state machine
        self.current_state = 'line_following' # Sets the initial state of the robot
        
        # Control variables
        self.phi = -math.pi/2  # Initializes the robot's heading (phi), starting south (-π/2 radians)
        self.intersection_counter = 0 # Counter used for intersection detection and drive-through delay
        self.no_intersection_counter = 0 # Counter to debounce intersection counter reset
        self.turn_target_heading = 0 # Stores the target heading for a turn
        self.line_search_counter = 0 # Counter used during post-turn line searching
        self.intersection_processed = False # Flag to prevent double processing of the same intersection
        
        # Timing
        self.last_update = ticks_ms() # Stores the timestamp of the last update
        
        # Initialize grid map for pathfinding
        self._init_grid_map() # Calls a private method to initialize the grid map
        
    def _init_grid_map(self):
        """Initialize the weighted grid map for Dijkstra's algorithm"""
        self.grid_map = { # Defines the grid map as a dictionary of dictionaries
            # Parking spots (P nodes)
            "P1": {"S": ("A1", 2.0)}, # Node P1 connects to A1 to the South with a weight of 2.0
            "P2": {"S": ("A2", 2.0)}, # Node P2 connects to A2 to the South with a weight of 2.0
            "P3": {"S": ("A3", 2.0)}, # Node P3 connects to A3 to the South with a weight of 2.0
            "P4": {"S": ("A4", 2.0)}, # Node P4 connects to A4 to the South with a weight of 2.0
            "P5": {"N": ("E3", 2.0)}, # Node P5 connects to E3 to the North with a weight of 2.0
            "P6": {"N": ("E4", 2.0)}, # Node P6 connects to E4 to the North with a weight of 2.0
            "P7": {"N": ("E5", 2.0)}, # Node P7 connects to E5 to the North with a weight of 2.0
            "P8": {"N": ("E6", 2.0)}, # Node P8 connects to E6 to the North with a weight of 2.0
            
            # A row (connections for nodes in row A)
            "A1": {"N": ("P1", 2.0), "E": ("A2", 1.0), "S": ("C1", 3.0)}, # A1 connections
            "A2": {"N": ("P2", 2.0), "E": ("A3", 1.0), "W": ("A1", 1.0)}, # A2 connections
            "A3": {"N": ("P3", 2.0), "E": ("A4", 1.0), "W": ("A2", 1.0)}, # A3 connections
            "A4": {"N": ("P4", 2.0), "E": ("A5", 1.0), "W": ("A3", 1.0)}, # A4 connections
            "A5": {"E": ("A6", 1.0), "S": ("B1", 1.0), "W": ("A4", 1.0)}, # A5 connections
            "A6": {"S": ("B2", 1.0), "W": ("A5", 1.0)}, # A6 connections
            
            # B row (connections for nodes in row B)
            "B1": {"N": ("A5", 1.0), "E": ("B2", 1.0), "S": ("C2", 1.0)}, # B1 connections
            "B2": {"N": ("A6", 1.0), "S": ("C3", 1.0), "W": ("B1", 1.0)}, # B2 connections
            
            # C row (connections for nodes in row C)
            "C1": {"N": ("A1", 3.0), "E": ("C2", 1.0), "S": ("D1", 1.0)}, # C1 connections
            "C2": {"N": ("B1", 1.0), "E": ("C3", 1.0), "S": ("D2", 1.0), "W": ("C1", 1.0)}, # C2 connections
            "C3": {"N": ("B2", 1.0), "S": ("E6", 1.0), "W": ("C2", 1.0)}, # C3 connections
            
            # D row (connections for nodes in row D)
            "D1": {"N": ("C1", 1.0), "E": ("D2", 1.0), "S": ("E1", 1.0)}, # D1 connections
            "D2": {"N": ("C2", 1.0), "S": ("E2", 1.0), "W": ("D1", 5.0)}, # D2 connections
            
            # E row (connections for nodes in row E)
            "E1": {"N": ("D1", 1.0), "E": ("E2", 1.0)}, # E1 connections
            "E2": {"N": ("D2", 1.0), "E": ("E3", 1.0), "W": ("E1", 1.0)}, # E2 connections
            "E3": {"S": ("P5", 2.0), "E": ("E4", 1.0), "W": ("E2", 1.0)}, # E3 connections
            "E4": {"S": ("P6", 2.0), "E": ("E5", 1.0), "W": ("E3", 1.0)}, # E4 connections
            "E5": {"S": ("P7", 2.0), "E": ("E6", 1.0), "W": ("E4", 1.0)}, # E5 connections
            "E6": {"N": ("C3", 1.0), "S": ("P8", 2.0), "W": ("E5", 1.0)}, # E6 connections
        }
    
    def wait_for_button_and_setup_uart(self):
        """Wait for button press and setup UART communication"""
        while not self.button_left(): # Loops indefinitely until the left button is pressed
            sleep(0.25) # Pauses for 250 milliseconds
            self.led_board.value(not self.led_board.value()) # Toggles the board LED to indicate waiting
        
        # Switch to UART1 for USB communication
        self.uart = UART(1, 115200, tx=1, rx=3) # Initializes UART1 with a baud rate of 115200, TX on pin 1, RX on pin 3
        self.led_board.off() # Turns off the board LED
        return True # Returns True indicating successful setup
    
    def dijkstra_pathfind(self, start, goal, blocked_nodes=None):
        """
        Dijkstra's algorithm implementation for shortest path finding
        Returns list of nodes representing the path
        """
        if blocked_nodes is None: # Checks if blocked_nodes is None
            blocked_nodes = set() # Initializes an empty set if it is None
        
        if start not in self.grid_map or goal not in self.grid_map: # Checks if start or goal nodes are not in the grid map
            return [] # Returns an empty list if either is not found
        
        # Distance tracking
        distances = {} # Dictionary to store the shortest distance from the start node to all other nodes
        previous = {} # Dictionary to store the predecessor node in the shortest path
        unvisited = set() # Set to keep track of unvisited nodes
        
        # Initialize distances
        for node in self.grid_map: # Iterates through all nodes in the grid map
            distances[node] = float('inf') if node != start else 0 # Sets distance to infinity for all nodes except start, which is 0
            previous[node] = None # Initializes previous node to None for all nodes
            unvisited.add(node) # Adds all nodes to the unvisited set
        
        while unvisited: # Continues as long as there are unvisited nodes
            # Find node with minimum distance
            current = min(unvisited, key=lambda node: distances[node]) # Finds the unvisited node with the smallest distance
            
            if distances[current] == float('inf'): # If the smallest distance is infinity, no path exists to remaining nodes
                break  # No path exists, so break the loop
                
            if current == goal: # If the current node is the goal node
                # Reconstruct path
                path = [] # Initializes an empty list to store the path
                while current is not None: # Loops until the start node is reached
                    path.append(current) # Adds the current node to the path
                    current = previous[current] # Moves to the previous node
                return path[::-1]  # Returns the reversed path (start to goal)
            
            unvisited.remove(current) # Removes the current node from the unvisited set
            
            # Check neighbors
            for direction, neighbor_info in self.grid_map[current].items(): # Iterates through neighbors of the current node
                if neighbor_info and neighbor_info[0] not in blocked_nodes: # Checks if neighbor info exists and neighbor is not blocked
                    neighbor, weight = neighbor_info # Unpacks neighbor and weight
                    if neighbor in unvisited: # If the neighbor is unvisited
                        alt_distance = distances[current] + weight # Calculates alternative distance to the neighbor
                        if alt_distance < distances[neighbor]: # If the alternative distance is shorter
                            distances[neighbor] = alt_distance # Updates the distance to the neighbor
                            previous[neighbor] = current # Sets the current node as the previous node for the neighbor
        
        return []  # Returns an empty list if no path is found
    
    def get_direction_to_next_node(self, current, next_node):
        """Get direction (N,E,S,W) to move from current to next node"""
        if current not in self.grid_map: # Checks if the current node is not in the grid map
            return None # Returns None if not found
        
        for direction, neighbor_info in self.grid_map[current].items(): # Iterates through neighbors of the current node
            if neighbor_info and neighbor_info[0] == next_node: # Checks if the neighbor matches the next node
                return direction # Returns the direction to that neighbor
        return None # Returns None if no direct path is found
    
    def normalize_angle(self, angle):
        """Normalize angle to [-π, π]"""
        while angle > math.pi: # Loops while angle is greater than pi
            angle -= 2 * math.pi # Subtracts 2*pi to normalize
        while angle < -math.pi: # Loops while angle is less than -pi
            angle += 2 * math.pi # Adds 2*pi to normalize
        return angle # Returns the normalized angle
    
    def direction_to_action(self, direction):
        """Convert grid direction to robot action based on current heading"""
        direction_angles = { # Defines a dictionary mapping directions to their corresponding angles
            'N': math.pi/2,    # North: π/2
            'E': 0,         # East: 0
            'S': -math.pi/2,   # South: -π/2
            'W': math.pi    # West: π
        }
        
        if direction not in direction_angles: # Checks if the given direction is not in the defined directions
            return 'forward', 0 # Returns 'forward' and 0 angle difference if direction is invalid
        
        target_angle = direction_angles[direction] # Gets the target angle for the given direction
        current_heading = self.normalize_angle(self.phi) # Gets the current normalized heading of the robot
        target_angle = self.normalize_angle(target_angle) # Normalizes the target angle
        
        # Calculate angle difference
        angle_diff = target_angle - current_heading # Calculates the raw angle difference
        angle_diff = self.normalize_angle(angle_diff) # Normalizes the angle difference
        
        # Determine action
        forward_tolerance = 0.2  # Defines a tolerance for considering movement as 'forward'
        if abs(angle_diff) < forward_tolerance: # If the absolute angle difference is within forward tolerance
            return 'forward', angle_diff # Returns 'forward' action
        elif abs(angle_diff) > 2.8:  # If the absolute angle difference is close to 180 degrees
            return 'turn_around', angle_diff # Returns 'turn_around' action
        elif angle_diff > 0: # If angle difference is positive (turn left)
            return 'turn_left', angle_diff # Returns 'turn_left' action
        else: # If angle difference is negative (turn right)
            return 'turn_right', angle_diff # Returns 'turn_right' action
    
    def detect_intersection(self, gs_values):
        """Detect intersection using ground sensors"""
        line_left = gs_values[2] < self.line_threshold # Checks if the left ground sensor detects a line
        line_center = gs_values[1] < self.line_threshold # Checks if the center ground sensor detects a line
        line_right = gs_values[0] < self.line_threshold # Checks if the right ground sensor detects a line
        
        # Reverted to stricter detection for clear intersections
        return line_left and line_center and line_right # Returns True if all three sensors detect a line (intersection)
    
    def calculate_line_error(self, gs_values):
        """Calculate line following error"""
        # Invert and normalize sensor values
        right_val = max(0, self.line_threshold - gs_values[0]) / self.line_threshold # Normalizes the right sensor value
        center_val = max(0, self.line_threshold - gs_values[1]) / self.line_threshold # Normalizes the center sensor value
        left_val = max(0, self.line_threshold - gs_values[2]) / self.line_threshold # Normalizes the left sensor value
        
        total_activation = right_val + center_val + left_val # Calculates the total activation of the sensors
        
        if total_activation < 0.1: # If total activation is very low (no line detected)
            return None  # Returns None
        
        # Calculate weighted position: -1 (left) to +1 (right)
        error = (right_val * 1.0 + center_val * 0.0 + left_val * (-1.0)) / total_activation # Calculates the weighted error for line following
        return error # Returns the calculated error
    
    def line_following_control(self, gs_values):
        """PID-based line following control"""
        error = self.calculate_line_error(gs_values) # Calculates the line following error
        
        if error is None: # If no line is detected
            # No line - slow forward search
            return self.base_speed * 0.3, self.base_speed * 0.3 # Returns slow forward speeds for searching
        
        # Simple proportional control
        kp = 1.5 # Defines the proportional gain for PID control
        max_turn_diff = self.MAX_SPEED * 0.3 # Defines the maximum allowed turn speed difference
        
        turn_adjustment = kp * error * max_turn_diff # Calculates the turn adjustment based on error and gain
        
        left_speed = self.base_speed - turn_adjustment # Calculates the left wheel speed
        right_speed = self.base_speed + turn_adjustment # Calculates the right wheel speed
        
        # Clamp speeds
        left_speed = max(self.base_speed * 0.2, min(self.MAX_SPEED, left_speed)) # Clamps the left speed within limits
        right_speed = max(self.base_speed * 0.2, min(self.MAX_SPEED, right_speed)) # Clamps the right speed within limits
        
        return left_speed, right_speed # Returns the calculated left and right speeds
    
    def execute_turn(self, target_heading):
        """Execute turn to reach target heading with proportional control."""
        current_heading = self.normalize_angle(self.phi) # Gets the current normalized heading
        target_heading = self.normalize_angle(target_heading) # Normalizes the target heading
        
        heading_error = target_heading - current_heading # Calculates the raw heading error
        heading_error = self.normalize_angle(heading_error) # Normalizes the heading error
        
        # Define a tolerance for turn completion
        turn_tolerance = 0.05 # Defines the tolerance for considering a turn complete
        
        if abs(heading_error) < turn_tolerance:  # If the absolute heading error is within tolerance
            return 0, 0, True # Returns zero speeds and True for turn completion
        
        # Proportional control for turning speed
        turn_kp = 1.2 # Defines the proportional gain for turning speed
        
        # Calculate base turn speed proportional to the error
        # Clamp turn_speed to a reasonable range
        raw_turn_speed = turn_kp * heading_error # Calculates raw turn speed
        
        # Max turn speed
        max_abs_turn_speed = self.MAX_SPEED * 0.7 # Defines the maximum absolute turn speed
        # Min turn speed to ensure it actually moves when error is small
        min_abs_turn_speed = self.base_speed * 0.1 # Defines the minimum absolute turn speed
        
        # Apply limits
        turn_speed = max(min_abs_turn_speed, min(max_abs_turn_speed, abs(raw_turn_speed))) # Clamps the turn speed within limits
        
        # Determine direction
        if heading_error > 0:  # If heading error is positive (turn left)
            left_speed = -turn_speed # Sets left wheel speed for left turn
            right_speed = turn_speed # Sets right wheel speed for left turn
        else:  # If heading error is negative (turn right)
            left_speed = turn_speed # Sets left wheel speed for right turn
            right_speed = -turn_speed # Sets right wheel speed for right turn
            
        return left_speed, right_speed, False # Returns calculated speeds and False for turn not complete
    
    def process_sensor_data(self, sensor_data):
        """Process incoming sensor data and update robot state"""
        gs_values = sensor_data.get('gs', [500, 500, 500]) # Gets ground sensor values, defaults if not present
        ps_values = sensor_data.get('ps', [0] * 8) # Gets proximity sensor values, defaults if not present
        self.phi = sensor_data.get('h', self.phi) # Gets robot heading, defaults to current phi if not present
        
        # Update LED indicators based on ground sensors
        self.led_blue.value(gs_values[2] < self.line_threshold)   # Sets blue LED based on left ground sensor
        self.led_green.value(gs_values[1] < self.line_threshold)  # Sets green LED based on center ground sensor
        self.led_red.value(gs_values[0] < self.line_threshold)    # Sets red LED based on right ground sensor
        
        # Obstacle detection - only front sensors
        obstacle_detected = ps_values[0] > 85 or ps_values[7] > 85 # Checks if front proximity sensors detect an obstacle
        
        return gs_values, ps_values, obstacle_detected # Returns processed sensor values and obstacle detection status
    
    def state_machine(self, gs_values, ps_values, obstacle_detected):
        """Main robot state machine"""
        left_speed, right_speed = 0, 0 # Initializes speeds to zero
        
        # --- Debug variables to send to Webots ---
        int_det_debug = False # Initializes intersection detection debug flag
        int_counter_debug = self.intersection_counter # Initializes intersection counter debug value
        # ----------------------------------------
        
        # Handle obstacle avoidance (highest priority)
        if obstacle_detected: # If an obstacle is detected
            if self.current_state != 'obstacle_avoidance' and self.current_state != 'turning_180': # If not already in obstacle avoidance or turning 180
                self.current_state = 'turning_180' # Changes state to turning 180
                self.turn_target_heading = self.normalize_angle(self.phi + math.pi) # Sets target heading for a 180-degree turn
                
                # Block the node the robot was trying to reach for future replanning
                if self.next_node_index < len(self.path): # If there's a next node in the path
                    self.blocked_nodes_for_replan.add(self.path[self.next_node_index]) # Adds the next node to blocked nodes for replanning
                left_speed, right_speed = 0, 0 # Stops the robot while transitioning
            # If already in turning_180, continue the turn
            elif self.current_state == 'turning_180': # If already in turning 180 state
                left_speed, right_speed, turn_complete = self.execute_turn(self.turn_target_heading) # Executes the turn
                if turn_complete: # If the 180-degree turn is complete
                    # NIEUWE LOGICA: Controleer of het knooppunt dat hij probeerde te bereiken het doel was
                    if self.next_node_index < len(self.path) and self.path[self.next_node_index] == self.goal_node: # Checks if the next node was the goal
                        self.current_node = self.goal_node # Manually sets current node to goal
                        self.current_state = 'stopping' # Sets state to stopping (mission complete)
                        self.blocked_nodes_for_replan.clear() # Clears blocked nodes
                        return self.current_state, left_speed, right_speed, gs_values, int_det_debug, int_counter_debug # Returns current state and debug info
                    
                    node_robot_was_trying_to_reach = None # Initializes variable for the node the robot was trying to reach
                    if self.next_node_index < len(self.path): # If there's a next node in the path
                        node_robot_was_trying_to_reach = self.path[self.next_node_index] # Sets it to the next node
                    else:
                        # Fallback if next_node_index is out of bounds (e.g., at goal)
                        node_robot_was_trying_to_reach = self.current_node # Defaults to current node if out of bounds

                    if node_robot_was_trying_to_reach: # If a node was identified
                        # Find a path from the last confirmed node (where robot physically is)
                        # to the goal, avoiding the node it just couldn't reach.
                        temp_new_path = self.dijkstra_pathfind(self.current_node, self.goal_node, self.blocked_nodes_for_replan) # Finds a new path avoiding blocked nodes

                        if temp_new_path: # If a new path is found
                            # Construct the user-requested path: [node_robot_was_trying_to_reach, current_node, ...]
                            # This means prepending the 'node_robot_was_trying_to_reach' to the path found from 'current_node'.
                            final_new_path = [node_robot_was_trying_to_reach] + temp_new_path # Prepends the blocked node to the new path
                            
                            self.path = final_new_path # Updates the robot's path
                            self.current_node = final_new_path[0] # Sets current node to the first node of the new path
                            self.next_node_index = 1 # Sets next node index to the second node
                            self.current_state = 'line_following' # Resumes line following
                            self.blocked_nodes_for_replan.clear() # Clears blocked nodes
                        else:
                            self.current_state = 'stopping' # If no new path, stop
                    else:
                        self.current_state = 'stopping' # If no node identified, stop
        
        # If obstacle is cleared while in obstacle_avoidance (and not yet turning 180)
        elif self.current_state == 'obstacle_avoidance' and not obstacle_detected: # If in obstacle avoidance and obstacle is cleared
            self.current_state = 'line_following' # Changes state back to line following
            self.blocked_nodes_for_replan.clear() # Clears blocked nodes (optional)

        # Check if mission is complete
        if self.current_node == self.goal_node: # If current node is the goal node
            self.current_state = 'stopping' # Sets state to stopping
            return 'mission_complete', 0, 0, gs_values, int_det_debug, int_counter_debug # Returns mission complete action and debug info
        
        # Main state machine transitions (excluding obstacle detection which is high priority)
        if self.current_state == 'line_following': # If in line following state
            if self.detect_intersection(gs_values): # If an intersection is detected
                int_det_debug = True # Sets debug flag
                self.intersection_counter += 1 # Increments intersection counter
                self.no_intersection_counter = 0 # Resets no-intersection counter
                
                # Only process this intersection if it hasn't been processed yet
                if self.intersection_counter >= 3 and not self.intersection_processed: # If intersection counter is high enough and not processed
                    self.current_state = 'at_intersection' # Changes state to at intersection
                    
                    # Update current_node to the node it just arrived at.
                    if self.next_node_index < len(self.path): # If there's a next node in the path
                        self.current_node = self.path[self.next_node_index] # Updates current node to the next node in path
                        self.last_confirmed_node = self.current_node # Updates last confirmed node
                        self.intersection_processed = True # Marks intersection as processed
                    else:
                        # No more nodes in path.
                        self.current_state = 'stopping' # If no more nodes, stop
                        self.intersection_counter = 0 # Resets counter
                        return self.current_state, left_speed, right_speed, gs_values, int_det_debug, int_counter_debug # Returns current state and debug info
                    
                    # INCREASE THIS VALUE to make the robot drive further into the intersection
                    self.intersection_counter = 15 # Sets delay cycles for driving through intersection
                                                  # This 12 is now the drive-through duration
            else: # No intersection detected
                self.no_intersection_counter += 1 # Increments no-intersection counter
                # Only reset intersection_counter if no intersection detected for a few cycles
                if self.no_intersection_counter >= 2: # If no intersection for a few cycles
                    self.intersection_counter = 0 # Resets intersection counter
                    self.intersection_processed = False # Resets processed flag
            
            left_speed, right_speed = self.line_following_control(gs_values) # Performs line following control
        
        elif self.current_state == 'at_intersection': # If in at intersection state
            left_speed, right_speed = self.base_speed * 0.4, self.base_speed * 0.4 # Drives slowly
            if self.intersection_counter > 0: # If drive-through counter is active
                # Continue driving through the intersection for a few more cycles
                self.intersection_counter -= 1 # Decrements counter
            else: # Drive-through is complete
                # Now determine the next action and transition out of 'at_intersection'
                
                # Increment next_node_index *only after* the drive-through period is over
                # and before determining the path to the *next* segment.
                self.next_node_index += 1 # Increments next node index

                if self.next_node_index >= len(self.path): # If no more nodes in path
                    self.current_state = 'stopping' # Stop
                else:
                    next_target = self.path[self.next_node_index] # Gets the next target node
                    direction = self.get_direction_to_next_node(self.current_node, next_target) # Gets direction to next node
                    
                    if direction: # If a direction is found
                        action, angle_diff = self.direction_to_action(direction) # Determines action and angle difference
                        if action == 'forward': # If action is forward
                            self.current_state = 'line_following' # Changes state to line following
                        else:
                            self.current_state = 'turning' # Changes state to turning
                            self.turn_target_heading = self.normalize_angle(self.phi + angle_diff) # Sets target heading for turn
                    else:
                        self.current_state = 'stopping' # If no direction, stop
                
                # Reset intersection_counter to 0 (already is, but for clarity)
                self.intersection_counter = 0 # Resets intersection counter
        
        elif self.current_state == 'turning': # If in turning state
            left_speed, right_speed, turn_complete = self.execute_turn(self.turn_target_heading) # Executes the turn
            
            if turn_complete: # If turn is complete
                self.current_state = 'post_turn_search' # Changes state to post-turn search
                self.line_search_counter = 0 # Resets line search counter
        
        elif self.current_state == 'post_turn_search': # If in post-turn search state
            error = self.calculate_line_error(gs_values) # Calculates line error
            
            if error is not None: # If a line is detected
                self.current_state = 'line_following' # Changes state to line following
                left_speed, right_speed = self.line_following_control(gs_values) # Performs line following control
            else:
                self.line_search_counter += 1 # Increments line search counter
                if self.line_search_counter < 30: # If search counter is within limits
                    # Search pattern
                    search_speed = self.base_speed * 0.3 # Sets search speed
                    if (self.line_search_counter // 5) % 2 == 0: # Alternates search direction
                        left_speed, right_speed = search_speed * 0.8, search_speed * 1.2 # Turns slightly right
                    else:
                        left_speed, right_speed = search_speed * 1.2, search_speed * 0.8 # Turns slightly left
                else:
                    # After extensive search, revert to line following
                    self.current_state = 'line_following' # Changes state to line following
                    left_speed, right_speed = self.base_speed * 0.5, self.base_speed * 0.5 # Sets speeds to base speed
        
        elif self.current_state == 'obstacle_avoidance': # If in obstacle avoidance state
            left_speed, right_speed = 0, 0 # Stops the robot
            # This state is now primarily for pausing before turning 180 or if obstacle clears without 180 turn
            # If the obstacle is no longer detected, transition back to line following
            if not obstacle_detected: # If obstacle is no longer detected
                self.current_state = 'line_following' # Changes state to line following
                self.blocked_nodes_for_replan.clear() # Clears blocked nodes
        
        elif self.current_state == 'turning_180': # If in turning 180 state
            # This state is handled at the top due to its high priority when an obstacle is present
            # If we reach here, it means the obstacle *might* have cleared, but the turn needs to complete.
            # However, the logic for replanning is now after the turn completes.
            # So, we just ensure it keeps turning if it's in this state.
            left_speed, right_speed, turn_complete = self.execute_turn(self.turn_target_heading) # Executes the turn
            if turn_complete: # If turn is complete
                # NIEUWE LOGICA: Deze controle wordt hier gedupliceerd, omdat de obstakeldetectie de status kan instellen
                # en vervolgens de draai onmiddellijk in dezelfde of de volgende cyclus kan voltooien.
                # Het is veiliger om het hier ook te hebben.
                if self.next_node_index < len(self.path) and self.path[self.next_node_index] == self.goal_node: # Checks if next node is goal
                    self.current_node = self.goal_node # Sets current node to goal
                    self.current_state = 'stopping' # Stops
                    self.blocked_nodes_for_replan.clear() # Clears blocked nodes
                    return self.current_state, left_speed, right_speed, gs_values, int_det_debug, int_counter_debug # Returns debug info

                node_robot_was_trying_to_reach = None # Initializes variable
                if self.next_node_index < len(self.path): # If there's a next node
                    node_robot_was_trying_to_reach = self.path[self.next_node_index] # Sets it
                else:
                    # Fallback if next_node_index is out of bounds (e.g., at goal)
                    node_robot_was_trying_to_reach = self.current_node # Defaults to current node

                if node_robot_was_trying_to_reach: # If a node was identified
                    # Find a path from the last confirmed node (where robot physically is)
                    # to the goal, avoiding the node it just couldn't reach.
                    temp_new_path = self.dijkstra_pathfind(self.current_node, self.goal_node, self.blocked_nodes_for_replan) # Finds new path

                    if temp_new_path: # If new path found
                        # Construct the user-requested path: [node_robot_was_trying_to_reach, current_node, ...]
                        # This means prepending the 'node_robot_was_trying_to_reach' to the path found from 'current_node'.
                        final_new_path = [node_robot_was_trying_to_reach] + temp_new_path # Prepends blocked node
                        
                        self.path = final_new_path # Updates path
                        self.current_node = final_new_path[0] # Sets current node
                        self.next_node_index = 1 # Sets next node index
                        self.current_state = 'line_following' # Resumes line following
                        self.blocked_nodes_for_replan.clear() # Clears blocked nodes
                    else:
                        self.current_state = 'stopping' # If no new path, stop
                else:
                    self.current_state = 'stopping' # If no node identified, stop
        
        elif self.current_state == 'stopping': # If in stopping state
            left_speed, right_speed = 0, 0 # Sets speeds to zero
        
        # --- Return updated debug values ---
        return self.current_state, left_speed, right_speed, gs_values, int_det_debug, int_counter_debug # Returns current state, speeds, and debug info

    def run(self):
        """Main control loop"""
        # Wait for button and setup communication
        if not self.wait_for_button_and_setup_uart(): # Waits for button press and sets up UART
            return # Returns if setup fails
        
        # Calculate initial path
        self.path = self.dijkstra_pathfind(self.current_node, self.goal_node) # Calculates the initial path
        if not self.path: # If no path is found
            return # Returns
        
        self.led_yellow.on()  # Turns on yellow LED to indicate path planning complete
        
        while True: # Enters an infinite loop for the main control
            try:
                # Receive sensor data from Webots
                if self.uart.any(): # Checks if there's any data in the UART buffer
                    line = self.uart.readline() # Reads a line from UART
                    if line: # If a line is read
                        sensor_data = ujson.loads(line.decode('utf-8').strip()) # Decodes and parses the JSON sensor data
                        
                        # Process sensor data
                        gs_values, ps_values, obstacle_detected = self.process_sensor_data(sensor_data) # Processes sensor data
                        
                        # Run state machine - now receives debug info
                        action, left_speed, right_speed, gs_debug, int_det_debug, int_counter_debug = self.state_machine(gs_values, ps_values, obstacle_detected) # Runs the state machine
                        
                        # Send command back to Webots
                        command = { # Creates a dictionary for the command to send back
                            'action': action, # The action to perform
                            'left_speed': left_speed, # Left wheel speed
                            'right_speed': right_speed, # Right wheel speed
                            'current_node': self.current_node, # Current node of the robot
                            'path': self.path, # The current path
                            'gs_debug': gs_debug, # Ground sensor debug info
                            'int_det_debug': int_det_debug, # Intersection detection debug info
                            'int_counter_debug': int_counter_debug # Intersection counter debug info
                        }
                        
                        command_str = ujson.dumps(command) + '\n' # Converts the command dictionary to a JSON string
                        self.uart.write(command_str.encode('utf-8')) # Sends the encoded command string over UART
                        
                        # Check for mission completion
                        if action == 'mission_complete': # If the action is mission complete
                            self.led_board.on() # Turns on the board LED
                            break # Exits the control loop
                
                sleep(0.02)  # Pauses for 20 milliseconds (50Hz control loop)
                
            except Exception as e: # Catches any exceptions
                sleep(0.1) # Pauses for 100 milliseconds in case of an error

# Main execution
def main():
    """Main function"""
    controller = PathPlannerESP32() # Creates an instance of the PathPlannerESP32 class
    try:
        controller.run() # Runs the main control loop
    except KeyboardInterrupt: # Catches KeyboardInterrupt (e.g., Ctrl+C)
        pass # Ignores the interrupt
    except Exception as e: # Catches any other exceptions
        pass # Ignores the exception

if __name__ == "__main__": # Checks if the script is being run directly
    main() # Calls the main function
