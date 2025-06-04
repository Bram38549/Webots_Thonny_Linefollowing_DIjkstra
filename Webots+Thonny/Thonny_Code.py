"""
ESP32 MicroPython Controller for Path Planning Robot
Implements Dijkstra's algorithm and robot control logic
Communicates with Webots via serial for Hardware-in-the-Loop simulation

Author: Wouter van den Bosch and Bram Smit
Compatible with: MicroPython v1.25.0 on ESP32
Required: ujson module (built into most MicroPython versions)
save on esp-32 as main.py, then close thonny and run the webots script
"""

from machine import Pin, UART
from time import sleep, ticks_ms, ticks_diff
import ujson
import math

class PathPlannerESP32:
    def __init__(self):
        """Initialize ESP32 controller with path planning capabilities"""
        
        # Hardware setup
        self.led_board = Pin(2, Pin.OUT)
        self.led_yellow = Pin(4, Pin.OUT)
        self.led_blue = Pin(23, Pin.OUT)
        self.led_green = Pin(22, Pin.OUT)
        self.led_red = Pin(21, Pin.OUT)
        self.button_left = Pin(34, Pin.IN, Pin.PULL_DOWN)
        self.button_right = Pin(35, Pin.IN, Pin.PULL_DOWN)
        
        # Serial communication (initially UART0, will switch to UART1)
        self.uart = None
        
        # Robot parameters
        self.MAX_SPEED = 6.28
        self.base_speed = self.MAX_SPEED * 0.5
        self.line_threshold = 600
        
        # Navigation parameters
        self.current_node = "P4"  # Changed to P1 as the starting position
        self.goal_node = "P6"     # Goal position
        self.path = []
        self.next_node_index = 1
        self.last_confirmed_node = "P1" # Assuming P1 is also the last confirmed node initially
        self.blocked_nodes_for_replan = set() # NEW: To store nodes blocked due to obstacles
        
        # Robot state machine
        self.states = ['line_following', 'at_intersection', 'turning', 
                      'post_turn_search', 'obstacle_avoidance', 'turning_180', 'stopping']
        self.current_state = 'line_following'
        
        # Control variables
        self.phi = -math.pi/2  # Robot heading (starts facing South: -π/2)
        self.intersection_counter = 0 # Used for intersection detection AND drive-through delay
        self.no_intersection_counter = 0 # New: to debounce intersection counter reset
        self.turn_target_heading = 0
        self.line_search_counter = 0
        self.intersection_processed = False # New flag to prevent double processing of same intersection
        
        # Timing
        self.last_update = ticks_ms()
        
        # Initialize grid map for pathfinding
        self._init_grid_map()
        
    def _init_grid_map(self):
        """Initialize the weighted grid map for Dijkstra's algorithm"""
        self.grid_map = {
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
            "A5": {"E": ("A6", 1.0), "S": ("B1", 1.0), "W": ("A4", 1.0)},
            "A6": {"S": ("B2", 1.0), "W": ("A5", 1.0)},
            
            # B row
            "B1": {"N": ("A5", 1.0), "E": ("B2", 1.0), "S": ("C2", 1.0)},
            "B2": {"N": ("A6", 1.0), "S": ("C3", 1.0), "W": ("B1", 1.0)},
            
            # C row
            "C1": {"N": ("A1", 3.0), "E": ("C2", 1.0), "S": ("D1", 1.0)},
            "C2": {"N": ("B1", 1.0), "E": ("C3", 1.0), "S": ("D2", 1.0), "W": ("C1", 1.0)},
            "C3": {"N": ("B2", 1.0), "S": ("E6", 1.0), "W": ("C2", 1.0)},
            
            # D row
            "D1": {"N": ("C1", 1.0), "E": ("D2", 1.0), "S": ("E1", 1.0)},
            "D2": {"N": ("C2", 1.0), "S": ("E2", 1.0), "W": ("D1", 5.0)},
            
            # E row
            "E1": {"N": ("D1", 1.0), "E": ("E2", 1.0)},
            "E2": {"N": ("D2", 1.0), "E": ("E3", 1.0), "W": ("E1", 1.0)},
            "E3": {"S": ("P5", 2.0), "E": ("E4", 1.0), "W": ("E2", 1.0)},
            "E4": {"S": ("P6", 2.0), "E": ("E5", 1.0), "W": ("E3", 1.0)},
            "E5": {"S": ("P7", 2.0), "E": ("E6", 1.0), "W": ("E4", 1.0)},
            "E6": {"N": ("C3", 1.0), "S": ("P8", 2.0), "W": ("E5", 1.0)},
        }
    
    def wait_for_button_and_setup_uart(self):
        """Wait for button press and setup UART communication"""
        while not self.button_left():
            sleep(0.25)
            self.led_board.value(not self.led_board.value())
        
        # Switch to UART1 for USB communication
        self.uart = UART(1, 115200, tx=1, rx=3)
        self.led_board.off()
        return True
    
    def dijkstra_pathfind(self, start, goal, blocked_nodes=None):
        """
        Dijkstra's algorithm implementation for shortest path finding
        Returns list of nodes representing the path
        """
        if blocked_nodes is None:
            blocked_nodes = set()
        
        if start not in self.grid_map or goal not in self.grid_map:
            return []
        
        # Distance tracking
        distances = {}
        previous = {}
        unvisited = set()
        
        # Initialize distances
        for node in self.grid_map:
            distances[node] = float('inf') if node != start else 0
            previous[node] = None
            unvisited.add(node)
        
        while unvisited:
            # Find node with minimum distance
            current = min(unvisited, key=lambda node: distances[node])
            
            if distances[current] == float('inf'):
                break  # No path exists
                
            if current == goal:
                # Reconstruct path
                path = []
                while current is not None:
                    path.append(current)
                    current = previous[current]
                return path[::-1]  # Reverse to get start->goal path
            
            unvisited.remove(current)
            
            # Check neighbors
            for direction, neighbor_info in self.grid_map[current].items():
                if neighbor_info and neighbor_info[0] not in blocked_nodes:
                    neighbor, weight = neighbor_info
                    if neighbor in unvisited:
                        alt_distance = distances[current] + weight
                        if alt_distance < distances[neighbor]:
                            distances[neighbor] = alt_distance
                            previous[neighbor] = current
        
        return []  # No path found
    
    def get_direction_to_next_node(self, current, next_node):
        """Get direction (N,E,S,W) to move from current to next node"""
        if current not in self.grid_map:
            return None
        
        for direction, neighbor_info in self.grid_map[current].items():
            if neighbor_info and neighbor_info[0] == next_node:
                return direction
        return None
    
    def normalize_angle(self, angle):
        """Normalize angle to [-π, π]"""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle
    
    def direction_to_action(self, direction):
        """Convert grid direction to robot action based on current heading"""
        direction_angles = {
            'N': math.pi/2,    # π/2
            'E': 0,         # 0
            'S': -math.pi/2,   # -π/2
            'W': math.pi    # π
        }
        
        if direction not in direction_angles:
            return 'forward', 0
        
        target_angle = direction_angles[direction]
        current_heading = self.normalize_angle(self.phi)
        target_angle = self.normalize_angle(target_angle)
        
        # Calculate angle difference
        angle_diff = target_angle - current_heading
        angle_diff = self.normalize_angle(angle_diff)
        
        # Determine action
        forward_tolerance = 0.2  # ~11.4 degrees, more robust for "straight"
        if abs(angle_diff) < forward_tolerance:
            return 'forward', angle_diff
        elif abs(angle_diff) > 2.8:  # ~160 degrees (close to 180)
            return 'turn_around', angle_diff
        elif angle_diff > 0:
            return 'turn_left', angle_diff
        else:
            return 'turn_right', angle_diff
    
    def detect_intersection(self, gs_values):
        """Detect intersection using ground sensors"""
        line_left = gs_values[2] < self.line_threshold
        line_center = gs_values[1] < self.line_threshold
        line_right = gs_values[0] < self.line_threshold
        
        # Reverted to stricter detection for clear intersections
        return line_left and line_center and line_right
    
    def calculate_line_error(self, gs_values):
        """Calculate line following error"""
        # Invert and normalize sensor values
        right_val = max(0, self.line_threshold - gs_values[0]) / self.line_threshold
        center_val = max(0, self.line_threshold - gs_values[1]) / self.line_threshold
        left_val = max(0, self.line_threshold - gs_values[2]) / self.line_threshold
        
        total_activation = right_val + center_val + left_val
        
        if total_activation < 0.1:
            return None  # No line detected
        
        # Calculate weighted position: -1 (left) to +1 (right)
        error = (right_val * 1.0 + center_val * 0.0 + left_val * (-1.0)) / total_activation
        return error
    
    def line_following_control(self, gs_values):
        """PID-based line following control"""
        error = self.calculate_line_error(gs_values)
        
        if error is None:
            # No line - slow forward search
            return self.base_speed * 0.3, self.base_speed * 0.3
        
        # Simple proportional control
        kp = 1.5
        max_turn_diff = self.MAX_SPEED * 0.3
        
        turn_adjustment = kp * error * max_turn_diff
        
        left_speed = self.base_speed - turn_adjustment
        right_speed = self.base_speed + turn_adjustment
        
        # Clamp speeds
        left_speed = max(self.base_speed * 0.2, min(self.MAX_SPEED, left_speed))
        right_speed = max(self.base_speed * 0.2, min(self.MAX_SPEED, right_speed))
        
        return left_speed, right_speed
    
    def execute_turn(self, target_heading):
        """Execute turn to reach target heading with proportional control."""
        current_heading = self.normalize_angle(self.phi)
        target_heading = self.normalize_angle(target_heading)
        
        heading_error = target_heading - current_heading
        heading_error = self.normalize_angle(heading_error)
        
        # Define a tolerance for turn completion
        turn_tolerance = 0.05 # Smaller tolerance for more precise turns (approx 2.8 degrees)
        
        if abs(heading_error) < turn_tolerance:  # Turn completed
            return 0, 0, True
        
        # Proportional control for turning speed
        turn_kp = 1.2 # Adjusted: Reduced from 1.8 for less overshoot
        
        # Calculate base turn speed proportional to the error
        # Clamp turn_speed to a reasonable range
        raw_turn_speed = turn_kp * heading_error
        
        # Max turn speed
        max_abs_turn_speed = self.MAX_SPEED * 0.7 # Allow higher turn speed for faster rotation
        # Min turn speed to ensure it actually moves when error is small
        min_abs_turn_speed = self.base_speed * 0.1 
        
        # Apply limits
        turn_speed = max(min_abs_turn_speed, min(max_abs_turn_speed, abs(raw_turn_speed)))
        
        # Determine direction
        if heading_error > 0:  # Turn left (positive error)
            left_speed = -turn_speed
            right_speed = turn_speed
        else:  # Turn right (negative error)
            left_speed = turn_speed
            right_speed = -turn_speed
            
        return left_speed, right_speed, False
    
    def process_sensor_data(self, sensor_data):
        """Process incoming sensor data and update robot state"""
        gs_values = sensor_data.get('gs', [500, 500, 500])
        ps_values = sensor_data.get('ps', [0] * 8)
        self.phi = sensor_data.get('h', self.phi)
        
        # Update LED indicators based on ground sensors
        self.led_blue.value(gs_values[2] < self.line_threshold)   # Left
        self.led_green.value(gs_values[1] < self.line_threshold)  # Center 
        self.led_red.value(gs_values[0] < self.line_threshold)    # Right
        
        # Obstacle detection - only front sensors
        obstacle_detected = ps_values[0] > 85 or ps_values[7] > 85
        
        return gs_values, ps_values, obstacle_detected
    
    def state_machine(self, gs_values, ps_values, obstacle_detected):
        """Main robot state machine"""
        left_speed, right_speed = 0, 0
        
        # --- Debug variables to send to Webots ---
        int_det_debug = False
        int_counter_debug = self.intersection_counter
        # ----------------------------------------
        
        # Handle obstacle avoidance (highest priority)
        if obstacle_detected:
            if self.current_state != 'obstacle_avoidance' and self.current_state != 'turning_180':
                self.current_state = 'turning_180'
                self.turn_target_heading = self.normalize_angle(self.phi + math.pi) # Target 180 degree turn
                
                # Block the node the robot was trying to reach for future replanning
                if self.next_node_index < len(self.path):
                    self.blocked_nodes_for_replan.add(self.path[self.next_node_index])
                left_speed, right_speed = 0, 0 # Stop while transitioning
            # If already in turning_180, continue the turn
            elif self.current_state == 'turning_180':
                left_speed, right_speed, turn_complete = self.execute_turn(self.turn_target_heading)
                if turn_complete:
                    # NIEUWE LOGICA: Controleer of het knooppunt dat hij probeerde te bereiken het doel was
                    if self.next_node_index < len(self.path) and self.path[self.next_node_index] == self.goal_node:
                        self.current_node = self.goal_node # Stel current_node handmatig in op het doel
                        self.current_state = 'stopping' # Missie voltooid
                        self.blocked_nodes_for_replan.clear() # Wis geblokkeerde knooppunten
                        return self.current_state, left_speed, right_speed, gs_values, int_det_debug, int_counter_debug
                    
                    node_robot_was_trying_to_reach = None
                    if self.next_node_index < len(self.path):
                        node_robot_was_trying_to_reach = self.path[self.next_node_index]
                    else:
                        # Fallback if next_node_index is out of bounds (e.g., at goal)
                        node_robot_was_trying_to_reach = self.current_node 

                    if node_robot_was_trying_to_reach:
                        # Find a path from the last confirmed node (where robot physically is)
                        # to the goal, avoiding the node it just couldn't reach.
                        temp_new_path = self.dijkstra_pathfind(self.current_node, self.goal_node, self.blocked_nodes_for_replan)

                        if temp_new_path:
                            # Construct the user-requested path: [node_robot_was_trying_to_reach, current_node, ...]
                            # This means prepending the 'node_robot_was_trying_to_reach' to the path found from 'current_node'.
                            final_new_path = [node_robot_was_trying_to_reach] + temp_new_path
                            
                            self.path = final_new_path
                            self.current_node = final_new_path[0] # Set current_node to the first node of the new path (e.g., C3)
                            self.next_node_index = 1 # Next node in path is the second one (e.g., C2)
                            self.current_state = 'line_following' # Resume line following on new path
                            self.blocked_nodes_for_replan.clear() # Obstacle handled, clear blocked nodes
                        else:
                            self.current_state = 'stopping'
                    else:
                        self.current_state = 'stopping'
        
        # If obstacle is cleared while in obstacle_avoidance (and not yet turning 180)
        elif self.current_state == 'obstacle_avoidance' and not obstacle_detected:
            self.current_state = 'line_following'
            self.blocked_nodes_for_replan.clear() # Clear blocked nodes if obstacle is gone (optional, based on desired behavior)

        # Check if mission is complete
        if self.current_node == self.goal_node:
            self.current_state = 'stopping'
            return 'mission_complete', 0, 0, gs_values, int_det_debug, int_counter_debug # Return debug info
        
        # Main state machine transitions (excluding obstacle detection which is high priority)
        if self.current_state == 'line_following':
            if self.detect_intersection(gs_values):
                int_det_debug = True # Set debug flag when intersection is detected
                self.intersection_counter += 1
                self.no_intersection_counter = 0 # Reset no-intersection counter when intersection is detected
                
                # Only process this intersection if it hasn't been processed yet
                if self.intersection_counter >= 3 and not self.intersection_processed:
                    self.current_state = 'at_intersection'
                    
                    # Update current_node to the node it just arrived at.
                    if self.next_node_index < len(self.path):
                        self.current_node = self.path[self.next_node_index] 
                        self.last_confirmed_node = self.current_node
                        self.intersection_processed = True # Mark this intersection as processed
                    else:
                        # No more nodes in path.
                        self.current_state = 'stopping'
                        self.intersection_counter = 0 # Ensure counter is reset if stopping
                        return self.current_state, left_speed, right_speed, gs_values, int_det_debug, int_counter_debug
                    
                    # INCREASE THIS VALUE to make the robot drive further into the intersection
                    self.intersection_counter = 15 # Set delay cycles for at_intersection state (e.g., 8 -> 12)
                                                  # This 12 is now the drive-through duration
            else: # No intersection detected
                self.no_intersection_counter += 1
                # Only reset intersection_counter if no intersection detected for a few cycles
                if self.no_intersection_counter >= 2: # Debounce period for resetting
                    self.intersection_counter = 0 # Reset intersection detection counter
                    self.intersection_processed = False # Reset flag when no intersection is detected for a while
            
            left_speed, right_speed = self.line_following_control(gs_values)
        
        elif self.current_state == 'at_intersection':
            left_speed, right_speed = self.base_speed * 0.4, self.base_speed * 0.4 # Drive slowly

            if self.intersection_counter > 0:
                # Continue driving through the intersection for a few more cycles
                self.intersection_counter -= 1
            else: # self.intersection_counter == 0, meaning drive-through is complete
                # Now determine the next action and transition out of 'at_intersection'
                
                # Increment next_node_index *only after* the drive-through period is over
                # and before determining the path to the *next* segment.
                self.next_node_index += 1 # This points to the actual next target node in the path

                if self.next_node_index >= len(self.path):
                    self.current_state = 'stopping'
                else:
                    next_target = self.path[self.next_node_index]
                    direction = self.get_direction_to_next_node(self.current_node, next_target)
                    
                    if direction:
                        action, angle_diff = self.direction_to_action(direction)
                        if action == 'forward':
                            self.current_state = 'line_following'
                        else:
                            self.current_state = 'turning'
                            self.turn_target_heading = self.normalize_angle(self.phi + angle_diff)
                    else:
                        self.current_state = 'stopping'
                
                # Reset intersection_counter to 0 (already is, but for clarity)
                self.intersection_counter = 0
        
        elif self.current_state == 'turning':
            left_speed, right_speed, turn_complete = self.execute_turn(self.turn_target_heading)
            
            if turn_complete:
                self.current_state = 'post_turn_search'
                self.line_search_counter = 0
        
        elif self.current_state == 'post_turn_search':
            error = self.calculate_line_error(gs_values)
            
            if error is not None:
                self.current_state = 'line_following'
                left_speed, right_speed = self.line_following_control(gs_values)
            else:
                self.line_search_counter += 1
                if self.line_search_counter < 30:
                    # Search pattern
                    search_speed = self.base_speed * 0.3
                    if (self.line_search_counter // 5) % 2 == 0:
                        left_speed, right_speed = search_speed * 0.8, search_speed * 1.2
                    else:
                        left_speed, right_speed = search_speed * 1.2, search_speed * 0.8
                else:
                    # After extensive search, revert to line following
                    self.current_state = 'line_following'
                    left_speed, right_speed = self.base_speed * 0.5, self.base_speed * 0.5
        
        elif self.current_state == 'obstacle_avoidance':
            left_speed, right_speed = 0, 0 # Stop the robot
            # This state is now primarily for pausing before turning 180 or if obstacle clears without 180 turn
            # If the obstacle is no longer detected, transition back to line following
            if not obstacle_detected:
                self.current_state = 'line_following'
                self.blocked_nodes_for_replan.clear() # Clear blocked nodes if obstacle is gone
        
        elif self.current_state == 'turning_180':
            # This state is handled at the top due to its high priority when an obstacle is present
            # If we reach here, it means the obstacle *might* have cleared, but the turn needs to complete.
            # However, the logic for replanning is now after the turn completes.
            # So, we just ensure it keeps turning if it's in this state.
            left_speed, right_speed, turn_complete = self.execute_turn(self.turn_target_heading)
            if turn_complete:
                # NIEUWE LOGICA: Deze controle wordt hier gedupliceerd, omdat de obstakeldetectie de status kan instellen
                # en vervolgens de draai onmiddellijk in dezelfde of de volgende cyclus kan voltooien.
                # Het is veiliger om het hier ook te hebben.
                if self.next_node_index < len(self.path) and self.path[self.next_node_index] == self.goal_node:
                    self.current_node = self.goal_node # Stel current_node handmatig in op het doel
                    self.current_state = 'stopping' # Missie voltooid
                    self.blocked_nodes_for_replan.clear() # Wis geblokkeerde knooppunten
                    return self.current_state, left_speed, right_speed, gs_values, int_det_debug, int_counter_debug

                node_robot_was_trying_to_reach = None
                if self.next_node_index < len(self.path):
                    node_robot_was_trying_to_reach = self.path[self.next_node_index]
                else:
                    # Fallback if next_node_index is out of bounds (e.g., at goal)
                    node_robot_was_trying_to_reach = self.current_node 

                if node_robot_was_trying_to_reach:
                    # Find a path from the last confirmed node (where robot physically is)
                    # to the goal, avoiding the node it just couldn't reach.
                    temp_new_path = self.dijkstra_pathfind(self.current_node, self.goal_node, self.blocked_nodes_for_replan)

                    if temp_new_path:
                        # Construct the user-requested path: [node_robot_was_trying_to_reach, current_node, ...]
                        # This means prepending the 'node_robot_was_trying_to_reach' to the path found from 'current_node'.
                        final_new_path = [node_robot_was_trying_to_reach] + temp_new_path
                        
                        self.path = final_new_path
                        self.current_node = final_new_path[0] # Set current_node to the first node of the new path (e.g., C3)
                        self.next_node_index = 1 # Next node in path is the second one (e.g., C2)
                        self.current_state = 'line_following' # Resume line following on new path
                        self.blocked_nodes_for_replan.clear() # Obstacle handled, clear blocked nodes
                    else:
                        self.current_state = 'stopping'
                else:
                    self.current_state = 'stopping'
        
        elif self.current_state == 'stopping':
            left_speed, right_speed = 0, 0
        
        # --- Return updated debug values ---
        return self.current_state, left_speed, right_speed, gs_values, int_det_debug, int_counter_debug

    def run(self):
        """Main control loop"""
        # Wait for button and setup communication
        if not self.wait_for_button_and_setup_uart():
            return
        
        # Calculate initial path
        self.path = self.dijkstra_pathfind(self.current_node, self.goal_node)
        if not self.path:
            return
        
        self.led_yellow.on()  # Indicate path planning complete
        
        while True:
            try:
                # Receive sensor data from Webots
                if self.uart.any():
                    line = self.uart.readline()
                    if line:
                        sensor_data = ujson.loads(line.decode('utf-8').strip())
                        
                        # Process sensor data
                        gs_values, ps_values, obstacle_detected = self.process_sensor_data(sensor_data)
                        
                        # Run state machine - now receives debug info
                        action, left_speed, right_speed, gs_debug, int_det_debug, int_counter_debug = self.state_machine(gs_values, ps_values, obstacle_detected)
                        
                        # Send command back to Webots
                        command = {
                            'action': action,
                            'left_speed': left_speed,
                            'right_speed': right_speed,
                            'current_node': self.current_node,
                            'path': self.path,
                            'gs_debug': gs_debug,
                            'int_det_debug': int_det_debug,
                            'int_counter_debug': int_counter_debug
                        }
                        
                        command_str = ujson.dumps(command) + '\n'
                        self.uart.write(command_str.encode('utf-8'))
                        
                        # Check for mission completion
                        if action == 'mission_complete':
                            self.led_board.on()
                            break
                
                sleep(0.02)  # 50Hz control loop
                
            except Exception as e:
                sleep(0.1)

# Main execution
def main():
    """Main function"""
    controller = PathPlannerESP32()
    try:
        controller.run()
    except KeyboardInterrupt:
        pass
    except Exception as e:
        pass

if __name__ == "__main__":
    main()