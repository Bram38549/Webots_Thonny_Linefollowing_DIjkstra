"""
ESP32 MicroPython Controller for Path Planning Robot
Implements Dijkstra's algorithm and robot control logic
Communicates with Webots via serial for Hardware-in-the-Loop simulation

Author: Generated for Robotics Assignment
Compatible with: MicroPython v1.25.0 on ESP32
Required: ujson module (built into most MicroPython versions)
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
        self.current_node = "A6"  # Starting position
        self.goal_node = "P8"     # Goal position
        self.path = []
        self.next_node_index = 1
        self.last_confirmed_node = "P1"
        
        # Robot state machine
        self.states = ['line_following', 'at_intersection', 'turning', 
                      'post_turn_search', 'obstacle_avoidance', 'stopping']
        self.current_state = 'line_following'
        
        # Control variables
        self.phi = -math.pi/2  # Robot heading (starts facing South: -π/2)
        self.intersection_counter = 0
        self.turn_target_heading = 0
        self.line_search_counter = 0
        
        # Timing
        self.last_update = ticks_ms()
        
        # Initialize grid map for pathfinding
        self._init_grid_map()
        
        print("ESP32 Path Planner initialized")
        print("Click left button to continue, then close Thonny and run Webots")
    
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
        print("UART communication established")
        return True
    
    def dijkstra_pathfind(self, start, goal, blocked_nodes=None):
        """
        Dijkstra's algorithm implementation for shortest path finding
        Returns list of nodes representing the path
        """
        if blocked_nodes is None:
            blocked_nodes = set()
        
        if start not in self.grid_map or goal not in self.grid_map:
            print(f"Invalid nodes: {start}, {goal}")
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
        if abs(angle_diff) < 0.1:  # ~5.7 degrees
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
        kp = 2.0
        max_turn_diff = self.MAX_SPEED * 0.4
        
        turn_adjustment = kp * error * max_turn_diff
        
        left_speed = self.base_speed - turn_adjustment
        right_speed = self.base_speed + turn_adjustment
        
        # Clamp speeds
        left_speed = max(self.base_speed * 0.2, min(self.MAX_SPEED, left_speed))
        right_speed = max(self.base_speed * 0.2, min(self.MAX_SPEED, right_speed))
        
        return left_speed, right_speed
    
    def execute_turn(self, target_heading):
        """Execute turn to reach target heading"""
        current_heading = self.normalize_angle(self.phi)
        target_heading = self.normalize_angle(target_heading)
        
        heading_error = target_heading - current_heading
        heading_error = self.normalize_angle(heading_error)
        
        if abs(heading_error) < 0.1:  # Turn completed
            return 0, 0, True
        
        # Proportional turn control
        turn_kp = 2.0
        turn_speed = self.MAX_SPEED * 0.5
        
        if heading_error > 0:  # Turn left
            return -turn_speed, turn_speed, False
        else:  # Turn right
            return turn_speed, -turn_speed, False
    
    def process_sensor_data(self, sensor_data):
        """Process incoming sensor data and update robot state"""
        gs_values = sensor_data.get('gs', [500, 500, 500])
        ps_values = sensor_data.get('ps', [0] * 8)
        self.phi = sensor_data.get('h', self.phi)
        
        # Update LED indicators based on ground sensors
        self.led_blue.value(gs_values[2] < self.line_threshold)   # Left
        self.led_green.value(gs_values[1] < self.line_threshold)  # Center  
        self.led_red.value(gs_values[0] < self.line_threshold)    # Right
        
        # Obstacle detection
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
        if obstacle_detected and self.current_state != 'obstacle_avoidance':
            print("Obstacle detected! Replanning...")
            self.current_state = 'obstacle_avoidance'
            # Block the current target node and replan
            blocked_nodes = {self.path[self.next_node_index]} if self.next_node_index < len(self.path) else set()
            new_path = self.dijkstra_pathfind(self.last_confirmed_node, self.goal_node, blocked_nodes)
            if new_path:
                self.path = new_path
                self.current_node = self.last_confirmed_node
                self.next_node_index = 1
                print(f"New path: {' -> '.join(self.path)}")
            self.current_state = 'line_following'
        
        # Check if mission is complete
        if self.current_node == self.goal_node:
            self.current_state = 'stopping'
            return 'mission_complete', 0, 0, gs_values, int_det_debug, int_counter_debug # Return debug info
        
        # Main state machine
        if self.current_state == 'line_following':
            if self.detect_intersection(gs_values):
                int_det_debug = True # Set debug flag when intersection is detected
                self.intersection_counter += 1
                # --- FIX: Reduced threshold for intersection detection ---
                if self.intersection_counter >= 4: # Changed from 8 to 4 for quicker detection
                    print(f"Intersection detected! Moving from {self.current_node} to {self.path[self.next_node_index]}")
                    self.current_state = 'at_intersection'
                    self.intersection_counter = 0 # Reset after transition
            else:
                self.intersection_counter = 0 # Reset if no intersection detected
            
            left_speed, right_speed = self.line_following_control(gs_values)
        
        elif self.current_state == 'at_intersection':
            # Drive through intersection
            left_speed, right_speed = self.base_speed * 0.4, self.base_speed * 0.4
            
            # Update logical position after driving through
            self.current_node = self.path[self.next_node_index] # This updates the node
            self.last_confirmed_node = self.current_node
            print(f"Robot now logically at node: {self.current_node}") # Added debug print for confirmation
            
            # Determine next action
            self.next_node_index += 1
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
                    # Fallback if no direction found, should ideally not happen with valid path
                    self.current_state = 'stopping'
        
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
            print("No path found!")
            return
        
        print(f"Path calculated: {' -> '.join(self.path)}")
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
                            'current_node': self.current_node,  # This will now update correctly
                            'path': self.path,                  # Path is also sent
                            'gs_debug': gs_debug,              # New debug field
                            'int_det_debug': int_det_debug,    # New debug field
                            'int_counter_debug': int_counter_debug # New debug field
                        }
                        
                        command_str = ujson.dumps(command) + '\n'
                        self.uart.write(command_str.encode('utf-8'))
                        
                        # Check for mission completion
                        if action == 'mission_complete':
                            print("Mission completed!")
                            self.led_board.on()
                            break
                
                sleep(0.02)  # 50Hz control loop
                
            except Exception as e:
                print(f"Error in main loop: {e}")
                sleep(0.1)

# Main execution
def main():
    """Main function"""
    controller = PathPlannerESP32()
    try:
        controller.run()
    except KeyboardInterrupt:
        print("Controller stopped by user")
    except Exception as e:
        print(f"Controller error: {e}")

if __name__ == "__main__":
    main()
