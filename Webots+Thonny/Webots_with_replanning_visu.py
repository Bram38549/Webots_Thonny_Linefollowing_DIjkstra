# line_following_wifi_HIL.py
"""
Webots Hardware-in-the-Loop Controller for Path Planning Robot
Communicates with ESP32 via serial to send sensor data and receive movement commands

Author: Bram Smit
Compatible with: Webots R2023a+, Python 3.10+
"""

from controller import Robot
import serial
import json
import time
import matplotlib.pyplot as plt
import networkx as nx # Used for easier graph visualization

# Serial communication setup
SERIAL_PORT = 'COM8'  # Change according to your system
BAUDRATE = 115200
TIMEOUT = 0.1         # Timeout for reading serial port in seconds

class WebotsHILController:
    def __init__(self):
        """Initialize robot and communication"""
        self.robot = Robot()
        self.timestep = int(self.robot.getBasicTimeStep())
        
        # Robot parameters
        self.MAX_SPEED = 6.28
        self.R = 0.0205  # Wheel radius [m]
        self.D = 0.057   # Distance between wheels [m]
        
        # Initialize devices
        self._init_sensors()
        self._init_motors()
        self._init_encoders()
        
        # Communication setup
        self._init_serial()
        
        # State variables
        self.current_command = {
            'action': 'stop', 
            'left_speed': 0, 
            'right_speed': 0, 
            'current_node': 'Unknown', 
            'path': [],
            'gs_debug': [0, 0, 0], 
            'int_det_debug': False,
            'int_counter_debug': 0
        }
        self.phi = -1.5708  # Robot heading (starts facing South: -π/2)
        self.oldEncoderValues = [0, 0]
        
        # --- Visualization setup ---
        self.node_positions = self._get_node_positions()
        self.grid_graph = self._create_grid_graph()
        self.fig, self.ax = plt.subplots(figsize=(8, 8))
        self._setup_plot()
        # --- End Visualization setup ---
        
        print("Webots HIL Controller initialized")
        print(f"Serial port: {SERIAL_PORT}, Baudrate: {BAUDRATE}")
    
    def _get_node_positions(self):
        """Define (x, y) coordinates for each node on the map for plotting."""
        # These coordinates should roughly correspond to the layout in your Webots world
        # You'll need to adjust these based on your specific map layout for accurate visualization.
        # Example:
        return {
            "P1": (0, 6), "P2": (1, 6), "P3": (2, 6), "P4": (3, 6),
            "P5": (8, 0), "P6": (9, 0), "P7": (10, 0), "P8": (11, 0),

            "A1": (0, 5), "A2": (1, 5), "A3": (2, 5), "A4": (3, 5), "A5": (5.5, 5), "A6": (11, 5),
            "B1": (5.5, 4), "B2": (11, 4),

            "C1": (0, 3), "C2": (5.5, 3), "C3": (11, 3), # Corrected based on A_algorthim.py C3 link to E6
            
            "D1": (0, 2), "D2": (5.5, 2),

            "E1": (0, 1), "E2": (5.5, 1), "E3": (8, 1), "E4": (9, 1), "E5": (10, 1), "E6": (11, 1),
        }
    
    def _create_grid_graph(self):
        """Create a networkx graph representation of your grid_map."""
        G = nx.Graph()
        # Referencing the grid_map from A_algorthim.py to create edges for visualization
        # This needs to be a hardcoded representation of your map structure for visualization
        # if you don't want to parse A_algorthim.py's content directly.
        # Ensure these match the _init_grid_map in A_algorthim.py
        edges = [
            ("P1", "A1"), ("P2", "A2"), ("P3", "A3"), ("P4", "A4"),
            ("P5", "E3"), ("P6", "E4"), ("P7", "E5"), ("P8", "E6"),
            
            ("A1", "A2"), ("A2", "A3"), ("A3", "A4"), ("A4", "A5"), ("A5", "A6"),
            ("A1", "C1"), ("A5", "B1"), ("A6", "B2"),
            
            ("B1", "B2"),
            ("B1", "C2"), ("B2", "C3"),
            
            ("C1", "C2"), ("C2", "C3"),
            ("C1", "D1"), ("C2", "D2"), ("C3", "E6"), # C3 now connects to E6
            
            ("D1", "D2"),
            ("D1", "E1"), ("D2", "E2"),
            
            ("E1", "E2"), ("E2", "E3"), ("E3", "E4"), ("E4", "E5"), ("E5", "E6"),
        ]
        G.add_edges_from(edges)
        return G
    
    def _setup_plot(self):
        """Set up the initial matplotlib plot."""
        self.ax.set_title("Robot Path Planning Visualization")
        self.ax.set_xlabel("X-coordinate")
        self.ax.set_ylabel("Y-coordinate")
        self.ax.set_aspect('equal', adjustable='box')
        
        # Draw the static grid map
        nx.draw_networkx_nodes(self.grid_graph, self.node_positions, node_color='lightgray', node_size=700, ax=self.ax)
        nx.draw_networkx_edges(self.grid_graph, self.node_positions, edge_color='gray', ax=self.ax)
        nx.draw_networkx_labels(self.grid_graph, self.node_positions, font_size=8, ax=self.ax)
        
        self.path_line, = self.ax.plot([], [], 'o-', color='blue', linewidth=2, markersize=8, label='Planned Path')
        self.current_node_marker, = self.ax.plot([], [], 'o', color='red', markersize=12, label='Current Node')
        
        self.ax.legend()
        plt.ion() # Turn on interactive mode
        plt.show()
    
    def _update_plot(self):
        """Update the plot with current robot state and path."""
        current_node_name = self.current_command.get('current_node', 'Unknown')
        planned_path_nodes = self.current_command.get('path', [])
        
        # Update current node marker
        if current_node_name in self.node_positions:
            x, y = self.node_positions[current_node_name]
            self.current_node_marker.set_data([x], [y])
        else:
            self.current_node_marker.set_data([], []) # Hide if unknown
            
        # Update planned path line
        if planned_path_nodes:
            path_x = [self.node_positions[node][0] for node in planned_path_nodes if node in self.node_positions]
            path_y = [self.node_positions[node][1] for node in planned_path_nodes if node in self.node_positions]
            self.path_line.set_data(path_x, path_y)
        else:
            self.path_line.set_data([], [])

        self.fig.canvas.draw_idle()
        self.fig.canvas.flush_events() # Process events
    
    def _init_sensors(self):
        """Initialize proximity and ground sensors"""
        self.ps = []
        for i in range(8):
            sensor = self.robot.getDevice(f'ps{i}')
            sensor.enable(self.timestep)
            self.ps.append(sensor)
        
        self.gs = []
        for i in range(3):
            sensor = self.robot.getDevice(f'gs{i}')
            sensor.enable(self.timestep)
            self.gs.append(sensor)
    
    def _init_motors(self):
        """Initialize wheel motors"""
        self.leftMotor = self.robot.getDevice('left wheel motor')
        self.rightMotor = self.robot.getDevice('right wheel motor')
        self.leftMotor.setPosition(float('inf'))
        self.rightMotor.setPosition(float('inf'))
        self.leftMotor.setVelocity(0.0)
        self.rightMotor.setVelocity(0.0)
    
    def _init_encoders(self):
        """Initialize wheel encoders"""
        self.encoder = []
        for name in ['left wheel sensor', 'right wheel sensor']:
            enc = self.robot.getDevice(name)
            enc.enable(self.timestep)
            self.encoder.append(enc)
    
    def _init_serial(self):
        """Initialize serial communication"""
        try:
            self.ser = serial.Serial(port=SERIAL_PORT, baudrate=BAUDRATE, timeout=TIMEOUT)
            self.ser.flushInput()
            print("Serial communication established")
        except Exception as e:
            print(f"Serial communication failed: {e}")
            print("Check cable connections and port settings, and ensure no other program is using the COM port.")
            raise
    
    def get_sensor_data(self):
        """Get current sensor readings"""
        gs_values = [self.gs[i].getValue() for i in range(3)]
        ps_values = [self.ps[i].getValue() for i in range(8)]
        encoder_values = [self.encoder[i].getValue() for i in range(2)]
        
        delta_t = self.timestep / 1000.0
        wl = (encoder_values[0] - self.oldEncoderValues[0]) / delta_t
        wr = (encoder_values[1] - self.oldEncoderValues[1]) / delta_t
        w = self.R / self.D * (wr - wl)
        self.phi += w * delta_t
        
        while self.phi > 3.14159:
            self.phi -= 2 * 3.14159
        while self.phi < -3.14159:
            self.phi += 2 * 3.14159
        
        self.oldEncoderValues = encoder_values.copy()
        
        return {
            'ground_sensors': gs_values,
            'proximity_sensors': ps_values,
            'heading': self.phi,
            'timestamp': time.time()
        }
    
    def send_sensor_data(self, sensor_data):
        """Send sensor data to ESP32"""
        try:
            message = {
                'gs': [int(val) for val in sensor_data['ground_sensors']],
                'ps': [int(val) for val in sensor_data['proximity_sensors']],
                'h': round(sensor_data['heading'], 4)
            }
            json_str = json.dumps(message) + '\n'
            self.ser.write(json_str.encode('utf-8'))
        except Exception as e:
            print(f"Error sending data: {e}")
    
    def receive_command(self):
        """Receive movement command from ESP32"""
        line = ""
        try:
            if self.ser.in_waiting:
                raw_line = self.ser.readline() 
                # print(f"RAW incoming (bytes): {raw_line!r}") # Uncomment for deeper debugging
                
                line = raw_line.decode('utf-8').strip()
                
                if not line:
                    print("Received empty/incomplete line from ESP32. Skipping JSON decode.")
                    return False
                
                command = json.loads(line)
                
                self.current_command['action'] = command.get('action', 'stop')
                self.current_command['left_speed'] = command.get('left_speed', 0)
                self.current_command['right_speed'] = command.get('right_speed', 0)
                self.current_command['current_node'] = command.get('current_node', 'Unknown')
                self.current_command['path'] = command.get('path', [])
                self.current_command['gs_debug'] = command.get('gs_debug', [0,0,0])
                self.current_command['int_det_debug'] = command.get('int_det_debug', False)
                self.current_command['int_counter_debug'] = command.get('int_counter_debug', 0)
                
                return True
                
        except json.JSONDecodeError as e:
            print(f"JSON Decode Error: {e} - Problematische regel (gedecodeerd): '{line}' (lengte: {len(line)})")
            self.ser.flushInput()
            return False
        except Exception as e:
            print(f"Algemene fout bij ontvangen commando: {e}")
            return False
        return False
    
    def execute_command(self):
        """Execute the current movement command"""
        left_speed = self.current_command.get('left_speed', 0)
        right_speed = self.current_command.get('right_speed', 0)
        
        left_speed = max(-self.MAX_SPEED, min(self.MAX_SPEED, left_speed))
        right_speed = max(-self.MAX_SPEED, min(self.MAX_SPEED, right_speed))
        
        self.leftMotor.setVelocity(left_speed)
        self.rightMotor.setVelocity(right_speed)
    
    def run(self):
        """Main control loop"""
        print("Starting HIL simulation...")
        print("Waiting for ESP32 to be ready...")
        
        step_counter = 0
        
        while self.robot.step(self.timestep) != -1:
            step_counter += 1
            
            sensor_data = self.get_sensor_data()
            self.send_sensor_data(sensor_data)
            received_new_command = self.receive_command() 
            
            # Update plot only when a new command is received to avoid unnecessary redraws
            if received_new_command:
                self._update_plot()
            
            # Print command info every 50 steps
            if step_counter % 50 == 0:
                action = self.current_command.get('action', 'stop')
                current_node = self.current_command.get('current_node', 'N/A')
                path = self.current_command.get('path', [])
                gs_debug = self.current_command.get('gs_debug', [0,0,0])
                int_det_debug = self.current_command.get('int_det_debug', False)
                int_counter_debug = self.current_command.get('int_counter_debug', 0)
                
                print(f"Step {step_counter}: Action={action}, "
                      f"Current Node: {current_node}, "
                      f"Path: {' -> '.join(path)}, "
                      f"Speeds=({self.current_command.get('left_speed', 0):.2f}, "
                      f"{self.current_command.get('right_speed', 0):.2f})\n"
                      f"  DEBUG: GS={gs_debug}, Intersection Detected={int_det_debug}, Int Counter={int_counter_debug}")
            
            self.execute_command()
            
            if self.current_command.get('action') == 'mission_complete':
                print("Mission completed by ESP32!")
                # Final update to show the robot at the goal
                self._update_plot() 
                plt.ioff() # Turn off interactive mode
                plt.show() # Keep the plot open after simulation ends
                break
        
        self.ser.close()
        print("HIL simulation ended")

def main():
    """Main function"""
    controller = WebotsHILController()
    try:
        controller.run()
    except KeyboardInterrupt:
        print("Simulation interrupted by user")
    except Exception as e:
        print(f"Simulation error: {e}")
    finally:
        if hasattr(controller, 'ser') and controller.ser.is_open:
            controller.ser.close()
        plt.close('all') # Close all matplotlib figures

if __name__ == "__main__":
    main()