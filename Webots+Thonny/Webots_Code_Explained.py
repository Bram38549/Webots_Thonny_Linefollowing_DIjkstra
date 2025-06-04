# line_following_wifi_HIL.py
"""
Webots Hardware-in-the-Loop Controller for Path Planning Robot
Communicates with ESP32 via serial to send sensor data and receive movement commands

Author: Bram Smit and Wouter van den Bosch
Compatible with: Webots R2023a+, Python 3.10+
"""

from controller import Robot  # Imports the Robot class from the Webots controller module, essential for controlling the robot.
import serial  # Imports the serial library for serial communication, used to talk to the ESP32.
import json  # Imports the json library for encoding and decoding JSON data, used for structured communication.
import time  # Imports the time module for time-related functions, used for timestamps.
import matplotlib.pyplot as plt  # Imports matplotlib for plotting and visualization of the robot's path.
import networkx as nx # Used for easier graph visualization, helps in creating and displaying the map.

# Serial communication setup
SERIAL_PORT = 'COM8'  # Defines the serial port to be used for communication with the ESP32. Change this according to your system.
BAUDRATE = 115200  # Sets the baud rate for serial communication, must match the ESP32's baud rate.
TIMEOUT = 0.1         # Sets the timeout for reading from the serial port in seconds, preventing indefinite waits.

class WebotsHILController:
    def __init__(self):
        """Initialize robot and communication"""
        self.robot = Robot()  # Creates an instance of the Webots Robot, connecting to the simulated robot.
        self.timestep = int(self.robot.getBasicTimeStep())  # Gets the simulation's basic time step and converts it to an integer.

        # Robot parameters
        self.MAX_SPEED = 6.28  # Defines the maximum rotational speed for the robot's wheels.
        self.R = 0.0205  # Wheel radius [m], used for odometry calculations.
        self.D = 0.057   # Distance between wheels [m], used for odometry calculations.

        # Initialize devices
        self._init_sensors()  # Calls a private method to initialize the robot's sensors.
        self._init_motors()  # Calls a private method to initialize the robot's motors.
        self._init_encoders()  # Calls a private method to initialize the robot's wheel encoders.

        # Communication setup
        self._init_serial()  # Calls a private method to set up serial communication.

        # State variables
        self.current_command = {  # Initializes a dictionary to store the current command received from the ESP32.
            'action': 'stop',  # Default action for the robot.
            'left_speed': 0,  # Default left wheel speed.
            'right_speed': 0,  # Default right wheel speed.
            'current_node': 'Unknown',  # Stores the robot's current node on the map.
            'path': [],  # Stores the planned path as a list of nodes.
            'gs_debug': [0, 0, 0],  # Debugging variable for ground sensor values.
            'int_det_debug': False,  # Debugging variable for intersection detection.
            'int_counter_debug': 0  # Debugging variable for intersection counter.
        }
        self.phi = -1.5708  # Robot heading (starts facing South: -π/2), used for orientation tracking.
        self.oldEncoderValues = [0, 0]  # Stores previous encoder values for odometry calculations.

        # --- Visualization setup ---
        self.node_positions = self._get_node_positions()  # Gets the (x,y) coordinates for nodes for plotting.
        self.grid_graph = self._create_grid_graph()  # Creates a NetworkX graph representation of the map for visualization.
        self.fig, self.ax = plt.subplots(figsize=(8, 8))  # Creates a matplotlib figure and axes for plotting.
        self._setup_plot()  # Calls a private method to set up the initial plot.
        # --- End Visualization setup ---

        print("Webots HIL Controller initialized")  # Prints a confirmation message.
        print(f"Serial port: {SERIAL_PORT}, Baudrate: {BAUDRATE}")  # Prints the serial port and baud rate being used.

    def _get_node_positions(self):
        """Define (x, y) coordinates for each node on the map for plotting."""
        # These coordinates should roughly correspond to the layout in your Webots world
        # You'll need to adjust these based on your specific map layout for accurate visualization.
        # Example:
        return {  # Returns a dictionary mapping node names to their (x, y) coordinates.
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
        G = nx.Graph()  # Creates an empty undirected graph using NetworkX.
        # Referencing the grid_map from A_algorthim.py to create edges for visualization
        # This needs to be a hardcoded representation of your map structure for visualization
        # Make sure that these match the _init_grid_map in A_algorthim.py
        edges = [  # Defines a list of tuples, where each tuple represents an edge between two nodes.
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
        G.add_edges_from(edges)  # Adds all defined edges to the graph.
        return G  # Returns the created graph.

    def _setup_plot(self):
        """Set up the initial matplotlib plot."""
        self.ax.set_title("Robot Path Planning Visualization")  # Sets the title of the plot.
        self.ax.set_xlabel("X-coordinate")  # Sets the label for the x-axis.
        self.ax.set_ylabel("Y-coordinate")  # Sets the label for the y-axis.
        self.ax.set_aspect('equal', adjustable='box')  # Ensures equal scaling for x and y axes.

        # Draw the static grid map
        nx.draw_networkx_nodes(self.grid_graph, self.node_positions, node_color='lightgray', node_size=700, ax=self.ax)  # Draws the nodes of the graph.
        nx.draw_networkx_edges(self.grid_graph, self.node_positions, edge_color='gray', ax=self.ax)  # Draws the edges of the graph.
        nx.draw_networkx_labels(self.grid_graph, self.node_positions, font_size=8, ax=self.ax)  # Draws labels for the nodes.

        self.path_line, = self.ax.plot([], [], 'o-', color='blue', linewidth=2, markersize=8, label='Planned Path')  # Initializes an empty line plot for the planned path.
        self.current_node_marker, = self.ax.plot([], [], 'o', color='red', markersize=12, label='Current Node')  # Initializes an empty marker for the current node.

        self.ax.legend()  # Displays the legend on the plot.
        plt.ion() # Turn on interactive mode, allowing plot updates without blocking.
        plt.show()  # Displays the plot.

    def _update_plot(self):
        """Update the plot with current robot state and path."""
        current_node_name = self.current_command.get('current_node', 'Unknown')  # Gets the current node name from the command.
        planned_path_nodes = self.current_command.get('path', [])  # Gets the planned path nodes from the command.

        # Update current node marker
        if current_node_name in self.node_positions:  # Checks if the current node exists in the defined positions.
            x, y = self.node_positions[current_node_name]  # Gets the (x, y) coordinates of the current node.
            self.current_node_marker.set_data([x], [y])  # Updates the data for the current node marker.
        else:
            self.current_node_marker.set_data([], []) # Hide if unknown, clears the marker if the node is unknown.

        # Update planned path line
        if planned_path_nodes:  # Checks if there are planned path nodes.
            path_x = [self.node_positions[node][0] for node in planned_path_nodes if node in self.node_positions]  # Extracts x-coordinates for the path.
            path_y = [self.node_positions[node][1] for node in planned_path_nodes if node in self.node_positions]  # Extracts y-coordinates for the path.
            self.path_line.set_data(path_x, path_y)  # Updates the data for the planned path line.
        else:
            self.path_line.set_data([], [])  # Clears the path line if no path is planned.

        self.fig.canvas.draw_idle()  # Redraws the canvas after changes, but only if necessary.
        self.fig.canvas.flush_events() # Processes GUI events, ensuring the plot updates immediately.

    def _init_sensors(self):
        """Initialize proximity and ground sensors"""
        self.ps = []  # Initializes an empty list for proximity sensors.
        for i in range(8):  # Loops 8 times for 8 proximity sensors.
            sensor = self.robot.getDevice(f'ps{i}')  # Gets a reference to the proximity sensor by its name.
            sensor.enable(self.timestep)  # Enables the sensor with the specified timestep.
            self.ps.append(sensor)  # Adds the sensor to the list.

        self.gs = []  # Initializes an empty list for ground sensors.
        for i in range(3):  # Loops 3 times for 3 ground sensors.
            sensor = self.robot.getDevice(f'gs{i}')  # Gets a reference to the ground sensor by its name.
            sensor.enable(self.timestep)  # Enables the sensor with the specified timestep.
            self.gs.append(sensor)  # Adds the sensor to the list.

    def _init_motors(self):
        """Initialize wheel motors"""
        self.leftMotor = self.robot.getDevice('left wheel motor')  # Gets a reference to the left wheel motor.
        self.rightMotor = self.robot.getDevice('right wheel motor')  # Gets a reference to the right wheel motor.
        self.leftMotor.setPosition(float('inf'))  # Sets the left motor to infinite position, allowing continuous rotation.
        self.rightMotor.setPosition(float('inf'))  # Sets the right motor to infinite position.
        self.leftMotor.setVelocity(0.0)  # Sets the initial velocity of the left motor to zero.
        self.rightMotor.setVelocity(0.0)  # Sets the initial velocity of the right motor to zero.

    def _init_encoders(self):
        """Initialize wheel encoders"""
        self.encoder = []  # Initializes an empty list for encoders.
        for name in ['left wheel sensor', 'right wheel sensor']:  # Iterates through the names of the wheel sensors.
            enc = self.robot.getDevice(name)  # Gets a reference to the encoder device.
            enc.enable(self.timestep)  # Enables the encoder with the specified timestep.
            self.encoder.append(enc)  # Adds the encoder to the list.

    def _init_serial(self):
        """Initialize serial communication"""
        try:
            self.ser = serial.Serial(port=SERIAL_PORT, baudrate=BAUDRATE, timeout=TIMEOUT)  # Attempts to establish a serial connection.
            self.ser.flushInput()  # Clears any pending input from the serial buffer.
            print("Serial communication established")  # Prints a success message.
        except Exception as e:
            print(f"Serial communication failed: {e}")  # Prints an error message if serial communication fails.
            print("Check cable connections and port settings, and ensure no other program is using the COM port.")  # Provides troubleshooting tips.
            raise  # Re-raises the exception to stop execution if serial fails.

    def get_sensor_data(self):
        """Get current sensor readings"""
        gs_values = [self.gs[i].getValue() for i in range(3)]  # Gets values from all ground sensors.
        ps_values = [self.ps[i].getValue() for i in range(8)]  # Gets values from all proximity sensors.
        encoder_values = [self.encoder[i].getValue() for i in range(2)]  # Gets values from both wheel encoders.

        delta_t = self.timestep / 1000.0  # Calculates the time difference in seconds.
        wl = (encoder_values[0] - self.oldEncoderValues[0]) / delta_t  # Calculates angular velocity of the left wheel.
        wr = (encoder_values[1] - self.oldEncoderValues[1]) / delta_t  # Calculates angular velocity of the right wheel.
        w = self.R / self.D * (wr - wl)  # Calculates the angular velocity of the robot.
        self.phi += w * delta_t  # Updates the robot's heading (phi) based on angular velocity.

        while self.phi > 3.14159:  # Normalizes the heading to be within -pi to pi.
            self.phi -= 2 * 3.14159
        while self.phi < -3.14159:  # Normalizes the heading to be within -pi to pi.
            self.phi += 2 * 3.14159

        self.oldEncoderValues = encoder_values.copy()  # Stores the current encoder values for the next iteration.

        return {  # Returns a dictionary containing all sensor data and heading.
            'ground_sensors': gs_values,
            'proximity_sensors': ps_values,
            'heading': self.phi,
            'timestamp': time.time()
        }

    def send_sensor_data(self, sensor_data):
        """Send sensor data to ESP32"""
        try:
            message = {  # Creates a dictionary with sensor data to be sent.
                'gs': [int(val) for val in sensor_data['ground_sensors']],  # Converts ground sensor values to integers.
                'ps': [int(val) for val in sensor_data['proximity_sensors']],  # Converts proximity sensor values to integers.
                'h': round(sensor_data['heading'], 4)  # Rounds heading to 4 decimal places.
            }
            json_str = json.dumps(message) + '\n'  # Converts the dictionary to a JSON string and appends a newline.
            self.ser.write(json_str.encode('utf-8'))  # Encodes the string to bytes and sends it over serial.
        except Exception as e:
            print(f"Error sending data: {e}")  # Prints an error message if sending data fails.

    def receive_command(self):
        """Receive movement command from ESP32"""
        line = ""  # Initializes an empty string to store the received line.
        try:
            if self.ser.in_waiting:  # Checks if there is data waiting in the serial buffer.
                raw_line = self.ser.readline()  # Reads a line from the serial port (up to newline).
                # print(f"RAW incoming (bytes): {raw_line!r}") # Uncomment for deeper debugging
                
                line = raw_line.decode('utf-8').strip()  # Decodes the bytes to a UTF-8 string and removes leading/trailing whitespace.
                
                if not line:  # Checks if the decoded line is empty.
                    print("Received empty/incomplete line from ESP32. Skipping JSON decode.")  # Prints a warning for empty lines.
                    return False  # Returns False indicating no valid command was received.
                
                command = json.loads(line)  # Parses the JSON string into a Python dictionary.
                
                self.current_command['action'] = command.get('action', 'stop')  # Updates the 'action' in current_command.
                self.current_command['left_speed'] = command.get('left_speed', 0)  # Updates the 'left_speed'.
                self.current_command['right_speed'] = command.get('right_speed', 0)  # Updates the 'right_speed'.
                self.current_command['current_node'] = command.get('current_node', 'Unknown')  # Updates the 'current_node'.
                self.current_command['path'] = command.get('path', [])  # Updates the 'path'.
                self.current_command['gs_debug'] = command.get('gs_debug', [0,0,0])  # Updates 'gs_debug'.
                self.current_command['int_det_debug'] = command.get('int_det_debug', False)  # Updates 'int_det_debug'.
                self.current_command['int_counter_debug'] = command.get('int_counter_debug', 0)  # Updates 'int_counter_debug'.
                
                return True  # Returns True indicating a command was successfully received and processed.
                
        except json.JSONDecodeError as e:  # Catches errors specific to JSON decoding.
            print(f"JSON Decode Error: {e} - Problematische regel (gedecodeerd): '{line}' (lengte: {len(line)})")  # Prints a detailed error message.
            self.ser.flushInput()  # Flushes the input buffer to clear bad data.
            return False  # Returns False due to a JSON decoding error.
        except Exception as e:  # Catches any other general exceptions during command reception.
            print(f"Algemene fout bij ontvangen commando: {e}")  # Prints a general error message.
            return False  # Returns False due to a general error.
        return False  # Default return for cases where no data was waiting.

    def execute_command(self):
        """Execute the current movement command"""
        left_speed = self.current_command.get('left_speed', 0)  # Gets the left speed from the current command.
        right_speed = self.current_command.get('right_speed', 0)  # Gets the right speed.

        left_speed = max(-self.MAX_SPEED, min(self.MAX_SPEED, left_speed))  # Clamps the left speed to MAX_SPEED.
        right_speed = max(-self.MAX_SPEED, min(self.MAX_SPEED, right_speed))  # Clamps the right speed to MAX_SPEED.

        self.leftMotor.setVelocity(left_speed)  # Sets the velocity of the left motor.
        self.rightMotor.setVelocity(right_speed)  # Sets the velocity of the right motor.

    def run(self):
        """Main control loop"""
        print("Starting HIL simulation...")  # Informs the user that the simulation is starting.
        print("Waiting for ESP32 to be ready...")  # Prompts the user to ensure ESP32 is ready.

        step_counter = 0  # Initializes a counter for simulation steps.

        while self.robot.step(self.timestep) != -1:  # Main simulation loop, continues as long as Webots is running.
            step_counter += 1  # Increments the step counter.

            sensor_data = self.get_sensor_data()  # Retrieves current sensor data.
            self.send_sensor_data(sensor_data)  # Sends the sensor data to the ESP32.
            received_new_command = self.receive_command()  # Attempts to receive a new command from the ESP32.

            # Update plot only when a new command is received to avoid unnecessary redraws
            if received_new_command:  # Checks if a new command was successfully received.
                self._update_plot()  # Updates the visualization plot.

            # Print command info every 50 steps
            if step_counter % 50 == 0:  # Checks if the current step is a multiple of 50.
                action = self.current_command.get('action', 'stop')  # Gets the current action.
                current_node = self.current_command.get('current_node', 'N/A')  # Gets the current node.
                path = self.current_command.get('path', [])  # Gets the planned path.
                gs_debug = self.current_command.get('gs_debug', [0,0,0])  # Gets ground sensor debug info.
                int_det_debug = self.current_command.get('int_det_debug', False)  # Gets intersection detection debug info.
                int_counter_debug = self.current_command.get('int_counter_debug', 0)  # Gets intersection counter debug info.

                print(f"Step {step_counter}: Action={action}, "  # Prints detailed information about the robot's state.
                      f"Current Node: {current_node}, "
                      f"Path: {' -> '.join(path)}, "
                      f"Speeds=({self.current_command.get('left_speed', 0):.2f}, "
                      f"{self.current_command.get('right_speed', 0):.2f})\n"
                      f"  DEBUG: GS={gs_debug}, Intersection Detected={int_det_debug}, Int Counter={int_counter_debug}")

            self.execute_command()  # Executes the received movement command.

            if self.current_command.get('action') == 'mission_complete':  # Checks if the mission is reported as complete.
                print("Mission completed by ESP32!")  # Prints a mission complete message.
                # Final update to show the robot at the goal
                self._update_plot()  # Performs a final plot update.
                plt.ioff() # Turn off interactive mode, freezing the plot.
                plt.show() # Keep the plot open after simulation ends.
                break  # Exits the simulation loop.

        self.ser.close()  # Closes the serial port when the simulation ends.
        print("HIL simulation ended")  # Informs the user that the simulation has ended.

def main():
    """Main function"""
    controller = WebotsHILController()  # Creates an instance of the WebotsHILController.
    try:
        controller.run()  # Starts the main control loop.
    except KeyboardInterrupt:  # Catches a KeyboardInterrupt (e.g., Ctrl+C).
        print("Simulation interrupted by user")  # Prints a message if interrupted by the user.
    except Exception as e:  # Catches any other exceptions during simulation.
        print(f"Simulation error: {e}")  # Prints an error message for general simulation errors.
    finally:
        if hasattr(controller, 'ser') and controller.ser.is_open:  # Checks if the serial port object exists and is open.
            controller.ser.close()  # Closes the serial port to ensure resources are released.
        plt.close('all') # Close all matplotlib figures to clean up.

if __name__ == "__main__":
    main()  # Calls the main function when the script is executed.