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

# Serial communication setup
SERIAL_PORT = 'COM3'  # Change according to your system
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
        # Belangrijk: Initialiseer met alle verwachte velden, inclusief nieuwe debug velden
        self.current_command = {
            'action': 'stop', 
            'left_speed': 0, 
            'right_speed': 0, 
            'current_node': 'Unknown', 
            'path': [],
            'gs_debug': [0, 0, 0], # Nieuwe debug velden
            'int_det_debug': False,
            'int_counter_debug': 0
        }
        self.phi = -1.5708  # Robot heading (starts facing South: -π/2)
        self.oldEncoderValues = [0, 0]
        
        print("Webots HIL Controller initialized")
        print(f"Serial port: {SERIAL_PORT}, Baudrate: {BAUDRATE}")
    
    def _init_sensors(self):
        """Initialize proximity and ground sensors"""
        # Proximity sensors
        self.ps = []
        for i in range(8):
            sensor = self.robot.getDevice(f'ps{i}')
            sensor.enable(self.timestep)
            self.ps.append(sensor)
        
        # Ground sensors
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
            # Clear any stale data in the input buffer
            self.ser.flushInput()
            print("Serial communication established")
        except Exception as e:
            print(f"Serial communication failed: {e}")
            print("Check cable connections and port settings, and ensure no other program is using the COM port.")
            raise
    
    def get_sensor_data(self):
        """Get current sensor readings"""
        # Ground sensor values
        gs_values = [self.gs[i].getValue() for i in range(3)]
        
        # Proximity sensor values
        ps_values = [self.ps[i].getValue() for i in range(8)]
        
        # Encoder values for odometry
        encoder_values = [self.encoder[i].getValue() for i in range(2)]
        
        # Calculate angular velocity and update heading
        delta_t = self.timestep / 1000.0
        wl = (encoder_values[0] - self.oldEncoderValues[0]) / delta_t
        wr = (encoder_values[1] - self.oldEncoderValues[1]) / delta_t
        w = self.R / self.D * (wr - wl)  # Angular velocity
        self.phi += w * delta_t
        
        # Normalize angle to [-π, π]
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
            # Create compact message
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
        line = "" # Initialize line to an empty string for error reporting
        try:
            if self.ser.in_waiting:
                raw_line = self.ser.readline() 
                
                # Debug: ALTIJD de ruwe inkomende data printen
                print(f"RAW incoming (bytes): {raw_line!r}")
                
                line = raw_line.decode('utf-8').strip()
                
                if not line:
                    print("Received empty/incomplete line from ESP32. Skipping JSON decode.")
                    return False
                
                command = json.loads(line)
                
                # Update current_command dictionary. Use .get() for new debug fields for robustness
                self.current_command['action'] = command.get('action', 'stop')
                self.current_command['left_speed'] = command.get('left_speed', 0)
                self.current_command['right_speed'] = command.get('right_speed', 0)
                self.current_command['current_node'] = command.get('current_node', 'Unknown')
                self.current_command['path'] = command.get('path', [])
                self.current_command['gs_debug'] = command.get('gs_debug', [0,0,0]) # Nieuwe velden
                self.current_command['int_det_debug'] = command.get('int_det_debug', False)
                self.current_command['int_counter_debug'] = command.get('int_counter_debug', 0)
                
                return True
                
        except json.JSONDecodeError as e:
            print(f"JSON Decode Error: {e} - Problematische regel (gedecodeerd): '{line}' (lengte: {len(line)})")
            self.ser.flushInput() # Leeg de buffer
            return False
        except Exception as e:
            print(f"Algemene fout bij ontvangen commando: {e}")
            return False
        return False # Retourneer False als er niets wachtte of een fout optrad.
    
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
            self.receive_command() # Roep aan om self.current_command bij te werken
            
            # Print command info elke 50 stappen
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

if __name__ == "__main__":
    main()