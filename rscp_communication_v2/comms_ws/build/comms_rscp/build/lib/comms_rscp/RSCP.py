#!/usr/bin/env python3
"""
RSCP Serial Communication Node

This ROS2 node listens to a serial port for RSCP (Robot Serial Communication Protocol) 
messages, decodes them using COBS encoding and Protocol Buffers, then executes 
corresponding bash scripts based on the message type.

Protocol Flow:
1. Receive COBS-encoded data over serial (delimited by 0x00)
2. Decode COBS frame to get raw protobuf data
3. Parse protobuf RequestEnvelope message
4. Execute corresponding bash script based on request type
"""

# Use ros2 run comms_rscp listener_node --ros-args -p device_path:=/dev/pts/2

import os
import threading
import subprocess
import rclpy
from rclpy.node import Node
import serial
from cobs import cobs
import rscp_protobuf


def main(args=None):
    """Main entry point for the RSCP listener node."""
    rclpy.init(args=args)
    node = PortListenerNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


class PortListenerNode(Node):
    """
    ROS2 Node that listens to serial port for RSCP messages and executes scripts.
    
    This node handles:
    - Serial port communication with configurable device and baud rate
    - COBS decoding of incoming data frames
    - Protocol Buffer message parsing
    - Non-blocking execution of corresponding bash scripts
    """
    
    def __init__(self):
        super().__init__('port_listener')
        
        # ============================================================================
        # PARAMETER CONFIGURATION
        # ============================================================================
        self._setup_parameters()
        
        # ============================================================================
        # SCRIPT MAPPING CONFIGURATION  
        # ============================================================================
        self._setup_script_mappings()
        
        self.get_logger().info(f'RSCP Listener initialized on port: {self.device_path}')
        
        # ============================================================================
        # START SERIAL COMMUNICATION THREAD
        # ============================================================================
        # Start the serial listener in a separate daemon thread to avoid blocking ROS
        threading.Thread(target=self._serial_listener, daemon=True).start()
    
    def _setup_parameters(self):
        """Configure ROS parameters for serial communication."""
        # Declare configurable parameters with defaults
        self.declare_parameter('device_path', '/dev/ttyUSB3')
        self.declare_parameter('baudrate', 115200)
        
        # Retrieve parameter values
        self.device_path = self.get_parameter('device_path').value
        self.baudrate = self.get_parameter('baudrate').value
    
    def _setup_script_mappings(self):
        """Configure the mapping between RSCP message types and bash scripts."""
        # Map RSCP message types to their corresponding script paths
        self.scripts = {
            'set_stage': os.path.expanduser('~/eric_anatolian/rscp_communication/bash_scripts/run_on_SetStage.sh'),
            'arm_disarm': os.path.expanduser('~/eric_anatolian/rscp_communication/bash_scripts/run_on_ArmDisarm.sh'),
            'search_area': os.path.expanduser('~/eric_anatolian/rscp_communication/bash_scripts/run_on_SearchArea.sh'),
            'navigate_to_gps': os.path.expanduser('~/eric_anatolian/rscp_communication/bash_scripts/run_on_NavigateToGPS.sh'),
            'start_exploration': os.path.expanduser('~/eric_anatolian/rscp_communication/bash_scripts/run_on_StartExploration.sh'),
        }
        
        # Fallback script for unknown message types
        self.default_script = os.path.expanduser('~/rscp_communication/bash_scripts/run_on_default.sh')
    
    def _execute_script_async(self, script_path, message_type=None, payload=None):
        """
        Execute a bash script asynchronously to avoid blocking the serial listener.
        
        Args:
            script_path (str): Path to the bash script to execute
            message_type (str): RSCP message type for environment variable
            payload: Message payload for environment variable
            
        Returns:
            threading.Thread: The thread executing the script
        """
        def script_runner():
            try:
                # ================================================================
                # ENVIRONMENT SETUP
                # ================================================================
                # Pass message information to script via environment variables
                if message_type:
                    os.environ['RSCP_MESSAGE_TYPE'] = message_type
                if payload:
                    os.environ['RSCP_PAYLOAD'] = str(payload)
                
                self.get_logger().info(f'Executing script: {script_path}')
                
                # ================================================================
                # SCRIPT EXECUTION WITH REAL-TIME OUTPUT
                # ================================================================
                # Execute script with real-time output streaming
                process = subprocess.Popen(
                    script_path,
                    shell=True,
                    stdout=subprocess.PIPE,
                    stderr=subprocess.STDOUT,
                    text=True,
                    bufsize=1,  # Line buffered
                    universal_newlines=True
                )
                
                # Stream script output line by line to ROS logger
                for line in process.stdout:
                    self.get_logger().info(f'Script Output: {line.rstrip()}')
                
                # Wait for completion and log result
                process.wait()
                self.get_logger().info(f'Script completed with return code: {process.returncode}')
                
            except Exception as e:
                self.get_logger().error(f'Script execution failed for {script_path}: {e}')
        
        # Execute script in separate daemon thread
        thread = threading.Thread(target=script_runner, daemon=True)
        thread.start()
        return thread
    
    def _serial_listener(self):
        """
        Main serial communication loop that handles RSCP message reception and processing.
        
        Protocol Details:
        - Messages are COBS-encoded and delimited by 0x00 bytes
        - Each frame contains a Protocol Buffer RequestEnvelope message
        - Successfully parsed messages trigger corresponding script execution
        """
        # ====================================================================
        # SERIAL PORT INITIALIZATION
        # ====================================================================
        try:
            self.ser = serial.Serial(self.device_path, self.baudrate, timeout=0.1)
            self.ser.reset_input_buffer()  # Clear any existing data
            self.get_logger().info(f"Serial port ready: {self.ser.port} @ {self.baudrate} baud")
        except Exception as e:
            self.get_logger().error(f'Serial port initialization failed: {e}')
            return
        
        # ====================================================================
        # MESSAGE RECEPTION AND PROCESSING LOOP
        # ====================================================================
        message_buffer = bytearray()
        
        while rclpy.ok():
            # Read available data from serial port
            data_chunk = self.ser.read(self.ser.in_waiting or 1)
            if not data_chunk:
                continue
            
            message_buffer.extend(data_chunk)
            
            # ================================================================
            # FRAME EXTRACTION (COBS uses 0x00 as delimiter)
            # ================================================================
            while True:
                try:
                    # Find next frame delimiter
                    delimiter_index = message_buffer.index(0)
                except ValueError:
                    # No complete frame available yet
                    break
                
                # Extract frame and update buffer
                raw_frame = message_buffer[:delimiter_index]
                message_buffer = message_buffer[delimiter_index + 1:]
                
                if not raw_frame:
                    continue  # Skip empty frames
                
                # Process the extracted frame
                self._process_rscp_frame(bytes(raw_frame))
    
    def _process_rscp_frame(self, raw_frame):
        """
        Process a single RSCP frame through COBS decoding and protobuf parsing.
        
        Args:
            raw_frame (bytes): Raw COBS-encoded frame data
        """
        # ====================================================================
        # STEP 1: COBS DECODING
        # ====================================================================
        try:
            decoded_data = cobs.decode(raw_frame)
        except Exception as e:
            self.get_logger().error(
                f"COBS decoding failed ({type(e).__name__}: {e}), "
                f"raw frame: {raw_frame.hex()}"
            )
            return
        
        # ====================================================================
        # STEP 2: PROTOCOL BUFFER PARSING
        # ====================================================================
        request_envelope = rscp_protobuf.RequestEnvelope()
        try:
            request_envelope.ParseFromString(decoded_data)
        except Exception as e:
            self.get_logger().error(
                f"Protobuf parsing failed ({type(e).__name__}: {e}), "
                f"decoded data: {decoded_data.hex()}"
            )
            return
        
        # ====================================================================
        # STEP 3: MESSAGE TYPE IDENTIFICATION AND DISPATCH
        # ====================================================================
        message_type = request_envelope.WhichOneof('request')
        if message_type is None:
            self.get_logger().warning("Received empty RequestEnvelope - no request field set")
            return
        
        self.get_logger().info(f"Processing RSCP request: {message_type}")
        
        # ====================================================================
        # STEP 4: SCRIPT EXECUTION
        # ====================================================================
        # Select appropriate script based on message type
        script_path = self.scripts.get(message_type, self.default_script)
        message_payload = getattr(request_envelope, message_type)
        
        # Execute script asynchronously
        self._execute_script_async(script_path, message_type, message_payload)


if __name__ == '__main__':
    main()
