import cv2
import numpy as np
import time
import serial as pyserial
import serial.tools.list_ports
import threading
import queue
from ultralytics import YOLO

class StereoDepthObstacleDetection:
    """
    Optimized Real-time Stereo Depth Estimation with Obstacle Detection using cv2.inRange.
    Includes PySerial communication with ESP32 and YOLO object verification.
    Uses manual calibration only (XML reading disabled).
    """
    
    def __init__(self, cam_left=1, cam_right=0, serial_port=None, baudrate=115200, yolo_model_path='yolov8n.pt'):
        self.cam_left_id = cam_left
        self.cam_right_id = cam_right
        self.cap_left = cv2.VideoCapture(self.cam_left_id)
        self.cap_right = cv2.VideoCapture(self.cam_right_id)
        
        if not self.cap_left.isOpened() or not self.cap_right.isOpened():
            print("Error: Could not open one or both cameras.")
            exit()
            
        self.set_camera_properties()
        
        # Use manual calibration only (XML reading disabled)
        print("Using manual calibration only (XML reading disabled)...")
        self.setup_manual_calibration_and_rectification()
            
        self.setup_sgbm_matcher() 

        self.baseline = np.linalg.norm(self.T) / 10.0
        self.focal_length = self.P1[0, 0]
        print(f"Baseline: {self.baseline:.2f} cm, Focal Length: {self.focal_length:.2f} px")
        
        # Obstacle detection parameters (from reference code)
        self.max_depth = 400  # maximum distance the setup can measure (in cm)
        self.min_depth = 50   # minimum distance the setup can measure (in cm)
        self.depth_thresh = 200  # Threshold for SAFE distance (in cm)
        
        # YOLO model for object verification
        self.yolo_model = None
        self.setup_yolo_model(yolo_model_path)
        
        # Serial communication setup
        self.serial_port = serial_port
        self.baudrate = baudrate
        self.ser = None
        self.serial_message_queue = queue.Queue()
        self.setup_serial_communication()
        
        # Command states for 1-second interval
        self.last_command = None
        self.current_command = "F"  # Default command
        self.last_command_time = 0
        self.command_interval = 1.0  # Send command every 1 second
        
        # Start serial reading thread
        self.serial_read_thread = None
        self.stop_serial_thread = False
        self.start_serial_reading()
        
        # Mouse callback for depth measurement
        cv2.namedWindow('Stereo Vision: Depth + YOLO Verification | Disparity Map', cv2.WINDOW_NORMAL)
        cv2.setMouseCallback('Stereo Vision: Depth + YOLO Verification | Disparity Map', self.mouse_click)

    def setup_yolo_model(self, model_path):
        """Initialize YOLO model for object verification."""
        try:
            print(f"Loading YOLO model from {model_path}...")
            self.yolo_model = YOLO(model_path)
            print("YOLO model loaded successfully!")
        except Exception as e:
            print(f"Error loading YOLO model: {e}")
            print("Continuing without YOLO object verification...")
            self.yolo_model = None

    def detect_objects_yolo(self, frame):
        """Detect objects using YOLO and return detections."""
        if self.yolo_model is None:
            return []
        
        try:
            results = self.yolo_model(frame, verbose=False)
            detections = []
            
            for r in results:
                for box in r.boxes:
                    bbox = box.xyxy[0].cpu().numpy()
                    class_id = int(box.cls[0])
                    confidence = box.conf[0].cpu().numpy()
                    class_name = self.yolo_model.names[class_id]
                    
                    detections.append({
                        'bbox': bbox,
                        'class': class_name,
                        'confidence': confidence,
                        'class_id': class_id
                    })
            
            return detections
        except Exception as e:
            print(f"Error in YOLO detection: {e}")
            return []

    def check_object_in_contour(self, contour_bbox, yolo_detections, overlap_threshold=0.1):
        """
        Check if any YOLO detection overlaps significantly with the depth-based contour.
        Returns True if an object to avoid is found in the contour region.
        """
        if not yolo_detections:
            return False, None
        
        contour_x1, contour_y1, contour_x2, contour_y2 = contour_bbox
        contour_area = (contour_x2 - contour_x1) * (contour_y2 - contour_y1)
        
        for detection in yolo_detections:
            if detection['confidence'] > 0.2:
                det_x1, det_y1, det_x2, det_y2 = detection['bbox']
                
                # Calculate intersection area
                inter_x1 = max(contour_x1, det_x1)
                inter_y1 = max(contour_y1, det_y1)
                inter_x2 = min(contour_x2, det_x2)
                inter_y2 = min(contour_y2, det_y2)
                
                if inter_x1 < inter_x2 and inter_y1 < inter_y2:
                    intersection_area = (inter_x2 - inter_x1) * (inter_y2 - inter_y1)
                    
                    # Calculate overlap ratio (intersection over contour area)
                    overlap_ratio = intersection_area / contour_area
                    
                    if overlap_ratio >= overlap_threshold:
                        return True, detection
        
        return False, None

    def setup_serial_communication(self):
        """Initialize serial communication with ESP32."""
        try:
            # If no serial port specified, show available ports and ask user
            if self.serial_port is None:
                self.serial_port = self.ask_user_for_serial_port()
                if self.serial_port is None:
                    print("No serial port selected. Continuing without serial communication.")
                    return
            
            print(f"Attempting to connect to {self.serial_port} at {self.baudrate} baud...")
            
            self.ser = pyserial.Serial(
                port=self.serial_port,
                baudrate=self.baudrate,
                parity=pyserial.PARITY_NONE,
                stopbits=pyserial.STOPBITS_ONE,
                bytesize=pyserial.EIGHTBITS,
                timeout=1
            )
            
            # Wait for connection to establish
            time.sleep(2)
            print(f"Serial communication established with {self.serial_port} at {self.baudrate} baud")
            
        except Exception as e:
            print(f"Error setting up serial communication: {e}")
            print("Continuing without serial communication...")
            self.ser = None

    def ask_user_for_serial_port(self):
        """Show available serial ports and let user choose manually."""
        print("\n" + "="*50)
        print("SERIAL PORT SELECTION")
        print("="*50)
        
        # Get all available serial ports
        ports = list(pyserial.tools.list_ports.comports())
        
        if not ports:
            print("No serial ports found!")
            return None
        
        # Display available ports
        print("Available serial ports:")
        for i, port in enumerate(ports):
            print(f"  {i+1}. {port.device} - {port.description}")
        
        print(f"  {len(ports)+1}. Enter COM port manually")
        print(f"  {len(ports)+2}. Skip serial communication")
        
        while True:
            try:
                choice = input(f"\nSelect an option (1-{len(ports)+2}): ").strip()
                
                if choice.isdigit():
                    choice_num = int(choice)
                    
                    if 1 <= choice_num <= len(ports):
                        selected_port = ports[choice_num-1].device
                        print(f"Selected: {selected_port}")
                        return selected_port
                    
                    elif choice_num == len(ports) + 1:
                        # Manual COM port entry
                        manual_port = input("Enter COM port (e.g., COM3, COM4, /dev/ttyUSB0): ").strip()
                        if manual_port:
                            print(f"Selected: {manual_port}")
                            return manual_port
                        else:
                            print("Invalid COM port entered.")
                    
                    elif choice_num == len(ports) + 2:
                        print("Skipping serial communication.")
                        return None
                
                print(f"Please enter a number between 1 and {len(ports)+2}")
                
            except (ValueError, KeyboardInterrupt):
                print("\nInvalid input. Please try again.")
            except Exception as e:
                print(f"Error: {e}")

    def start_serial_reading(self):
        """Start a background thread to continuously read from ESP32."""
        if self.ser and self.ser.is_open:
            self.stop_serial_thread = False
            self.serial_read_thread = threading.Thread(target=self.read_serial_messages)
            self.serial_read_thread.daemon = True
            self.serial_read_thread.start()
            print("Started serial reading thread...")

    def read_serial_messages(self):
        """Background thread function to continuously read messages from ESP32."""
        print("Serial reading thread started. Listening for ESP32 messages...")
        while not self.stop_serial_thread and self.ser and self.ser.is_open:
            try:
                if self.ser.in_waiting:
                    # Read all available lines
                    while self.ser.in_waiting:
                        line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                        if line:
                            # Print to terminal immediately
                            print(f"ESP32: {line}")
                            # Also store in queue if needed for other purposes
                            self.serial_message_queue.put(line)
                time.sleep(0.01)  # Small delay to prevent excessive CPU usage
            except Exception as e:
                if not self.stop_serial_thread:
                    print(f"Error reading serial: {e}")
                break
        print("Serial reading thread stopped.")

    def send_command(self, command):
        """Send command to ESP32 with 1-second interval."""
        current_time = time.time()
        
        # Check if 1 second has passed since last command
        if current_time - self.last_command_time < self.command_interval:
            return
        
        if self.ser and self.ser.is_open:
            try:
                # Format: "COMMAND\n"
                message = f"{command}\n"
                self.ser.write(message.encode('utf-8'))
                self.last_command = command
                self.last_command_time = current_time
                print(f"PC -> ESP32: {command} (sent every {self.command_interval} second)")
                
            except Exception as e:
                print(f"Error sending command: {e}")

    def determine_obstacle_command(self, obstacle_detected, obstacle_position, obstacle_depth, verified_obstacle,object_type):
        """Determine the appropriate command based on obstacle detection."""
        if not obstacle_detected or not verified_obstacle:
            # No obstacle or unverified obstacle - send FORWARD command
            return "F"
        elif object_type == "Human" :
            return "H"
        else:
            # Verified obstacle detected - send command based on position
            if obstacle_position == "LEFT":
                return "R"  # Turn right if obstacle on left
            elif obstacle_position == "RIGHT":
                return "L"   # Turn left if obstacle on right
            else:  # CENTER
                # Moderate distance
                return "R"

    def mouse_click(self, event, x, y, flags, param):
        """Mouse callback function to display depth at clicked point."""
        if event == cv2.EVENT_LBUTTONDBLCLK and hasattr(self, 'depth_map'):
            # Adjust x coordinate if clicking on the combined display
            if x >= self.width:
                x -= self.width
            if 0 <= x < self.width and 0 <= y < self.height:
                depth_value = self.depth_map[y, x]
                print(f"Distance at ({x}, {y}) = {depth_value:.2f} cm")

    def set_camera_properties(self):
        """Set camera resolution, ROI, and zone boundaries."""
        self.width, self.height = 640, 480
        
        self.cap_left.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        self.cap_left.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        self.cap_right.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        self.cap_right.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)

        self.roi_vertices = np.array([
            (280, 280), (360, 280), (self.width, self.height), (0, self.height)
        ], dtype=np.int32)

        self.zone_boundary_left = 220
        self.zone_boundary_right = 420
        self.color_info = (255, 255, 0) # Cyan for guidelines

    def setup_manual_calibration_and_rectification(self):
        """Sets up calibration and rectification using hardcoded values."""
        print("Loading MANUAL calibration data...")
        self.K1 = np.array([
            [927.74605080, 0.0, 284.12117832],
            [0.0, 934.31985090, 254.12343189],
            [0.0, 0.0, 1.0]
        ])
                
        # Intrinsic parameters - Right Camera  
        self.K2 = np.array([
            [934.66745195, 0.0, 297.91336457],
            [0.0, 940.68645368, 248.75853761],
            [0.0, 0.0, 1.0]
        ])

        # Distortion Coefficients - Left Camera
        self.D1 = np.array([[0.12453996], [0.38217399], [0.00288491], [0.00171692], [-5.85100310]])

        # Distortion Coefficients - Right Camera
        self.D2 = np.array([[0.08789484], [1.99437608], [-0.00276750], [0.00613820], [-20.11036379]])

        # Extrinsics: Rotation between cameras
        self.R = np.array([
            [0.99999253, 0.00385485, -0.00026472],
            [-0.00385219, 0.99994820, 0.00942069],
            [0.00030102, -0.00941960, 0.99995559]
        ])

        # Translation vector between cameras (in mm)
        self.T = np.array([[-82.07051694],[2.03908621],[2.78366730]])
        print("Manual calibration data set. Performing stereo rectification...")
        self.R1, self.R2, self.P1, self.P2, self.Q, _, _ = cv2.stereoRectify(self.K1, self.D1, self.K2, self.D2, (self.width, self.height), self.R, self.T, alpha=0)
        self.map1x, self.map1y = cv2.initUndistortRectifyMap(self.K1, self.D1, self.R1, self.P1, (self.width, self.height), cv2.CV_32FC1)
        self.map2x, self.map2y = cv2.initUndistortRectifyMap(self.K2, self.D2, self.R2, self.P2, (self.width, self.height), cv2.CV_32FC1)
        print("Rectification maps created successfully.")

    def setup_sgbm_matcher(self):
        """Creates and configures the SGBM stereo matcher with hardcoded values."""
        print("Setting up SGBM matcher with manual parameters...")
        blockSize = 5 
        self.stereo_sgbm = cv2.StereoSGBM_create(minDisparity=0, numDisparities=11 * 16, blockSize=blockSize, mode=cv2.STEREO_SGBM_MODE_SGBM_3WAY)
        p1_val = 8 * 3 * (blockSize ** 2)
        p2_val = 32 * 3 * (blockSize ** 2)
        self.stereo_sgbm.setP1(p1_val)
        self.stereo_sgbm.setP2(p2_val)
        self.stereo_sgbm.setDisp12MaxDiff(10)
        self.stereo_sgbm.setUniquenessRatio(5)
        self.stereo_sgbm.setSpeckleWindowSize(50)
        self.stereo_sgbm.setSpeckleRange(2)
        self.stereo_sgbm.setPreFilterCap(31)
        print("SGBM matcher configured.")

    def obstacle_avoid(self, frame, depth_map,normal_frame):
        """
        Detect obstacles using cv2.inRange and verify with YOLO.
        Draw warnings only when YOLO confirms the object needs to be avoided.
        """
        output_frame = frame.copy()
        obstacle_detected = False
        obstacle_position = "CENTER"  # Default position
        obstacle_depth = 0
        verified_obstacle = False
        object_type = "Unknown"
        
        # Run YOLO detection on the frame
        yolo_detections = self.detect_objects_yolo(normal_frame)
        
        # Draw YOLO detections (for visualization)
        for detection in yolo_detections:
            if detection['confidence'] > 0.2    :
                x1, y1, x2, y2 = map(int, detection['bbox'])
                class_name = detection['class']
                confidence = detection['confidence']
                
                # Draw YOLO bounding box in blue
                cv2.rectangle(output_frame, (x1, y1), (x2, y2), (255, 0, 0), 2)
                label = f"{class_name} {confidence:.2f}"
                cv2.putText(output_frame, label, (x1, y1 - 10), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
        
        # Mask to segment regions with depth less than threshold
        mask = cv2.inRange(depth_map, 10, self.depth_thresh)

        # Check if a significantly large obstacle is present and filter out smaller noisy regions
        if np.sum(mask)/255.0 > 0.01 * mask.shape[0] * mask.shape[1]:

            # Contour detection 
            contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
            cnts = sorted(contours, key=cv2.contourArea, reverse=True)
            
            # Check if detected contour is significantly large (to avoid multiple tiny regions)
            if cnts and cv2.contourArea(cnts[0]) > 0.01 * mask.shape[0] * mask.shape[1]:

                x, y, w, h = cv2.boundingRect(cnts[0])
                obstacle_detected = True

                # Determine obstacle position
                center_x = x + w // 2
                if center_x < self.zone_boundary_left:
                    obstacle_position = "LEFT"
                elif center_x > self.zone_boundary_right:
                    obstacle_position = "RIGHT"
                else:
                    obstacle_position = "CENTER"

                # finding average depth of region represented by the largest contour 
                mask2 = np.zeros_like(mask)
                cv2.drawContours(mask2, cnts, 0, (255), -1)

                # Calculating the average depth of the object closer than the safe distance
                depth_mean, _ = cv2.meanStdDev(depth_map, mask=mask2)
                obstacle_depth = depth_mean[0, 0]
                
                # Check if YOLO confirms this is an object to avoid
                verified_obstacle, yolo_detection = self.check_object_in_contour(
                    [x, y, x + w, y + h], yolo_detections
                )
                
                if verified_obstacle and yolo_detection:
                    object_type = yolo_detection['class']
                    # Draw verified obstacle bounding box in red
                    cv2.rectangle(output_frame, (x, y), (x + w, y + h), (0, 0, 255), 3)
                    
                    # Calculate center point of the rectangle
                    center_x = x + w // 2
                    center_y = y + h // 2
                    
                    # Create text to display
                    warning_text = "WARNING!"
                    object_text = f"{object_type}"
                    depth_text = f"{obstacle_depth:.2f} cm"
                    
                    # Calculate text sizes for proper positioning
                    (warning_width, warning_height), _ = cv2.getTextSize(warning_text, cv2.FONT_HERSHEY_SIMPLEX, 0.7, 2)
                    (object_width, object_height), _ = cv2.getTextSize(object_text, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2)
                    (depth_width, depth_height), _ = cv2.getTextSize(depth_text, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2)
                    
                    # Calculate total height needed for all text
                    total_text_height = warning_height + object_height + depth_height + 20
                    
                    # Calculate starting Y position to center text vertically in the rectangle
                    text_start_y = center_y - total_text_height // 2
                    
                    # Display warning text at the center of the rectangle
                    cv2.putText(output_frame, warning_text, 
                               (center_x - warning_width // 2, text_start_y + warning_height), 
                               cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2, cv2.LINE_AA)
                    
                    # Display object type
                    cv2.putText(output_frame, object_text, 
                               (center_x - object_width // 2, text_start_y + warning_height + object_height + 10), 
                               cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2, cv2.LINE_AA)
                    
                    # Display depth
                    cv2.putText(output_frame, depth_text, 
                               (center_x - depth_width // 2, text_start_y + warning_height + object_height + depth_height + 20), 
                               cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2, cv2.LINE_AA)
                    
                    # Add position indicator at top of frame
                    cv2.putText(output_frame, f"Position: {obstacle_position}", 
                               (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
                else:
                    # Draw unverified obstacle in yellow (depth detection but no YOLO confirmation)
                    cv2.rectangle(output_frame, (x, y), (x + w, y + h), (0, 255, 255), 2)
                    
                    # Calculate center point for unverified obstacle text
                    center_x = x + w // 2
                    center_y = y + h // 2
                    
                    alert_text = "Depth Alert"
                    (alert_width, alert_height), _ = cv2.getTextSize(alert_text, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2)
                    
                    cv2.putText(output_frame, alert_text, 
                               (center_x - alert_width // 2, center_y), 
                               cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
        else:
            cv2.putText(output_frame, "SAFE!", (100, 100), cv2.FONT_HERSHEY_SIMPLEX, 
                       2, (0, 255, 0), 3, cv2.LINE_AA)
            
        return output_frame, obstacle_detected, obstacle_position, obstacle_depth, verified_obstacle, object_type

    def visualize_results(self, frame, depth_map):
        """Visualize depth map and obstacle detection results."""
        display_frame = frame.copy()
        
        # Add zone boundaries
        cv2.line(display_frame, (self.zone_boundary_left, 0), 
                (self.zone_boundary_left, self.height), self.color_info, 2)
        cv2.line(display_frame, (self.zone_boundary_right, 0), 
                (self.zone_boundary_right, self.height), self.color_info, 2)
        
        # Apply obstacle detection with YOLO verification
        display_frame, obstacle_detected, obstacle_position, obstacle_depth, verified_obstacle, object_type = self.obstacle_avoid(display_frame, depth_map,frame)
        
        # Determine current command (but don't send it yet)
        self.current_command = self.determine_obstacle_command(
            obstacle_detected, obstacle_position, obstacle_depth, verified_obstacle,object_type
        )
        
        # Add detection status
        if verified_obstacle:
            status_text = f"VERIFIED: {object_type}"
            status_color = (0, 0, 255)
        elif obstacle_detected:
            status_text = "UNVERIFIED: Depth Only"
            status_color = (0, 255, 255)
        else:
            status_text = "CLEAR"
            status_color = (0, 255, 0)
            
        cv2.putText(display_frame, f"Status: {status_text}", 
                   (10, self.height - 40), cv2.FONT_HERSHEY_SIMPLEX, 0.5, status_color, 2)
        
        # Add current command to display
        cv2.putText(display_frame, f"Command: {self.current_command}", 
                   (10, self.height - 70), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 2)
        
        # Add serial interval info
        cv2.putText(display_frame, f"Serial: 1s interval", 
                   (self.width - 200, self.height - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        
        # Add depth information text
        if np.any(depth_map > 0):
            min_depth_val = np.min(depth_map[depth_map > 0])
            max_depth_val = np.max(depth_map[depth_map > 0])
            cv2.putText(display_frame, f"Depth range: {min_depth_val:.1f}-{max_depth_val:.1f} cm", 
                       (10, self.height - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        
        # Add serial status
        serial_status = "Serial: Connected" if self.ser and self.ser.is_open else "Serial: Disconnected"
        cv2.putText(display_frame, serial_status, (self.width - 200, 30), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        
        # Add YOLO status
        yolo_status = "YOLO: Active" if self.yolo_model else "YOLO: Inactive"
        cv2.putText(display_frame, yolo_status, (self.width - 200, 60), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        
        return display_frame

    def visualize_performance_stats(self, frame, depth_time, yolo_time, total_time):
        """Draws performance metrics on the top-left of the frame."""
        other_time = total_time - depth_time - yolo_time
        fps = 1000 / total_time if total_time > 0 else 0

        # Create text strings
        stats = [
            f"Total: {total_time:.1f}ms ({fps:.1f} FPS)",
            f"  - Depth: {depth_time:.1f}ms",
            f"  - YOLO: {yolo_time:.1f}ms",
            f"  - Other: {other_time:.1f}ms"
        ]

        y_offset = 60  # Start lower to avoid overlap with position text
        for i, text in enumerate(stats):
            # Draw semi-transparent background
            (w, h), _ = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2)
            cv2.rectangle(frame, (5, y_offset - h - 5), (10 + w, y_offset + 5), (0, 0, 0), -1)
            # Draw text
            cv2.putText(frame, text, (10, y_offset), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
            y_offset += 30

    def run(self):
        """Main loop for capturing, processing, and displaying video."""
        try:
            while True:
                # --- Start Total Timer ---
                start_total_time = time.perf_counter()

                ret_left, frame_left_raw = self.cap_left.read()
                ret_right, frame_right_raw = self.cap_right.read()
                if not ret_left or not ret_right: 
                    break
                
                frame_left = cv2.resize(frame_left_raw, (self.width, self.height))
                frame_right = cv2.resize(frame_right_raw, (self.width, self.height))
                frame_left_rect = cv2.remap(frame_left, self.map1x, self.map1y, cv2.INTER_LINEAR)
                frame_right_rect = cv2.remap(frame_right, self.map2x, self.map2y, cv2.INTER_LINEAR)
                
                # --- Start Depth Timer ---
                start_depth_time = time.perf_counter()
                gray_left = cv2.cvtColor(frame_left_rect, cv2.COLOR_BGR2GRAY)
                gray_right = cv2.cvtColor(frame_right_rect, cv2.COLOR_BGR2GRAY)
                raw_disparity = self.stereo_sgbm.compute(gray_left, gray_right)
                disparity_map = raw_disparity.astype(np.float32) / 16.0

                # Calculate depth map from disparity
                valid_disparity = disparity_map > 0.1
                self.depth_map = np.zeros_like(disparity_map)
                self.depth_map[valid_disparity] = (self.baseline * self.focal_length) / disparity_map[valid_disparity]
                
                # Filter depth map to valid range
                mask_temp = cv2.inRange(self.depth_map, self.min_depth, self.max_depth)
                self.depth_map = cv2.bitwise_and(self.depth_map, self.depth_map, mask=mask_temp)
                
                end_depth_time = time.perf_counter()
                # --- End Depth Timer ---

                # --- Start YOLO Timer ---
                start_yolo_time = time.perf_counter()
                result_frame = self.visualize_results(frame_left_rect, self.depth_map)
                end_yolo_time = time.perf_counter()
                # --- End YOLO Timer ---

                # Send command every 1 second
                self.send_command(self.current_command)

                disparity_display = cv2.normalize(disparity_map, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)
                disparity_display = cv2.applyColorMap(disparity_display, cv2.COLORMAP_JET)
                combined_display = np.hstack([result_frame, disparity_display])
                
                # --- End Total Timer and Calculate Timings ---
                end_total_time = time.perf_counter()
                depth_ms = (end_depth_time - start_depth_time) * 1000
                yolo_ms = (end_yolo_time - start_yolo_time) * 1000
                total_ms = (end_total_time - start_total_time) * 1000

                # Draw the performance stats on the final combined display
                self.visualize_performance_stats(combined_display, depth_ms, yolo_ms, total_ms)

                cv2.imshow('Stereo Vision: Depth + YOLO Verification | Disparity Map', combined_display)
                
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q'): 
                    break
                elif key == ord('s'):
                    timestamp = int(time.time())
                    cv2.imwrite(f'output_{timestamp}.png', combined_display)
                    print(f"Saved combined output with timestamp: {timestamp}")
        
        finally:
            # Cleanup
            self.stop_serial_thread = True
            if self.serial_read_thread and self.serial_read_thread.is_alive():
                self.serial_read_thread.join(timeout=1.0)
            
            if self.ser and self.ser.is_open:
                self.send_command("STOP")  # Send stop command before closing
                time.sleep(0.1)
                self.ser.close()
                print("Serial connection closed.")
            
            self.cap_left.release()
            self.cap_right.release()
            cv2.destroyAllWindows()
            print("Stereo Depth Obstacle Detection System Stopped")

if __name__ == "__main__":
    # Usage examples:
    
    # 1. With YOLO model:
    stereo_depth = StereoDepthObstacleDetection(
        cam_left=3, 
        cam_right=0, 
        yolo_model_path='Kuay.pt'  # Change to your YOLO model path
    )
    
    # 2. Without YOLO (depth-only):
    # stereo_depth = StereoDepthObstacleDetection(cam_left=1, cam_right=0)
    
    stereo_depth.run()