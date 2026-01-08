"""
This contains all the math and logic for the robotic arm.

BASE servo: rotates the entire arm left/right
SHOULDER servo: lifts the arm up/down
ELBOW servo: bends the arm
CAMERA servo: keeps the camera level with the ground
"""

import math
import time
from adafruit_servokit import ServoKit


class RoboticArm:
    
    def __init__(self, l1=80.0, l2=80.0, camera_neutral=45):
        """
        Initialize the robotic arm controller.
        Args:
            l1: Length of first arm segment in mm (shoulder to elbow)
            l2: Length of second arm segment in mm (elbow to wrist)
            camera_neutral: The servo angle where camera is level
        """
        # Initialize the servo controller (16-channel PCA9685)
        self.kit = ServoKit(channels=16)
        
        # Define which servo is connected to which channel
        self.BASE = 0 #rotates 0-180 degrees
        self.SHOULDER = 1
        self.ELBOW = 2
        self.CAMERA = 3
        
        # Arm segment lengths in mm
        self.L1 = l1 #shoulder to elbow
        self.L2 = l2 #elbow to wrist
        
        # Camera calibration value
        self.CAMERA_NEUTRAL = camera_neutral
        
        # starts at a safe home position
        self.current_pos = {"x": 80.0, "y": 0.0, "z": 80.0}
    
    
    def move_to(self, x, y, z):
        """
        Move the arm to a target 3D position using inverse kinematics.
        Coordinate system
        X axis: forward/backward (positive = forward)
        Y axis: left/right (positive = left)
        Z axis: up/down (positive = up)
        """
        
 # Calculate BASE rotation angle
        base_rad = math.atan2(y, x)
        
        # Convert to degrees and adjust for servo mounting
        physical_base = 90 + math.degrees(base_rad)
        
        
# Calculate reach distance
        r = math.sqrt(x**2 + y**2) # calculate the distance in the horizontal plane
        
        # Total 3D distance from shoulder to target
        dist_sq = r**2 + z**2
        dist = math.sqrt(dist_sq)
        
# here we check if target is reachable
        max_reach = self.L1 + self.L2
        min_reach = abs(self.L1 - self.L2)
        
        if dist > max_reach or dist < min_reach:
            print(f"Target out of reach! Distance: {dist:.1f}mm (min: {min_reach:.1f}, max: {max_reach:.1f})")
            return None
        
        
# Calculate ELBOW angle (Law of Cosines)
        cos_elbow = (self.L1**2 + self.L2**2 - dist_sq) / (2 * self.L1 * self.L2)
        
        # Clamp to [-1, 1] to avoid math errors from floating point precision
        cos_elbow = max(-1, min(1, cos_elbow))
        
        elbow_math_deg = math.degrees(math.acos(cos_elbow))
        
        

# Calculate SHOULDER angle (Law of Cosines)
        alpha = math.atan2(z, r)  # Angle to target in vertical plane
        
        # Calculate beta using law of cosines again
        cos_beta = (self.L1**2 + dist_sq - self.L2**2) / (2 * self.L1 * dist)
        cos_beta = max(-1, min(1, cos_beta))
        beta = math.acos(cos_beta)
        
        shoulder_math_deg = math.degrees(alpha + beta)
        
        

#Convert to physical servo angles

        # Servo angles go from 0° to 180°
        # Shoulder 90° is horizontal, subtract to get servo angle
        p_shoulder = 90 - shoulder_math_deg
    
        # Elbow 180° is straight, subtract from 180 to get servo angle
        p_elbow = 180 - elbow_math_deg
        
        p_base = physical_base
        
        # Clamp all angles to valid servo range [0, 180]
        p_shoulder = max(0, min(180, p_shoulder))
        p_elbow = max(0, min(180, p_elbow))
        p_base = max(0, min(180, p_base))
        
        
        # ========================================
        # STEP 6: Calculate CAMERA angle (keep level)
        # ========================================
        # We want the camera to always point at the horizon
        # Calculate the total pitch (tilt) of the arm
        arm_pitch = shoulder_math_deg - (180 - elbow_math_deg)
        
        # Counter-rotate the camera by the same amount
        p_camera = self.CAMERA_NEUTRAL - arm_pitch
        
        # Clamp to valid range
        p_camera = max(0, min(180, p_camera))
        
        

# Send commands to servos

        self.kit.servo[self.BASE].angle = p_base
        self.kit.servo[self.SHOULDER].angle = p_shoulder
        self.kit.servo[self.ELBOW].angle = p_elbow
        self.kit.servo[self.CAMERA].angle = p_camera
        
        return True
    

    def move_smooth(self, target_x, target_y, target_z, steps=50, speed=0.02):
        """
        Move the arm smoothly from current position to target position.
        
        Instead of jumping directly to the target, this breaks the movement
        into many small steps, creating a smooth interpolated path. ---> Our TA bhaskers idea
        
        """
        
        # Calculate how much to move in each axis per step
        step_x = (target_x - self.current_pos["x"]) / steps
        step_y = (target_y - self.current_pos["y"]) / steps
        step_z = (target_z - self.current_pos["z"]) / steps
        
        # Execute each small step
        for i in range(steps):
            # Increment position by one step
            self.current_pos["x"] += step_x
            self.current_pos["y"] += step_y
            self.current_pos["z"] += step_z
            
            # Move arm to this intermediate position
            success = self.move_to(
                self.current_pos["x"],
                self.current_pos["y"],
                self.current_pos["z"]
            )
            
            # If movement out of reach thenstop
            if not success:
                print("Smooth movement interrupted, point out of reach!")
                return False
            
            # Small delay to make movement visible
            time.sleep(speed)
        
        # Ensuring we end exactly at target 
        self.current_pos["x"] = target_x
        self.current_pos["y"] = target_y
        self.current_pos["z"] = target_z
        
        return True
    
    
    def get_position(self):
        """
        Get the current position of the arm.
        Returns:
            Dictionary with 'x', 'y', 'z' keys
        """
        return self.current_pos.copy()
    
    
    def home(self):
        """
        Move the arm to a safe home position.
        """
        print("Moving to home position...")
        self.move_smooth(80.0, 0.0, 80.0, steps=50, speed=0.02)