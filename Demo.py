# Simplle program that moves the arm around in the given coordinates

import time
from arm_kinematics import RoboticArm


def main():

    # Initialize the arm
    arm = RoboticArm(l1=80.0, l2=80.0, camera_neutral=45)
    print("Arm initialized")
    print(f"Starting position: {arm.get_position()}")


    print("moving 1")
    arm.move_smooth(
        target_x=-155,  
        target_y=0,   
        target_z=20,    
        steps=100,      
        speed=0.01     
    )
    time.sleep(1)
    
    
    print("moving 2")
    arm.move_smooth(
        target_x=100,   
        target_y=0,
        target_z=20,    
        steps=100,
        speed=0.01
    )
    time.sleep(1)
    

    print("moving 3")
    arm.move_smooth(
        target_x=0,     
        target_y=-100,  
        target_z=90,    
        steps=100,
        speed=0.01
    )
    
    time.sleep(1)
    
    
    print("returning home")
    arm.home()
    
    print(f"Final position: {arm.get_position()}")

'''
def interactive_mode():
    arm = RoboticArm(l1=80.0, l2=80.0, camera_neutral=45)
    
    print("Interactive mode")
    print("Enter coordinates to move the arm.")
    print("Type 'home' to return to home position.")
    print("Type 'quit' to exit")
    
    while True:
        try:
            command = input("Enter command (or x,y,z coordinates): ").strip().lower()
            
            if command == 'quit':
                print(" Goodbye!")
                break
            
            elif command == 'home':
                arm.home()
            
            else:
                # Parse coordinates like "100,0,80"
                coords = [float(x.strip()) for x in command.split(',')]
                if len(coords) != 3:
                    print(" Please enter exactly 3 coordinates: x,y,z")
                    continue
                
                x, y, z = coords
                print(f"Moving to ({x}, {y}, {z})...")
                
                success = arm.move_smooth(x, y, z, steps=50, speed=0.02)
                
                if success:
                    print(f" Moved to: {arm.get_position()}")
                else:
                    print(f" Movement failed")
        
        except ValueError:
            print(" Invalid input. Use format: x,y,z (example: 100,0,80)")
        except KeyboardInterrupt:
            print("\ Goodbye!")
            break
'''
if __name__ == "__main__":
    main()
