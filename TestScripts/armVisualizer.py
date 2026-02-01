import matplotlib.pyplot as plt
import numpy as np
import math
import random

# --- ARM CONFIGURATION (METERS) ---
L1 = 0.10  # Base Height (Joint 1 to Shoulder)
L2 = 0.40  # Upper Arm (Shoulder to Elbow)
L3 = 0.40  # Forearm (Elbow to Wrist)
L4 = 0.20  # Hand/Gripper Length (Wrist to Tip)

class armVisualizer:
    def __init__(self):
        # Current Joint Angles (Degrees)
        self.q = [0.0, 90.0, -90.0, 0.0, 0.0] # Initial "Home" pose
        
    def solve_ik(self, target_x, target_y, target_z, desired_pitch_deg=0.0):
        """
        Calculates the 5 joint angles to reach (x, y, z) with a specific pitch.
        Assumes a standard "Anthropomorphic" arm (Base-Shoulder-Elbow-Wrist).
        """
        # 1. Base Rotation (q1)
        q1 = math.atan2(target_y, target_x)
        
        # 2. Wrist Center Calculation
        pitch_rad = math.radians(desired_pitch_deg)
        r_target = math.sqrt(target_x**2 + target_y**2)
        r_wrist = r_target - (L4 * math.cos(pitch_rad))
        z_wrist = target_z - (L4 * math.sin(pitch_rad))
        z_wrist_rel = z_wrist - L1
        
        # 3. Planar IK (Shoulder & Elbow)
        dist_sq = r_wrist**2 + z_wrist_rel**2
        dist = math.sqrt(dist_sq)
        
        if dist > (L2 + L3):
            print("Target Unreachable (Too far)")
            return None
        
        try:
            alpha = math.acos((L2**2 + dist_sq - L3**2) / (2 * L2 * dist))
            beta  = math.acos((L2**2 + L3**2 - dist_sq) / (2 * L2 * L3))
        except ValueError:
            print("Target Unreachable (Math Error)")
            return None
            
        angle_to_vector = math.atan2(z_wrist_rel, r_wrist)
        q2 = angle_to_vector + alpha
        q3 = -1 * (math.pi - beta)
        
        # 4. Wrist Pitch (q4)
        q4 = pitch_rad - q2 - q3
        
        # 5. Wrist Roll (q5)
        # Keep existing roll if just moving position, or reset to 0
        q5 = self.q[4] if len(self.q) > 4 else 0.0
        
        return [math.degrees(q1), math.degrees(q2), math.degrees(q3), math.degrees(q4), q5]

    def forward_kinematics(self, joints_deg):
        q = [math.radians(a) for a in joints_deg]
        points = []
        
        # P0: Ground
        points.append([0, 0, 0])
        
        # P1: Shoulder
        p1 = np.array([0, 0, L1])
        points.append(p1)
        
        # Rotation Matrices
        R = np.array([
            [math.cos(q[0]), -math.sin(q[0]), 0],
            [math.sin(q[0]),  math.cos(q[0]), 0],
            [0, 0, 1]
        ])
        
        # Shoulder Pitch
        Ry_q2 = np.array([
            [math.cos(q[1]), 0, -math.sin(q[1])],
            [0, 1, 0],
            [math.sin(q[1]), 0, math.cos(q[1])]
        ])
        R = R @ Ry_q2
        p2 = p1 + (R @ np.array([L2, 0, 0]))
        points.append(p2)
        
        # Elbow Pitch
        Ry_q3 = np.array([
            [math.cos(q[2]), 0, -math.sin(q[2])],
            [0, 1, 0],
            [math.sin(q[2]), 0, math.cos(q[2])]
        ])
        R = R @ Ry_q3
        p3 = p2 + (R @ np.array([L3, 0, 0]))
        points.append(p3)
        
        # Wrist Pitch
        Ry_q4 = np.array([
            [math.cos(q[3]), 0, -math.sin(q[3])],
            [0, 1, 0],
            [math.sin(q[3]), 0, math.cos(q[3])]
        ])
        R = R @ Ry_q4

        # Wrist Roll (Local X Rotation)
        Rx_q5 = np.array([
            [1, 0, 0],
            [0, math.cos(q[4]), -math.sin(q[4])],
            [0, math.sin(q[4]),  math.cos(q[4])]
        ])
        R = R @ Rx_q5
        
        p4 = p3 + (R @ np.array([L4, 0, 0]))
        points.append(p4)
        
        return np.array(points), R

# --- VISUALIZATION SETUP ---
fig = plt.figure(figsize=(10, 8))
ax = fig.add_subplot(111, projection='3d')
arm = armVisualizer()
current_target = None # Store target so it doesn't vanish when rolling

def update_plot(target_pos=None):
    ax.cla()
    limit = L2 + L3 + L4
    ax.set_xlim([-limit, limit])
    ax.set_ylim([-limit, limit])
    ax.set_zlim([0, limit*1.5])
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_zlabel('Z (m)')
    
    # Updated Title with Controls
    ax.set_title("5-Axis Simulator\nSPACE: New Target | 'A'/'D': Roll Wrist")
    
    if target_pos is not None:
        ax.scatter([target_pos[0]], [target_pos[1]], [target_pos[2]], color='r', s=100, label='Target')
    
    points, tip_rotation = arm.forward_kinematics(arm.q)
    
    xs = points[:, 0]
    ys = points[:, 1]
    zs = points[:, 2]
    
    ax.plot(xs, ys, zs, linewidth=4, marker='o', markersize=8)
    
    # Tip Coordinate Frame
    tip_pos = points[-1]
    axis_len = 0.15
    
    x_vec = tip_rotation @ np.array([axis_len, 0, 0])
    ax.plot([tip_pos[0], tip_pos[0]+x_vec[0]], 
            [tip_pos[1], tip_pos[1]+x_vec[1]], 
            [tip_pos[2], tip_pos[2]+x_vec[2]], 'r-', linewidth=2)
            
    y_vec = tip_rotation @ np.array([0, axis_len, 0])
    ax.plot([tip_pos[0], tip_pos[0]+y_vec[0]], 
            [tip_pos[1], tip_pos[1]+y_vec[1]], 
            [tip_pos[2], tip_pos[2]+y_vec[2]], 'g-', linewidth=2)

    z_vec = tip_rotation @ np.array([0, 0, axis_len])
    ax.plot([tip_pos[0], tip_pos[0]+z_vec[0]], 
            [tip_pos[1], tip_pos[1]+z_vec[1]], 
            [tip_pos[2], tip_pos[2]+z_vec[2]], 'b-', linewidth=2)

    # Added Roll info to display
    info = f"Base={arm.q[0]:.0f}, Shldr={arm.q[1]:.0f}, Elbw={arm.q[2]:.0f}, Pitch={arm.q[3]:.0f}, Roll={arm.q[4]:.0f}"
    ax.text2D(0.05, 0.95, info, transform=ax.transAxes)
    
    plt.draw()

def on_key(event):
    global current_target
    
    if event.key == ' ':
        # Generate new random target
        r = random.uniform(0.3, 0.8)
        theta = random.uniform(-math.pi/2, math.pi/2)
        tx = r * math.cos(theta)
        ty = r * math.sin(theta)
        tz = random.uniform(0.0, 0.6)
        
        print(f"New Target: ({tx:.2f}, {ty:.2f}, {tz:.2f})")
        current_target = [tx, ty, tz]
        
        new_angles = arm.solve_ik(tx, ty, tz, desired_pitch_deg=0.0)
        
        if new_angles:
            # Preserve current roll when moving to new position
            new_angles[4] = arm.q[4] 
            
            start_angles = np.array(arm.q)
            target_angles = np.array(new_angles)
            steps = 30
            for i in range(1, steps + 1):
                t = i / steps
                current_angles = start_angles + (target_angles - start_angles) * t
                arm.q = current_angles.tolist()
                update_plot(target_pos=current_target)
                plt.pause(0.001)
        else:
            print("Target Unreachable!")

    # Wrist Roll Control
    elif event.key == 'a':
        arm.q[4] -= 5.0 # Roll Left
        update_plot(target_pos=current_target)
        
    elif event.key == 'd':
        arm.q[4] += 5.0 # Roll Right
        update_plot(target_pos=current_target)

fig.canvas.mpl_connect('key_press_event', on_key)

update_plot()
plt.show()