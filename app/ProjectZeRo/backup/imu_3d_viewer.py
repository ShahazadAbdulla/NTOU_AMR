import cv2
import numpy as np
import json
from dora import Node

def quat_to_rot_mat(qx, qy, qz, qw):
    """Converts a quaternion into a 3x3 rotation matrix."""
    return np.array([
        [1 - 2*qy**2 - 2*qz**2,     2*qx*qy - 2*qz*qw,     2*qx*qz + 2*qy*qw],
        [    2*qx*qy + 2*qz*qw, 1 - 2*qx**2 - 2*qz**2,     2*qy*qz - 2*qx*qw],
        [    2*qx*qz - 2*qy*qw,     2*qy*qz + 2*qx*qw, 1 - 2*qx**2 - 2*qy**2]
    ])

def draw_3d_wedge(img, rot_mat):
    """Projects a 3D directional wedge onto the 2D OpenCV image."""
    s = 100
    
    # Vertices of a wedge. 
    # V0 is the nose (Camera forward, which is -Z in Android coordinates)
    # V1, V2, V3 form the flat rear screen of the phone (+Z)
    vertices = np.array([
        [ 0,  0, -s*1.5], # 0: Nose (Forward)
        [-s, -s,  s],     # 1: Bottom left rear
        [ s, -s,  s],     # 2: Bottom right rear
        [ 0,  s,  s],     # 3: Top center rear
    ])

    # Apply the IMU rotation matrix
    rotated = np.dot(vertices, rot_mat.T)

    # 3D to 2D Orthographic Projection
    z_offset = 300
    projected = []
    center_x, center_y = img.shape[1] // 2, img.shape[0] // 2

    for v in rotated:
        x, y, z = v
        factor = 400 / (z + z_offset)
        px = int(x * factor + center_x)
        py = int(-y * factor + center_y) # Invert Y for OpenCV screen coordinates
        projected.append((px, py))

    # Draw the rear flat screen (Green)
    cv2.line(img, projected[1], projected[2], (0, 255, 0), 2, cv2.LINE_AA)
    cv2.line(img, projected[2], projected[3], (0, 255, 0), 2, cv2.LINE_AA)
    cv2.line(img, projected[3], projected[1], (0, 255, 0), 2, cv2.LINE_AA)

    # Draw the sides connecting to the nose (Blue)
    cv2.line(img, projected[1], projected[0], (255, 100, 0), 2, cv2.LINE_AA)
    cv2.line(img, projected[2], projected[0], (255, 100, 0), 2, cv2.LINE_AA)
    
    # Draw the top ridge line to the nose in RED to clearly show Forward
    cv2.line(img, projected[3], projected[0], (0, 0, 255), 4, cv2.LINE_AA)

def main():
    node = Node()
    print("Project ZeRo - Dashcam 3D Visualizer Active", flush=True)

    window_name = "Dashcam Orientation (Wedge)"
    cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)

    for event in node:
        if event["type"] == "INPUT" and event["id"] == "imu":
            try:
                data = json.loads(event["value"][0].as_py())

                if data["mode"] != "FUSED":
                    continue # Skip raw data, we need the quaternion for 3D

                # Black canvas
                frame = np.zeros((600, 600, 3), dtype=np.uint8)

                # Get rotation matrix and draw
                rot_mat = quat_to_rot_mat(data["q_x"], data["q_y"], data["q_z"], data["q_w"])
                draw_3d_wedge(frame, rot_mat)

                # UI Overlay
                cv2.putText(frame, f"Yaw Drift Tracking", (20, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
                cv2.putText(frame, f"qX: {data['q_x']:.3f}", (20, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
                cv2.putText(frame, f"qY: {data['q_y']:.3f}", (20, 100), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
                cv2.putText(frame, f"qZ: {data['q_z']:.3f}", (20, 130), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
                cv2.putText(frame, f"qW: {data['q_w']:.3f}", (20, 160), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)

                cv2.imshow(window_name, frame)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break

            except Exception as e:
                print(f"Visualization error: {e}", flush=True)
                
        elif event["type"] == "STOP":
            break

    cv2.destroyAllWindows()
    print("Visualizer Node shutting down.")

if __name__ == "__main__":
    main()
