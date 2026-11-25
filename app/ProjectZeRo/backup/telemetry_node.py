import json
import time
from dora import Node

def main():
    node = Node()
    print("Telemetry Viewer Node Active. Waiting for sensor data...", flush=True)

    # Tracking variables for Hz calculation
    imu_count = 0
    gps_count = 0
    start_time = time.time()

    for event in node:
        if event["type"] == "INPUT":
            try:
                # Extract the JSON string from the PyArrow array
                arrow_array = event["value"]
                data_str = arrow_array[0].as_py()
                data = json.loads(data_str)

                input_id = event["id"]
                current_time = time.time()

                if input_id == "imu":
                    imu_count += 1
                    # Print IMU data at a throttled rate (every 20 frames) so you can actually read it
                    if imu_count % 20 == 0:
                        if data["mode"] == "FUSED":
                            print(f"[IMU FUSED] qX:{data['q_x']:.3f} qY:{data['q_y']:.3f} qZ:{data['q_z']:.3f} qW:{data['q_w']:.3f}", flush=True)
                        else:
                            print(f"[IMU RAW] Ax:{data['accel_x']:.2f} Gx:{data['gyro_x']:.2f}", flush=True)

                elif input_id == "gps":
                    gps_count += 1
                    bearing = data.get('bearing', 0.0)
                    b_acc = data.get('bearing_acc', 0.0)
                    print(f"[GPS] Lat:{data['lat']:.5f} Lon:{data['lon']:.5f} Alt:{data['alt']:.1f}m Spd:{data['speed']:.1f}m/s | Heading:{bearing:.1f}° (±{b_acc:.1f}°)", flush=True)
                    
                # Calculate and print Hz every 1.0 seconds
                elapsed = current_time - start_time
                if elapsed >= 1.0:
                    imu_hz = imu_count / elapsed
                    gps_hz = gps_count / elapsed
                    print(f"--- FREQUENCY REPORT | IMU: {imu_hz:.1f} Hz | GPS: {gps_hz:.1f} Hz ---", flush=True)
                    
                    # Reset counters
                    start_time = current_time
                    imu_count = 0
                    gps_count = 0

            except Exception as e:
                print(f"Error parsing telemetry: {e}", flush=True)
                
        elif event["type"] == "STOP":
            break

    print("Telemetry Node shutting down.")

if __name__ == "__main__":
    main()
