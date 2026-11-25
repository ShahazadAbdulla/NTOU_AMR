import zenoh
import pyarrow as pa
import numpy as np
import struct
import json
from dora import Node

def main():
    node = Node()

    conf = zenoh.Config()
    conf.insert_json5("mode", '"peer"')
    conf.insert_json5("listen/endpoints", '["udp/0.0.0.0:7447", "tcp/0.0.0.0:7447"]')
    conf.insert_json5("scouting/multicast/enabled", "false")
    
    session = zenoh.open(conf)
    print("Ingestion Node: Listening on UDP & TCP port 7447 (Multicast Disabled)...", flush=True)

    # --- CAMERA CALLBACK ---
    def camera_listener(sample: zenoh.Sample):
        try:
            payload = bytes(sample.payload)
            arrow_array = pa.array(np.frombuffer(payload, dtype=np.uint8))
            
            meta_dict = {}
            if sample.attachment is not None:
                meta_dict = {"zenoh_meta": bytes(sample.attachment).decode('utf-8')}
                
            node.send_output("camera_data", arrow_array, meta_dict)
        except Exception as e:
            print(f"Camera Ingestion error: {e}", flush=True)

    # --- IMU CALLBACK (200Hz) ---
    def imu_listener(sample: zenoh.Sample):
        try:
            payload = bytes(sample.payload)
            if len(payload) == 33:
                # Unpack: int64 (8) + int8 (1) + 6x float32 (24) = 33 bytes
                unpacked = struct.unpack('<qB6f', payload)
                
                timestamp = unpacked[0]
                mode = unpacked[1]
                
                if mode == 0:
                    # Fused Quaternion Mode
                    data = {
                        "ts": timestamp, "mode": "FUSED",
                        "q_x": unpacked[2], "q_y": unpacked[3], "q_z": unpacked[4], "q_w": unpacked[5]
                    }
                else:
                    # Raw Accel + Gyro Mode
                    data = {
                        "ts": timestamp, "mode": "RAW",
                        "accel_x": unpacked[2], "accel_y": unpacked[3], "accel_z": unpacked[4],
                        "gyro_x": unpacked[5], "gyro_y": unpacked[6], "gyro_z": unpacked[7]
                    }
                
                # Convert the dictionary to a JSON string and send via PyArrow
                arrow_array = pa.array([json.dumps(data)])
                node.send_output("imu_data", arrow_array)
        except Exception as e:
            print(f"IMU Ingestion error: {e}", flush=True)
    # --- GPS CALLBACK (15Hz) ---
    def gps_listener(sample: zenoh.Sample):
        try:
            payload = bytes(sample.payload)
            if len(payload) == 48:
                # Unpack: int64 (8) + 3x float64 (24) + 4x float32 (16) = 48 bytes
                unpacked = struct.unpack('<q3d4f', payload)
                
                data = {
                    "ts": unpacked[0],
                    "lat": unpacked[1],
                    "lon": unpacked[2],
                    "alt": unpacked[3],
                    "speed": unpacked[4],
                    "accuracy": unpacked[5],
                    "bearing": unpacked[6],
                    "bearing_acc": unpacked[7]
                }
                
                arrow_array = pa.array([json.dumps(data)])
                node.send_output("gps_data", arrow_array)
        except Exception as e:
            print(f"GPS Ingestion error: {e}", flush=True)

    # Wire up the Zenoh subscribers
    sub_cam = session.declare_subscriber("phone/sensors/camera", camera_listener)
    sub_imu = session.declare_subscriber("phone/sensors/imu", imu_listener)
    sub_gps = session.declare_subscriber("phone/sensors/gps", gps_listener)

    for event in node:
        if event["type"] == "STOP":
            break

    session.close()
    print("Ingestion Node shutting down.")

if __name__ == "__main__":
    main()
