import cv2
import numpy as np
import time
import json
from dora import Node

def main():
    node = Node()
    print("Project ZeRo - Analytics Viewer Active", flush=True)

    window_name = "Project ZeRo - S23 Main Wide"
    cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
    
    # Telemetry Counters
    valid_frames = 0
    corrupted_frames = 0
    seq_drops = 0
    last_seq = -1
    
    # Instant FPS window
    start_time = time.time()
    window_frames = 0
    
    for event in node:
        if event["type"] == "INPUT" and event["id"] == "camera":
            try:
                # 1. Read Metadata for Telemetry
                metadata = event.get("metadata", {})
                zenoh_meta_str = metadata.get("zenoh_meta", "{}")
                zenoh_meta = json.loads(zenoh_meta_str)
                
                current_seq = zenoh_meta.get("seq", -1)
                
                # Detect missing packets (Sequence jumps)
                if last_seq != -1 and current_seq > (last_seq + 1):
                    seq_drops += (current_seq - last_seq - 1)
                last_seq = current_seq

                # 2. Decode the Image
                arrow_array = event["value"]
                jpeg_buffer = np.frombuffer(arrow_array.to_numpy().tobytes(), dtype=np.uint8)
                frame = cv2.imdecode(jpeg_buffer, cv2.IMREAD_COLOR)
                
                if frame is not None:
                    valid_frames += 1
                    window_frames += 1
                    cv2.imshow(window_name, frame)
                else:
                    corrupted_frames += 1

                # 3. Print Telemetry Dashboard every 1.0 second
                current_time = time.time()
                elapsed = current_time - start_time
                if elapsed >= 1.0:
                    instant_fps = window_frames / elapsed
                    
                    total_received = valid_frames + corrupted_frames
                    corruption_pct = (corrupted_frames / total_received) * 100 if total_received > 0 else 0
                    
                    print(f"| INSTANT FPS: {instant_fps:.1f} | VALID: {valid_frames} | CORRUPT JPEGs: {corrupted_frames} ({corruption_pct:.1f}%) | NETWORK DROPS: {seq_drops} |", flush=True)
                    
                    # Reset window for next second
                    start_time = current_time
                    window_frames = 0

            except Exception as e:
                print(f"Decoding error: {e}", flush=True)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
                
        elif event["type"] == "STOP":
            break

    cv2.destroyAllWindows()
    print("Viewer Node shutting down.")

if __name__ == "__main__":
    main()
