import zenoh
import json
import time
import sys

def main():
    config = zenoh.Config()
    config.insert_json5("connect/endpoints", '["tcp/100.73.159.102:7447"]')
    session = zenoh.open(config)

    # 1. SMALL PAYLOAD (Baseline)
    payload_small = {
        "state": "Frontier exploration tick: hallway corridor open ahead.",
        "choices": {
            "move_forward": "Continue along current corridor heading.",
            "halt": "Stop and await revised map."
        }
    }

    # 2. MEDIUM PAYLOAD (PyGame Maze Scale)
    payload_medium = {
        "state": "Pose: (1, 5) facing EAST. Front: UNEXPLORED OPENING. Left: UNEXPLORED OPENING. Right: WALL. Previous action: none.",
        "choices": {
            "move_forward": "The Front is an UNEXPLORED OPENING or OPEN space. Drive straight.",
            "explore_left_door": "The Front is OPEN, but there is an UNEXPLORED OPENING on the Left. Go left.",
            "explore_right_door": "The Front is OPEN, but there is an UNEXPLORED OPENING on the Right. Go right.",
            "turn_left": "The Front is a WALL, but the Left is an UNEXPLORED OPENING. Turn left to avoid crash.",
            "turn_right": "The Front is a WALL, but the Right is an UNEXPLORED OPENING. Turn right to avoid crash.",
            "turn_around": "The Front is a WALL, the Left is a WALL, and the Right is a WALL. Dead end."
        }
    }

    # 3. LARGE PAYLOAD (Real Semantic SLAM / Vision Payload)
    payload_large = {
        "state": (
            "Mission: Locate tool storage. "
            "Odometry: X=12.4, Y=-3.2, Theta=1.57. "
            "LiDAR: Clear path to 5.0m at 0 deg. Walls at -90 and 90 deg. "
            "Camera Detections: "
            "[AprilTag_14: dist=2.1m, angle=15deg], "
            "[YOLO_brown_box: dist=3.0m, angle=-10deg], "
            "[YOLO_red_stool: dist=1.5m, angle=45deg], "
            "[OCR_Text: 'MAINTENANCE BAY B' at dist=4.0m, angle=0deg]. "
            "Battery: 42%. Nav2 Status: Idle."
        ),
        "choices": {
            "approach_apriltag_14": "Set Nav2 goal to AprilTag_14 coordinate for closer inspection.",
            "approach_brown_box": "Set Nav2 goal to brown box coordinate.",
            "approach_red_stool": "Set Nav2 goal to red stool coordinate.",
            "enter_maintenance_bay": "Set Nav2 goal to OCR text 'MAINTENANCE BAY B'.",
            "continue_hallway_scan": "Ignore objects and continue mapping the main corridor.",
            "return_to_dock": "Battery below 50%. Abort mission and return to charging station.",
            "halt_and_query_operator": "Uncertain state. Stop motors and request human intervention.",
            "recalibrate_sensors": "Spin 360 degrees to clear LiDAR costmap artifacts."
        }
    }

    tests = [
        ("SMALL", payload_small),
        ("MEDIUM", payload_medium),
        ("LARGE", payload_large)
    ]

    print("=" * 70)
    print("      STRANDS-12B ZENOH PAYLOAD SCALABILITY STRESS TEST")
    print("=" * 70)

    for name, payload in tests:
        encoded = json.dumps(payload)
        byte_size = len(encoded.encode('utf-8'))
        print(f"\n[*] Testing {name} Payload ({byte_size} bytes, {len(payload['choices'])} choices)")
        
        # Warmup
        _ = session.get("lab/robot/mission", payload=encoded, timeout=10.0)
        
        latencies = []
        for i in range(1, 6):
            t0 = time.perf_counter()
            replies = session.get("lab/robot/mission", payload=encoded, timeout=15.0)
            
            success = False
            for reply in replies:
                if reply.ok is not None:
                    success = True
                    break
            
            dt = (time.perf_counter() - t0) * 1000
            if success:
                latencies.append(dt)
                sys.stdout.write(f"\r  Iter {i}/5 | Latency: {dt:7.2f} ms")
                sys.stdout.flush()
            else:
                sys.stdout.write(f"\r  Iter {i}/5 | [!] TIMEOUT/ERROR (>15s)")
                sys.stdout.flush()
        
        if latencies:
            avg = sum(latencies) / len(latencies)
            print(f"\n  -> {name} Average Latency: {avg:.2f} ms")

    session.close()

if __name__ == "__main__":
    main()
