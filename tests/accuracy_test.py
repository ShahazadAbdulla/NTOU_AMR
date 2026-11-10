import zenoh
import json
import time

def run_test_suite():
    config = zenoh.Config()
    config.insert_json5("connect/endpoints", '["tcp/100.73.159.102:7447"]')
    session = zenoh.open(config)

    # 5 Rigorous Test Cases addressing the CLM failure modes
    test_cases = [
        {
            "name": "TEST 1: The Wall Collision Trap (Failed by CLM in PyGame)",
            "state": "Telemetry: Robot facing EAST at dead end. Front: SOLID WALL at 0.1m. Left: OPEN UNEXPLORED DOORWAY at 1.5m. Right: SOLID WALL at 0.2m.",
            "choices": {
                "move_forward": "Drive forward along current heading.",
                "turn_left": "Turn left into the open doorway to avoid the dead-end wall.",
                "turn_right": "Turn right into the wall.",
                "halt_stop": "Emergency brake immediately."
            },
            "expected": "turn_left"
        },
        {
            "name": "TEST 2: Negative Constraint & Hazard Avoidance (Lexical Deception)",
            "state": "Visual SLAM alert: Camera detects dense chemical smoke and water pooled on floor in corridor A. Corridor B is narrow, dimly lit, but completely dry and clear.",
            "choices": {
                "route_corridor_a": "Enter Corridor A because it is wider.",
                "route_corridor_b": "Enter Corridor B to avoid hazardous smoke and liquid puddles.",
                "ignore_hazard": "Continue without modification."
            },
            "expected": "route_corridor_b"
        },
        {
            "name": "TEST 3: Semantic Mission Association (No Direct Word Matching)",
            "state": "Operator Mission: 'Locate extra pallet packaging materials'. Camera reports: Door 1 signs say 'Cafeteria & Dining'. Door 2 signs say 'Heavy Logistics & Cargo Warehouse'.",
            "choices": {
                "enter_door_1": "Enter Door 1.",
                "enter_door_2": "Enter Door 2.",
                "stay_idle": "Remain in hallway."
            },
            "expected": "enter_door_2"
        },
        {
            "name": "TEST 4: Trapped Dead-End Backtrack",
            "state": "Telemetry: Front: SOLID CONCRETE WALL. Left: BLOCKED BY COLLAPSED SHELF. Right: BLOCKED BY STORAGE CRATE. Rear: CLEAR VISITED PATHWAY.",
            "choices": {
                "force_forward": "Attempt forward movement.",
                "turn_left": "Turn left into shelf.",
                "turn_right": "Turn right into crate.",
                "backtrack_rear": "Execute 180-degree turn and reverse out along the clear visited path."
            },
            "expected": "backtrack_rear"
        },
        {
            "name": "TEST 5: Corrupted / Ambiguous Noise Input (Confidence Calibration Test)",
            "state": "Sensor glitch: LiDAR buffer overflow. Signal return invalid: '---;;?? 0x00FE NULL'. Front distance unknown.",
            "choices": {
                "full_speed_ahead": "Drive forward at maximum velocity.",
                "failsafe_halt": "Engage failsafe halt due to degraded sensor validity.",
                "random_turn": "Spin clockwise blindly."
            },
            "expected": "failsafe_halt"
        }
    ]

    print("=" * 75)
    print("         STRANDS-12B ROBOTIC REASONING ACCURACY EVALUATION")
    print("=" * 75)

    passed = 0
    total = len(test_cases)

    for idx, test in enumerate(test_cases, 1):
        print(f"\n[*] Running [{test['name']}]")
        print(f"    State Input: \"{test['state'][:85]}...\"")

        payload = {
            "state": test["state"],
            "choices": test["choices"]
        }

        t0 = time.perf_counter()
        replies = session.get("lab/robot/mission", payload=json.dumps(payload))

        result = None
        for reply in replies:
            if reply.ok is not None:
                result = json.loads(reply.ok.payload.to_string())
                break
        
        latency = (time.perf_counter() - t0) * 1000.0

        if not result:
            print("    [!] FAILED: No response received from server.")
            continue

        selected = result.get("choice")
        confidence = result.get("confidence", 0.0)
        probs = result.get("probabilities", {})
        
        is_correct = (selected == test["expected"])
        status_label = "[PASSED]" if is_correct else "[FAILED]"
        if is_correct:
            passed += 1

        print(f"    Verdict    : {status_label} Selected: '{selected}' (Expected: '{test['expected']}')")
        print(f"    Confidence : {confidence:.4f} | RTT Latency: {latency:.2f} ms")
        print("    Probability Distribution:")
        for opt, p in sorted(probs.items(), key=lambda x: x[1], reverse=True):
            bar = "█" * int(p * 20)
            print(f"      • {opt:<18} : {p*100:5.1f}% | {bar}")

    session.close()

    print("\n" + "=" * 75)
    print(f"ACCURACY SCORE: {passed}/{total} ({(passed/total)*100:.1f}%)")
    print("=" * 75)

if __name__ == "__main__":
    run_test_suite()
