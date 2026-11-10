import zenoh
import json
import time
import sys

def main():
    print("Configuring Client Networking...", flush=True)
    config = zenoh.Config()
    # Target the 4090 Tailscale IP address directly over TCP
    config.insert_json5("connect/endpoints", '["tcp/100.73.159.102:7447"]')
    
    print("Opening Zenoh 1.0.0 Session Client...", flush=True)
    session = zenoh.open(config)
    
    # Define a complex mock SLAM frontier puzzle for your 12B model
    mock_slam_state = (
        "Robot at dead end split. Path_Left enters a cluttered maintenance bay with high dynamic obstacle risk. "
        "Path_Right remains on main hallway grid mapping vector but shows structural drop-offs."
    )
    available_vectors = ["Path_Left", "Path_Right", "halt_and_wait"]
    
    payload = {
        "state": mock_slam_state,
        "choices": available_vectors
    }
    
    print("\n[Client] Querying 12B Model Server over Tailscale P2P tunnel...", flush=True)
    start_time = time.time()
    
    # In Zenoh 1.0.0, we query data passing explicit json payloads via the query payload argument
    replies = session.get("lab/robot/mission", payload=json.dumps(payload))
    
    for reply in replies:
        if reply.ok is not None:
            latency = (time.time() - start_time) * 1000
            # Read output string out of the incoming payload container
            decision_json = reply.ok.payload.to_string()
            print(f"\n[Client] Success! Response fetched in {latency:.2f} ms!")
            print(f"[Client] 12B Decision Vector Outcome -> {decision_json}")
        else:
            print(f"[Client] Network Error: {reply.err.payload.to_string()}")
            
    session.close()

if __name__ == "__main__":
    main()
