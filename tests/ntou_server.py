import sys
import json
import time
import torch
import zenoh
from strands_decider.infer import load_engine

DEVICE = "cuda"
MODEL_ID = "StrandsAgents/strands-decider-12B-gemma4-v1-2610"

print("--> Booting Strands-12B Server Configuration...", flush=True)

# 1. Initialize and cache the 12B Pointer Head onto the RTX 4090
print(f"Loading engine and caching model weights onto [{DEVICE.upper()}]...", flush=True)
engine = load_engine(MODEL_ID, device=DEVICE)

def handle_robot_query(query):
    try:
        raw_payload = query.payload.to_string()
        payload = json.loads(raw_payload)
        
        state = payload["state"]
        choices = payload["choices"]

        print(f"\n[Incoming Packet] State Scenario: {state}", flush=True)
        print(f"[Incoming Packet] Evaluated Vectors: {choices}", flush=True)

        # Map choices into the verified dictionary layout standard
        criteria_map = {choice: f"Path option: {choice}" for choice in choices}

        # 2. Compute local tensor probabilities on GPU using the verified signature
        response = engine.ask(
            state=state,
            questions={
                "next_move": {
                    "type": "choice",
                    "instructions": "Next Move?",
                    "criteria": criteria_map
                }
            }
        )

        # 3. Extract the underlying ChoiceAnswer dataclass fields cleanly
        answer_block = response.answers.get("next_move")
        result = {
            "choice": answer_block.choice,
            "probabilities": answer_block.probabilities,
            "confidence": answer_block.confidence
        }

        # 4. Push JSON results back cleanly matching the calling expression
        query.reply(query.key_expr, json.dumps(result))
        print(f"[Verdict Dispatched] Weights sent back down wire: {result}", flush=True)

    except Exception as e:
        print(f"Error handling network telemetry payload: {e}", flush=True)

def main():
    config = zenoh.Config()
    # Explicitly bind to port 7447 over Tailscale adapter
    config.insert_json5("listen/endpoints", '["tcp/0.0.0.0:7447"]')
    
    print("Opening Zenoh 1.0.0 Server Routing Engine...", flush=True)
    session = zenoh.open(config)
    
    queryable = session.declare_queryable("lab/robot/mission", handle_robot_query)
    print("Strands Arbiter Node is live and awaiting explorer inputs...", flush=True)
    
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("\nShutting down server node.", flush=True)
        session.close()

if __name__ == "__main__":
    main()
