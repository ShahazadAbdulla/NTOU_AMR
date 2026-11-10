import base64
import json
import time
import urllib.request
import zenoh

# Target your 4090's Tailscale IP
ZENOH_ENDPOINT = "tcp/100.73.159.102:7447"

TEST_CASES = [
    {
        "name": "Animal Identification (Dog)",
        "url": "https://raw.githubusercontent.com/pytorch/hub/master/images/dog.jpg",
        "state": "Look closely at the animal in the image.",
        "choices": {
            "dog": "It is a dog.",
            "cat": "It is a cat.",
            "bird": "It is a bird."
        }
    },
    {
        "name": "Vehicle Identification (Bus)",
        "url": "https://raw.githubusercontent.com/ultralytics/yolov5/master/data/images/bus.jpg",
        "state": "Identify the primary vehicle in this scene.",
        "choices": {
            "bus": "A large passenger bus.",
            "car": "A small sports car.",
            "train": "A high-speed train."
        }
    },
    {
        "name": "Environment (Intersection)",
        "url": "https://raw.githubusercontent.com/ultralytics/yolov5/master/data/images/intersection.jpg",
        "state": "Describe the environment shown in the camera feed.",
        "choices": {
            "urban_intersection": "A busy urban street intersection.",
            "empty_highway": "An empty desert highway.",
            "forest_trail": "A dirt path in a forest."
        }
    }
]

def run_evaluation():
    print(f"[*] Connecting to Zenoh Network at {ZENOH_ENDPOINT}...")
    config = zenoh.Config()
    config.insert_json5("connect/endpoints", f'["{ZENOH_ENDPOINT}"]')
    session = zenoh.open(config)
    
    print("[*] Starting Automated VLM Benchmark...\n" + "-"*50)

    for i, test in enumerate(TEST_CASES, 1):
        print(f"Test {i}/{len(TEST_CASES)}: {test['name']}")
        
        try:
            req = urllib.request.Request(test['url'], headers={'User-Agent': 'Mozilla/5.0'})
            with urllib.request.urlopen(req) as response:
                img_data = response.read()
                img_b64 = base64.b64encode(img_data).decode("utf-8")
        except Exception as e:
            print(f"[!] Failed to download image: {e}\n")
            continue

        payload = {
            "state": test["state"],
            "images": [img_b64],
            "choices": test["choices"]
        }

        t0 = time.perf_counter()
        replies = session.get("lab/robot/mission", payload=json.dumps(payload), timeout=15.0)
        
        for reply in replies:
            if reply.ok is not None:
                latency = (time.perf_counter() - t0) * 1000
                data = json.loads(reply.ok.payload.to_string())
                
                print(f"  -> Latency:    {latency:.1f} ms")
                print(f"  -> Verdict:    {data['choice']}")
                print(f"  -> Confidence: {data['confidence']:.3f}")
                
                print("  -> Breakdown:")
                for choice, prob in data['probabilities'].items():
                    print(f"       {choice}: {prob*100:.1f}%")
            elif reply.err is not None:
                print(f"[!] ERROR: {reply.err.payload.to_string()}")
        
        print("-" * 50)
        time.sleep(1)

    session.close()
    print("[*] Benchmark Complete.")

if __name__ == "__main__":
    run_evaluation()
