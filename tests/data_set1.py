import base64
import json
import time
import urllib.request
import zenoh
import csv
import os

ZENOH_ENDPOINT = "tcp/100.73.159.102:7447"
CONFIDENCE_THRESHOLD = 0.85 

# Updated Dataset targeting your specific classes
DATASET = [
    {"id": "img_001_dog", "url": "https://raw.githubusercontent.com/pytorch/hub/master/images/dog.jpg"},
    {"id": "img_002_kite", "url": "https://raw.githubusercontent.com/pjreddie/darknet/master/data/kite.jpg"},
    {"id": "img_003_tree", "url": "https://raw.githubusercontent.com/ultralytics/yolov5/master/data/images/bus.jpg"}, # Has trees in background
    {"id": "img_004_ambiguous", "url": "https://raw.githubusercontent.com/pjreddie/darknet/master/data/scream.jpg"} 
]

# The new specific taxonomy
TAXONOMY = {
    "dog": "The primary subject is a dog.",
    "kite": "The primary subject is a kite flying in the sky.",
    "tree": "The primary subject is a tree or forest.",
    "none": "The primary subject is none of these."
}

def run_labeling_pipeline():
    print(f"[*] Initializing Edge Labeling Node...")
    config = zenoh.Config()
    config.insert_json5("connect/endpoints", f'["{ZENOH_ENDPOINT}"]')
    session = zenoh.open(config)
    
    csv_filename = "auto_labeled_dataset.csv"
    file_exists = os.path.isfile(csv_filename)
    
    with open(csv_filename, mode='a', newline='') as csv_file:
        writer = csv.writer(csv_file)
        if not file_exists:
            writer.writerow(["image_id", "auto_label", "confidence", "status", "latency_ms"])
            
        print("\n[*] Starting High-Throughput System 1 Annotation...")
        print("-" * 60)

        for item in DATASET:
            try:
                req = urllib.request.Request(item['url'], headers={'User-Agent': 'Mozilla/5.0'})
                with urllib.request.urlopen(req, timeout=5) as response:
                    img_b64 = base64.b64encode(response.read()).decode("utf-8")
            except Exception as e:
                print(f"[!] Failed to load {item['id']}: {e}")
                continue

            payload = {
                "state": "Analyze the image and identify the primary subject.",
                "images": [img_b64],
                "choices": TAXONOMY
            }

            t0 = time.perf_counter()
            replies = session.get("lab/robot/mission", payload=json.dumps(payload), timeout=15.0)
            
            for reply in replies:
                if reply.ok is not None:
                    latency = (time.perf_counter() - t0) * 1000
                    data = json.loads(reply.ok.payload.to_string())
                    
                    chosen = data['choice']
                    conf = data['confidence']
                    
                    if conf >= CONFIDENCE_THRESHOLD:
                        status = "AUTO_VERIFIED"
                    else:
                        status = "NEEDS_HUMAN_REVIEW"
                        chosen = "UNKNOWN"
                    
                    print(f"[{item['id']}] -> {chosen.upper()} (Conf: {conf:.2f}) | Status: {status} | Latency: {latency:.1f}ms")
                    writer.writerow([item['id'], chosen, f"{conf:.3f}", status, f"{latency:.1f}"])

    session.close()
    print("-" * 60)
    print(f"[*] Pipeline Complete. Results saved to {csv_filename}")

if __name__ == "__main__":
    run_labeling_pipeline()
