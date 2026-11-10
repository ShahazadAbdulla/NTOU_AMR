import base64
import json
import time
import urllib.request
import zenoh
import io
from PIL import Image, ImageDraw

ZENOH_ENDPOINT = "tcp/100.73.159.102:7447"

# 1. Define our mixed batch of Image and Text scenarios
RAW_SCENARIOS = [
    # --- BLOCK 1: IMAGES ---
    {"type": "image", "url": "https://raw.githubusercontent.com/pytorch/hub/master/images/dog.jpg", "name": "Image 1: Dog", "state": "What animal is this?", "choices": ["dog", "cat", "bird"]},
    {"type": "image", "url": "https://raw.githubusercontent.com/ultralytics/yolov5/master/data/images/bus.jpg", "name": "Image 2: Bus", "state": "What vehicle is this?", "choices": ["bus", "car", "train"]},
    {"type": "image", "url": "https://raw.githubusercontent.com/ultralytics/yolov5/master/data/images/zidane.jpg", "name": "Image 3: People", "state": "Are there people here?", "choices": ["yes_people", "no_people"]},
    {"type": "image", "url": "https://raw.githubusercontent.com/pjreddie/darknet/master/data/kite.jpg", "name": "Image 4: Kites", "state": "What is in the sky?", "choices": ["kites", "airplanes", "birds"]},
    {"type": "image", "url": "https://raw.githubusercontent.com/pjreddie/darknet/master/data/eagle.jpg", "name": "Image 5: Eagle", "state": "What bird is this?", "choices": ["eagle", "pigeon", "duck"]},
    
    # --- BLOCK 2: TEXT ONLY ---
    {"type": "text", "name": "Text 1: Wall Ahead", "state": "My lidar detects a solid wall 0.5 meters directly in front of me.", "choices": ["stop", "drive_forward", "turn_around"]},
    {"type": "text", "name": "Text 2: Clear Path", "state": "Sensors indicate a clear hallway for the next 10 meters.", "choices": ["stop", "drive_forward"]},
    
    # --- BLOCK 3: GENERATED IMAGES (Guaranteed to exist) ---
    {"type": "gen_image", "color": "red", "name": "Image 6: Red Block", "state": "What color is the main object?", "choices": ["red", "blue", "green"]},
    {"type": "gen_image", "color": "blue", "name": "Image 7: Blue Block", "state": "What color is the main object?", "choices": ["red", "blue", "green"]},
    {"type": "gen_image", "color": "green", "name": "Image 8: Green Block", "state": "What color is the main object?", "choices": ["red", "blue", "green"]},
    {"type": "gen_image", "color": "yellow", "name": "Image 9: Yellow Block", "state": "Is this a hazard?", "choices": ["yes_hazard", "safe"]},
    {"type": "gen_image", "color": "black", "name": "Image 10: Pitch Black", "state": "Is the camera blocked?", "choices": ["camera_blocked", "camera_clear"]},

    # --- BLOCK 4: TEXT ONLY ---
    {"type": "text", "name": "Text 3: Logic Check", "state": "The traffic light is red.", "choices": ["stop", "go"]}
]

def generate_test_image(color):
    """Generates a solid color block image in memory."""
    img = Image.new('RGB', (400, 400), color=color)
    buf = io.BytesIO()
    img.save(buf, format='JPEG')
    return base64.b64encode(buf.getvalue()).decode('utf-8')

def prepare_payloads():
    print("[*] Phase 1: Pre-downloading and encoding all assets (Timer OFF)...")
    valid_payloads = []
    
    for idx, sc in enumerate(RAW_SCENARIOS):
        images = []
        if sc["type"] == "image":
            try:
                req = urllib.request.Request(sc['url'], headers={'User-Agent': 'Mozilla/5.0'})
                with urllib.request.urlopen(req, timeout=5) as response:
                    images = [base64.b64encode(response.read()).decode("utf-8")]
            except Exception as e:
                print(f"  [!] Skipping {sc['name']} - Download failed: {e}")
                continue
        elif sc["type"] == "gen_image":
            images = [generate_test_image(sc["color"])]
        
        # Build the final dict
        valid_payloads.append({
            "name": sc["name"],
            "payload": {
                "state": sc["state"],
                "images": images,
                "choices": sc["choices"]
            }
        })
        print(f"  [+] Prepared: {sc['name']}")
        
    return valid_payloads

def run_benchmark():
    payloads = prepare_payloads()
    
    print(f"\n[*] Phase 2: Connecting to Zenoh ({ZENOH_ENDPOINT})...")
    config = zenoh.Config()
    config.insert_json5("connect/endpoints", f'["{ZENOH_ENDPOINT}"]')
    session = zenoh.open(config)
    
    print("\n" + "="*50)
    print("🚀 STARTING PURE LATENCY BENCHMARK")
    print("="*50)

    for item in payloads:
        name = item["name"]
        pl = item["payload"]
        
        print(f"\nExecuting: {name}")
        
        # TIMING STARTS NOW (Only measuring network transmission + model inference)
        t0 = time.perf_counter()
        replies = session.get("lab/robot/mission", payload=json.dumps(pl), timeout=15.0)
        
        for reply in replies:
            if reply.ok is not None:
                latency = (time.perf_counter() - t0) * 1000
                data = json.loads(reply.ok.payload.to_string())
                
                print(f"  -> Latency:    {latency:.1f} ms")
                print(f"  -> Verdict:    {data['choice']} (Conf: {data['confidence']:.2f})")
            elif reply.err is not None:
                print(f"  [!] ERROR: {reply.err.payload.to_string()}")
        
        # Brief pause so we don't flood the Tailscale buffer
        time.sleep(0.5)

    session.close()
    print("\n[*] Benchmark Complete.")

if __name__ == "__main__":
    run_benchmark()
