import base64
import json
import zenoh
from PIL import Image, ImageDraw

# 1. Generate a mock camera frame (a red circle on a gray background)
print("1. Generating test image (Red Circle)...")
img = Image.new('RGB', (400, 400), color=(50, 50, 50))
draw = ImageDraw.Draw(img)
draw.ellipse([(100, 100), (300, 300)], fill=(239, 68, 68)) # Draw red circle
img.save("test_camera_frame.jpg")

# 2. Encode the image to Base64
with open("test_camera_frame.jpg", "rb") as img_file:
    img_b64 = base64.b64encode(img_file.read()).decode("utf-8")

# 3. Connect to Zenoh
print("2. Connecting to Zenoh Network...")
config = zenoh.Config()
# Ensure this targets your 4090's Tailscale IP
config.insert_json5("connect/endpoints", '["tcp/100.73.159.102:7447"]')
session = zenoh.open(config)

# 4. Build the Multimodal Payload
payload = {
    "state": "System Diagnostic. Look at the attached camera frame.",
    "images": [img_b64], # Injecting the raw image
    "choices": {
        "hazard_red_circle": "I see a red circle. This is a hazard.",
        "safe_blue_square": "I see a blue square. This is safe.",
        "empty_hallway": "I see nothing but gray walls."
    }
}

print("3. Firing Multimodal Payload to RTX 4090...")
replies = session.get("lab/robot/mission", payload=json.dumps(payload), timeout=15.0)

for reply in replies:
    if reply.ok is not None:
        print("\n[+] 4090 ARBITER VERDICT:")
        print(reply.ok.payload.to_string())
    elif reply.err is not None:
        print(f"\n[!] ERROR: {reply.err.payload.to_string()}")

session.close()
