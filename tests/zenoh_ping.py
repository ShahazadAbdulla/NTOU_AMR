import zenoh
import time
import sys

def main():
    print("Configuring Zenoh 1.0.0 Target Client...", flush=True)
    config = zenoh.Config()
    
    # Explicitly tell your laptop to connect straight to the 4090 over Tailscale
    config.insert_json5("connect/endpoints", '["tcp/100.73.159.102:7447"]')
    
    print("Opening Laptop Zenoh Client Connection...", flush=True)
    session = zenoh.open(config)

    key_expression = "ntou/test/tunnel"
    message_data = "Hello from Ubuntu Laptop over Tailscale!"

    print(f"Publishing data payload to '{key_expression}'...", flush=True)
    session.put(key_expression, message_data)

    print("Holding frame open to flush network cache buffers...", flush=True)
    time.sleep(2)

    print("Closing session cleanly.", flush=True)
    session.close()

if __name__ == "__main__":
    main()
