import zenoh
import time

if __name__ == "__main__":
    print("Starting Zenoh Receiver...")

    # 1. Configure the laptop as a Peer listening on port 7447
    conf = zenoh.Config()
    conf.insert_json5("mode", '"peer"')
    conf.insert_json5("listen/endpoints", '["tcp/0.0.0.0:7447"]')

    # 2. Open the session
    session = zenoh.open(conf)

    # 3. Define the callback for incoming data
    def listener(sample):
        # FIX: Cast Zenoh's ZBytes to standard Python bytes, then decode
        raw_bytes = bytes(sample.payload)
        decoded_text = raw_bytes.decode('utf-8')
        print(f"SUCCESS: Received -> {decoded_text}")

    # 4. Subscribe to the test topic from your phone
    sub = session.declare_subscriber("phone/sensors/test", listener)

    print("Listening for the S23 on port 7447. Waiting for data...")
    
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("\nShutting down...")
        session.close()
