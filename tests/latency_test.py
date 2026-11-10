import zenoh
import json
import time
import statistics
import sys

def main():
    config = zenoh.Config()
    config.insert_json5("connect/endpoints", '["tcp/100.73.159.102:7447"]')
    session = zenoh.open(config)

    payload = {
        "state": "Frontier exploration tick: hallway corridor open ahead, walls on both sides.",
        "choices": {
            "move_forward": "Continue along current corridor heading.",
            "halt": "Stop and await revised map."
        }
    }
    encoded_payload = json.dumps(payload)

    print("=" * 60)
    print("      STRANDS-12B + ZENOH LATENCY & JITTER BENCHMARK       ")
    print("=" * 60)

    # 1. Warm-up runs (discard initial CUDA allocation spikes)
    print("[*] Sending 3 warmup queries to establish GPU cache...", flush=True)
    for _ in range(3):
        replies = session.get("lab/robot/mission", payload=encoded_payload)
        for _ in replies:
            pass

    # 2. Benchmark Loop
    total_samples = 50
    latencies = []
    print(f"[*] Benchmarking {total_samples} consecutive inference cycles...\n", flush=True)

    for i in range(1, total_samples + 1):
        t0 = time.perf_counter()
        replies = session.get("lab/robot/mission", payload=encoded_payload)
        
        got_reply = False
        for reply in replies:
            if reply.ok is not None:
                got_reply = True
                break
        
        dt_ms = (time.perf_counter() - t0) * 1000.0
        if got_reply:
            latencies.append(dt_ms)
            sys.stdout.write(f"\r  Iter {i:02d}/{total_samples} | Last: {dt_ms:6.2f} ms")
            sys.stdout.flush()
        else:
            print(f"\n[!] Dropped packet on iteration {i}")

    session.close()

    if not latencies:
        print("\nNo valid responses received.")
        return

    # 3. Statistics
    latencies.sort()
    mean_lat = statistics.mean(latencies)
    median_lat = statistics.median(latencies)
    std_dev = statistics.stdev(latencies) if len(latencies) > 1 else 0.0
    p95_lat = latencies[int(len(latencies) * 0.95) - 1]
    p99_lat = latencies[int(len(latencies) * 0.99) - 1]
    min_lat = min(latencies)
    max_lat = max(latencies)

    print("\n\n" + "-" * 60)
    print(f"  Samples Tested   : {len(latencies)} / {total_samples}")
    print(f"  Minimum Latency  : {min_lat:6.2f} ms")
    print(f"  Median (P50)     : {median_lat:6.2f} ms")
    print(f"  Average (Mean)   : {mean_lat:6.2f} ms")
    print(f"  95th Percentile  : {p95_lat:6.2f} ms")
    print(f"  99th Percentile  : {p99_lat:6.2f} ms")
    print(f"  Maximum Latency  : {max_lat:6.2f} ms")
    print(f"  Jitter (Std Dev) : ±{std_dev:5.2f} ms")
    print("-" * 60)

if __name__ == "__main__":
    main()
