import pygame
import time
import sys
import zenoh
import json
import math
import multiprocessing as mp

# ---------------------------------------------------------
# MULTIPROCESSING ZENOH WORKER
# ---------------------------------------------------------
def strands_worker(query_q, result_q):
    config = zenoh.Config()
    config.insert_json5("connect/endpoints", '["tcp/100.73.159.102:7447"]')
    session = zenoh.open(config)

    while True:
        try:
            payload = query_q.get(timeout=0.1)
            if payload is None: break
            
            t0 = time.perf_counter()
            replies = session.get("lab/robot/mission", payload=json.dumps(payload), timeout=10.0)
            
            received = False
            for reply in replies:
                if reply.ok is not None:
                    raw = reply.ok.payload.to_string()
                    dt = (time.perf_counter() - t0) * 1000
                    result_q.put((json.loads(raw), dt))
                    received = True
            
            if not received:
                result_q.put(({"choice": "halt", "confidence": 0.0, "probabilities": {}}, 0))
                
        except mp.queues.Empty:
            continue
        except Exception:
            pass
    session.close()

# ---------------------------------------------------------
# GEOMETRY HELPERS
# ---------------------------------------------------------
def ray_intersect(p1, p2, p3, p4):
    x1, y1 = p1; x2, y2 = p2
    x3, y3 = p3; x4, y4 = p4
    den = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4)
    if den == 0: return None
    t = ((x1 - x3) * (y3 - y4) - (y1 - y3) * (x3 - x4)) / den
    u = -((x1 - x2) * (y1 - y3) - (y1 - y2) * (x1 - x3)) / den
    if 0 <= t <= 1 and 0 <= u <= 1:
        return (x1 + t * (x2 - x1), y1 + t * (y2 - y1))
    return None

def normalize_angle(angle):
    while angle > math.pi: angle -= 2 * math.pi
    while angle < -math.pi: angle += 2 * math.pi
    return angle

if __name__ == '__main__':
    print("=" * 70)
    print("   CONTINUOUS SEMANTIC ARBITRATION SIMULATOR (STRANDS-E4B)")
    print("=" * 70)
    print("Available environment entities:")
    print("  • [tool_cart]          : Blue cart in north corridor")
    print("  • [chemical_spill]    : Green hazardous puddle in south corridor")
    print("  • [door_maintenance]  : Yellow door (OCR: 'MAINTENANCE BAY')")
    print("  • [exit_corridor]     : White exit path")
    print("-" * 70)
    
    # Prompt the operator for a natural language mission
    default_mission = "Locate the maintenance bay door. Avoid chemical spills."
    user_mission = input(f"Enter mission prompt [Press Enter for default]: ").strip()
    current_mission = user_mission if user_mission else default_mission

    query_queue = mp.Queue()
    result_queue = mp.Queue()
    worker_proc = mp.Process(target=strands_worker, args=(query_queue, result_queue), daemon=True)
    worker_proc.start()

    pygame.init()
    WIDTH, HEIGHT = 1000, 700
    screen = pygame.display.set_mode((WIDTH, HEIGHT))
    pygame.display.set_caption("Semantic Arbiter Continuous Sim")
    font = pygame.font.SysFont("monospace", 13, bold=True)
    font_small = pygame.font.SysFont("monospace", 11)
    clock = pygame.time.Clock()

    robot_x, robot_y = 150.0, 350.0
    robot_theta = 0.0
    v, w = 0.0, 0.0
    MAX_V = 2.2
    MAX_W = 0.06

    walls = [
        ((50, 50), (950, 50)), ((950, 50), (950, 650)), 
        ((950, 650), (50, 650)), ((50, 650), (50, 50)),
        ((300, 50), (300, 250)), ((300, 450), (300, 650)),
        ((600, 200), (800, 200)), ((600, 200), (600, 400))
    ]

    objects = {
        "tool_cart": {"x": 450, "y": 150, "color": (59, 130, 246), "desc": "Blue mechanical tool cart."},
        "chemical_spill": {"x": 450, "y": 550, "color": (34, 197, 94), "desc": "Green hazardous puddle."},
        "door_maintenance": {"x": 850, "y": 100, "color": (234, 179, 8), "desc": "Yellow door labeled 'MAINTENANCE BAY'."},
        "exit_corridor": {"x": 100, "y": 100, "color": (255, 255, 255), "desc": "White corridor leading to facility exit."}
    }

    active_target = None
    completed_targets = set()
    awaiting_arbiter = False
    
    # State tracking
    is_scanning = False
    scan_ticks_remaining = 0
    last_decision_str = "None"
    
    running = True
    while running:
        clock.tick(60)
        screen.fill((25, 25, 28))

        # 1. LiDAR Raycasting
        lidar_dists = {"front": 1000, "left": 1000, "right": 1000}
        ray_angles = {"front": 0, "left": -math.pi/4, "right": math.pi/4}
        for key, offset in ray_angles.items():
            angle = robot_theta + offset
            end_x = robot_x + math.cos(angle) * 1000
            end_y = robot_y + math.sin(angle) * 1000
            min_dist = 1000
            hit_point = (end_x, end_y)
            for wall in walls:
                intersect = ray_intersect((robot_x, robot_y), (end_x, end_y), wall[0], wall[1])
                if intersect:
                    dist = math.hypot(intersect[0] - robot_x, intersect[1] - robot_y)
                    if dist < min_dist:
                        min_dist = dist
                        hit_point = intersect
            lidar_dists[key] = min_dist
            pygame.draw.line(screen, (239, 68, 68) if min_dist < 45 else (70, 70, 75), (robot_x, robot_y), hit_point, 1)

        # 2. Vision Cone
        fov_objects = {}
        for obj_id, obj in objects.items():
            if obj_id in completed_targets: continue
            dx = obj["x"] - robot_x
            dy = obj["y"] - robot_y
            dist = math.hypot(dx, dy)
            angle_to_obj = math.atan2(dy, dx)
            angle_diff = abs(normalize_angle(angle_to_obj - robot_theta))
            
            if angle_diff < math.pi/4 and dist < 320:
                los = True
                for wall in walls:
                    if ray_intersect((robot_x, robot_y), (obj["x"], obj["y"]), wall[0], wall[1]):
                        los = False; break
                if los: fov_objects[obj_id] = obj

        # 3. AI Arbitration Trigger
        if active_target is None and not awaiting_arbiter and not is_scanning:
            if fov_objects:
                state_str = f"Mission Directive: '{current_mission}'. Camera reports visible entities: "
                choices = {
                    "scan_around": "The visible items do not fulfill the mission. Rotate to scan for other paths.",
                    "halt": "Immediate danger. Stop all motion."
                }
                for fid, fobj in fov_objects.items():
                    state_str += f"[{fid}: {fobj['desc']}], "
                    choices[f"approach_{fid}"] = f"Drive to {fid} ({fobj['desc']})."

                print("\n" + "=" * 65)
                print(f"[*] ARBITRATION TRIGGERED")
                print(f"    State Prompt   : \"{state_str}\"")
                print(f"    Action Choices : {list(choices.keys())}")
                print("=" * 65)

                query_queue.put({"state": state_str, "choices": choices})
                awaiting_arbiter = True
                v, w = 0.0, 0.0
            else:
                # Default exploration cruise
                if lidar_dists["front"] > 70:
                    v = 1.4
                    w = 0.0
                else:
                    v = 0.0
                    w = MAX_W if lidar_dists["left"] > lidar_dists["right"] else -MAX_W

        # 4. Handle Decision
        while not result_queue.empty():
            result, latency = result_queue.get()
            choice = result.get("choice", "halt")
            conf = result.get("confidence", 0.0)
            probs = result.get("probabilities", {})
            last_decision_str = f"{choice.upper()} ({conf*100:.1f}%)"

            print(f"[Arbiter Verdict] >> {choice.upper()} << | Confidence: {conf*100:.1f}% | RTT: {latency:.1f} ms")
            print("  Probabilities:")
            for c_name, p_val in sorted(probs.items(), key=lambda x: x[1], reverse=True):
                print(f"    • {c_name:<24}: {p_val*100:5.1f}%")

            if choice.startswith("approach_"):
                target_id = choice.split("approach_")[1]
                if target_id in objects:
                    active_target = objects[target_id]
                    is_scanning = False
            elif choice == "scan_around":
                # Execute an actual rotational scan for 60 frames (~1 sec)
                is_scanning = True
                scan_ticks_remaining = 60
                active_target = None
            else:
                active_target = None
                is_scanning = False
            
            awaiting_arbiter = False

        # 5. Handle Scanning Motion
        if is_scanning:
            v = 0.0
            w = 0.05
            scan_ticks_remaining -= 1
            if scan_ticks_remaining <= 0:
                is_scanning = False
                # Step forward slightly to escape local FOV traps
                if lidar_dists["front"] > 80:
                    v = 1.0

        # 6. Nav2 P-Controller Execution
        if active_target and not awaiting_arbiter:
            dx = active_target["x"] - robot_x
            dy = active_target["y"] - robot_y
            dist = math.hypot(dx, dy)
            target_angle = math.atan2(dy, dx)
            angle_err = normalize_angle(target_angle - robot_theta)

            if dist < 22:
                for k, obj_data in objects.items():
                    if obj_data == active_target:
                        print(f"\n[Nav2] >>> Reached waypoint: [{k}]. Completed.")
                        completed_targets.add(k)
                        break
                active_target = None
                v, w = 0.0, 0.0
            else:
                w = max(-MAX_W, min(MAX_W, angle_err * 0.12))
                v = min(MAX_V, dist * 0.04) if abs(angle_err) < 0.6 else 0.0

                # Obstacle safety brake
                if lidar_dists["front"] < 35:
                    v = 0.0
                    w = MAX_W if lidar_dists["left"] > lidar_dists["right"] else -MAX_W

        # 7. Apply Kinematics
        robot_theta = normalize_angle(robot_theta + w)
        robot_x += math.cos(robot_theta) * v
        robot_y += math.sin(robot_theta) * v

        # --- RENDERING ---
        for wall in walls:
            pygame.draw.line(screen, (90, 90, 95), wall[0], wall[1], 4)

        for oid, obj in objects.items():
            is_done = oid in completed_targets
            color = (60, 60, 60) if is_done else obj["color"]
            pygame.draw.circle(screen, color, (obj["x"], obj["y"]), 15)
            if active_target == obj:
                pygame.draw.circle(screen, (250, 204, 21), (obj["x"], obj["y"]), 22, 2)
            lbl = font_small.render(f"[{oid}]", True, (160, 160, 160))
            screen.blit(lbl, (obj["x"] - 25, obj["y"] + 20))

        # Vision Cone
        lx = robot_x + math.cos(robot_theta - math.pi/4) * 320
        ly = robot_y + math.sin(robot_theta - math.pi/4) * 320
        rx = robot_x + math.cos(robot_theta + math.pi/4) * 320
        ry = robot_y + math.sin(robot_theta + math.pi/4) * 320
        pygame.draw.polygon(screen, (38, 38, 48), [(robot_x, robot_y), (lx, ly), (rx, ry)], 1)

        # Robot
        r_color = (250, 204, 21) if awaiting_arbiter else (34, 197, 94)
        pygame.draw.circle(screen, r_color, (int(robot_x), int(robot_y)), 12)
        pygame.draw.line(screen, (255, 255, 255), (robot_x, robot_y), 
                         (robot_x + math.cos(robot_theta)*18, robot_y + math.sin(robot_theta)*18), 3)

        # HUD Panel
        screen.blit(font.render(f"MISSION: \"{current_mission}\"", True, (255, 255, 255)), (20, 15))
        screen.blit(font_small.render(f"ARBITER LAST DECISION: {last_decision_str}", True, (250, 204, 21)), (20, 36))
        status_txt = "Awaiting 4090..." if awaiting_arbiter else ("Scanning Area..." if is_scanning else ("Driving to Target" if active_target else "Cruising Corridor"))
        screen.blit(font_small.render(f"ROBOT STATE: {status_txt}", True, (150, 150, 150)), (20, 54))

        pygame.display.flip()

        for event in pygame.event.get():
            if event.type == pygame.QUIT: running = False
            if event.type == pygame.KEYDOWN and event.key == pygame.K_ESCAPE: running = False

    query_queue.put(None)
    worker_proc.join()
    pygame.quit()
    sys.exit()
