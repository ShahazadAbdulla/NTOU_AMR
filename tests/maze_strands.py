import pygame
import time
import sys
import zenoh
import json
import threading
import queue

print("[*] Initializing Zenoh Client to 4090 Server...", flush=True)
config = zenoh.Config()
# Target the 4090 Tailscale IP directly over TCP
config.insert_json5("connect/endpoints", '["tcp/100.73.159.102:7447"]')
session = zenoh.open(config)

query_queue = queue.Queue()
result_queue = queue.Queue()

pygame.init()
WIDTH, HEIGHT = 900, 560
screen = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption("Strands-12B Asynchronous Auto-Explore")
font = pygame.font.SysFont("monospace", 13)
font_bold = pygame.font.SysFont("monospace", 14, bold=True)
clock = pygame.time.Clock() # PyGame framerate controller

grid = [
    [1,1,1,1,1,1,1,1,1,1,1,1,1,1,1],
    [1,0,0,0,1,0,0,0,0,0,1,0,0,0,1],
    [1,0,1,0,1,0,1,1,1,0,1,0,1,0,1],
    [1,0,1,0,0,0,1,0,0,0,0,0,1,0,1],
    [1,0,1,1,1,1,1,0,1,1,1,1,1,0,1],
    [1,0,0,0,0,0,0,0,0,0,0,0,0,0,1], 
    [1,1,1,1,0,1,1,1,1,1,1,1,1,1,1],
    [1,0,0,0,0,0,1,0,0,0,1,0,0,0,1],
    [1,1,1,1,1,1,1,1,1,1,1,1,1,1,1]
]
CELL_SIZE = 36
GRID_OFFSET_X = 40
GRID_OFFSET_Y = 50

visited_cells = set()
robot_x, robot_y = 1, 5
visited_cells.add((robot_x, robot_y))
directions = [(0, -1), (1, 0), (0, 1), (-1, 0)]
dir_names = ["NORTH", "EAST", "SOUTH", "WEST"]
facing = 1  

actions = {
    "explore_forward": "Front is an UNEXPLORED OPENING. Drive straight to map it.",
    "explore_left": "Left is an UNEXPLORED OPENING. Turn left to map it.",
    "explore_right": "Right is an UNEXPLORED OPENING. Turn right to map it.",
    "backtrack_left": "Left is OPEN (previously visited). Turn left into the side path to exit the current corridor.",
    "backtrack_right": "Right is OPEN (previously visited). Turn right into the side path to exit the current corridor.",
    "backtrack_forward": "Front is OPEN (previously visited) AND sides are WALLs. Continue driving straight.",
    "dead_end_uturn": "Front, Left, and Right are WALLs. Dead end. Turn around."
}

def get_sensor_telemetry():
    dx, dy = directions[facing]
    left_dir = (facing - 1) % 4
    right_dir = (facing + 1) % 4
    
    fx, fy = robot_x + dx, robot_y + dy
    lx, ly = robot_x + directions[left_dir][0], robot_y + directions[left_dir][1]
    rx, ry = robot_x + directions[right_dir][0], robot_y + directions[right_dir][1]
    
    def describe(x, y):
        if grid[y][x] == 1: return "WALL"
        if (x, y) in visited_cells: return "OPEN (previously visited)"
        return "UNEXPLORED OPENING"

    return f"Pose: ({robot_x}, {robot_y}) facing {dir_names[facing]}. Front: {describe(fx, fy)}. Left: {describe(lx, ly)}. Right: {describe(rx, ry)}."

# ---------------------------------------------------------
# ROBUST BACKGROUND ZENOH WORKER
# ---------------------------------------------------------
def strands_worker():
    print("[Thread] Worker initialized and listening to queue...", flush=True)
    while True:
        try:
            payload = query_queue.get(timeout=0.1)
            if payload is None: break
            
            print(f"\n[Thread] >> Firing Zenoh Packet at {time.time():.3f}...", flush=True)
            t0 = time.perf_counter()
            
            # Explicit timeout added to prevent early Zenoh hangs
            replies = session.get("lab/robot/mission", payload=json.dumps(payload), timeout=15.0)
            
            received_valid_reply = False
            for reply in replies:
                dt = (time.perf_counter() - t0) * 1000
                print(f"[Thread] << Reply object triggered at {dt:.1f} ms", flush=True)
                
                if reply.ok is not None:
                    raw_str = reply.ok.payload.to_string()
                    print(f"[Thread] Raw OK Payload: {raw_str[:60]}...", flush=True)
                    result_queue.put((json.loads(raw_str), dt))
                    received_valid_reply = True
                elif reply.err is not None:
                    err_str = reply.err.payload.to_string()
                    print(f"[Thread] [!] Raw ERR Payload: {err_str}", flush=True)
            
            if not received_valid_reply:
                print(f"[Thread] [!] Warning: Loop completed but no 'reply.ok' found. Time elapsed: {(time.perf_counter()-t0)*1000:.1f}ms", flush=True)
                
            query_queue.task_done()
            
        except queue.Empty:
            # Forcing GIL release so Zenoh C-bindings can process incoming Tailscale TCP packets
            time.sleep(0.05) 
            continue
        except Exception as e:
            print(f"[Thread] [!] Hard Exception: {e}", flush=True)

thread = threading.Thread(target=strands_worker, daemon=True)
thread.start()

running = True
auto_step = False
awaiting_arbiter = False
last_action = "none"
tick_timer = time.time()

print("="*70)
print("  STRANDS-12B ASYNCHRONOUS AUTO-EXPLORE (ZENOH NETWORK)")
print("="*70)

bump_alert = ""

while running:
    # Cap framerate to 60 FPS to prevent PyGame from hogging the CPU core
    clock.tick(60) 
    screen.fill((25, 25, 28))
    
    # Draw Map
    for y, row in enumerate(grid):
        for x, cell in enumerate(row):
            rx, ry = x * CELL_SIZE + GRID_OFFSET_X, y * CELL_SIZE + GRID_OFFSET_Y
            rect = pygame.Rect(rx, ry, CELL_SIZE, CELL_SIZE)
            if cell == 1: color = (60, 60, 65)
            elif (x, y) in visited_cells: color = (180, 215, 240)
            else: color = (235, 235, 235)
            pygame.draw.rect(screen, color, rect)
            pygame.draw.rect(screen, (40, 40, 45), rect, 1)

    # Draw Robot
    cx, cy = robot_x * CELL_SIZE + GRID_OFFSET_X + CELL_SIZE // 2, robot_y * CELL_SIZE + GRID_OFFSET_Y + CELL_SIZE // 2
    robot_color = (250, 204, 21) if awaiting_arbiter else (34, 197, 94)
    pygame.draw.circle(screen, robot_color, (cx, cy), CELL_SIZE // 2 - 4)
    
    # Draw Heading
    dx, dy = directions[facing]
    pygame.draw.line(screen, (239, 68, 68), (cx, cy), (cx + dx * 14, cy + dy * 14), 4)

    # UI Overlay
    screen.blit(font_bold.render("STRANDS-12B: ACTIVE DISTRIBUTED TELEMETRY", True, (255, 255, 255)), (GRID_OFFSET_X, 18))
    status_text = "THINKING (Awaiting 4090)..." if awaiting_arbiter else ("AUTO" if auto_step else "MANUAL")
    screen.blit(font.render(f"[SPACE] Step  [A] Auto-Loop  Status: {status_text}", True, (250, 204, 21)), (GRID_OFFSET_X, HEIGHT - 35))
    pygame.display.flip()

    step_now = False
    for event in pygame.event.get():
        if event.type == pygame.QUIT: running = False
        if event.type == pygame.KEYDOWN:
            if event.key == pygame.K_SPACE and not awaiting_arbiter: step_now = True
            if event.key == pygame.K_a: auto_step = not auto_step
            if event.key in (pygame.K_ESCAPE, pygame.K_q): running = False

    if auto_step and not awaiting_arbiter and (time.time() - tick_timer > 0.5):
        step_now = True
        tick_timer = time.time()

    # Trigger Network Request
    if step_now and not awaiting_arbiter:
        sensor_str = get_sensor_telemetry()
        print(f"\n[Client Main] Pos: ({robot_x}, {robot_y}) | Sending Request...", flush=True)

        payload = {"state": f"{sensor_str} Previous action: {last_action}. {bump_alert}", "choices": actions}
        query_queue.put(payload)
        awaiting_arbiter = True

    # Process Network Response
    while not result_queue.empty():
        result, latency_ms = result_queue.get()
        chosen = result.get('choice')
        conf = result.get('confidence', 0)
        
        print(f"[Client Main] >> ARBITER VERDICT: {chosen.upper()} << (Conf: {conf*100:.1f}%) | Latency: {latency_ms:.1f}ms", flush=True)
        last_action = chosen
        awaiting_arbiter = False

        if chosen in ("explore_forward", "backtrack_forward"):
            nx, ny = robot_x + directions[facing][0], robot_y + directions[facing][1]
            if grid[ny][nx] == 0:
                robot_x, robot_y = nx, ny
                visited_cells.add((robot_x, robot_y))
                bump_alert = "" # Clear alert on success
            else:
                print(" [!] PHYSICAL COLLISION PREVENTED BY CONTROLLER.")
                bump_alert = " [CRITICAL ERROR: The previous forward action FAILED because it hit a WALL. You MUST turn.]"
        elif chosen in ("explore_left", "backtrack_left"): 
            facing = (facing - 1) % 4
            bump_alert = ""
        elif chosen in ("explore_right", "backtrack_right"): 
            facing = (facing + 1) % 4
            bump_alert = ""
        elif chosen == "dead_end_uturn": 
            facing = (facing + 2) % 4
            bump_alert = ""

pygame.quit()
session.close()
sys.exit()
