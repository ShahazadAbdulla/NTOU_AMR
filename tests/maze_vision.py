import pygame
import time
import sys
import zenoh
import json
import threading
import queue
import base64
import io
from PIL import Image

print("[*] Initializing Multimodal Zenoh Client to 4090 Server...", flush=True)
config = zenoh.Config()
config.insert_json5("connect/endpoints", '["tcp/100.73.159.102:7447"]')
session = zenoh.open(config)

query_queue = queue.Queue()
result_queue = queue.Queue()

pygame.init()
WIDTH, HEIGHT = 900, 560
screen = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption("Strands-2B Multimodal Vision Auto-Explore")
font = pygame.font.SysFont("monospace", 13)
font_bold = pygame.font.SysFont("monospace", 14, bold=True)
clock = pygame.time.Clock()

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

def capture_screen_to_b64(surface):
    """Captures the PyGame screen, resizes it for latency optimization, and encodes to Base64."""
    # 1. Grab raw RGB buffer from PyGame
    raw_str = pygame.image.tostring(surface, "RGB")
    img = Image.frombytes("RGB", surface.get_size(), raw_str)
    
    # 2. Downscale to keep ViT latency < 200ms
    img.thumbnail((450, 450)) 
    
    # 3. Compress to JPEG
    buf = io.BytesIO()
    img.save(buf, format="JPEG", quality=75)
    return base64.b64encode(buf.getvalue()).decode("utf-8")

# ---------------------------------------------------------
# ZENOH WORKER (No changes needed here)
# ---------------------------------------------------------
def strands_worker():
    print("[Thread] Worker initialized and listening to queue...", flush=True)
    while True:
        try:
            payload = query_queue.get(timeout=0.1)
            if payload is None: break
            
            t0 = time.perf_counter()
            replies = session.get("lab/robot/mission", payload=json.dumps(payload), timeout=15.0)
            
            received_valid_reply = False
            for reply in replies:
                dt = (time.perf_counter() - t0) * 1000
                if reply.ok is not None:
                    raw_str = reply.ok.payload.to_string()
                    result_queue.put((json.loads(raw_str), dt))
                    received_valid_reply = True
                elif reply.err is not None:
                    print(f"[Thread] [!] Raw ERR Payload: {reply.err.payload.to_string()}", flush=True)
            
            if not received_valid_reply:
                print(f"[Thread] [!] Warning: No 'reply.ok' found.", flush=True)
            query_queue.task_done()
            
        except queue.Empty:
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

bump_alert = ""

while running:
    clock.tick(60) 
    screen.fill((25, 25, 28))
    
    # High-Contrast Map Drawing for the VLM
    for y, row in enumerate(grid):
        for x, cell in enumerate(row):
            rx, ry = x * CELL_SIZE + GRID_OFFSET_X, y * CELL_SIZE + GRID_OFFSET_Y
            rect = pygame.Rect(rx, ry, CELL_SIZE, CELL_SIZE)
            
            # HIGH CONTRAST COLORS
            if cell == 1: color = (40, 40, 45) # Dark Gray (Wall)
            elif (x, y) in visited_cells: color = (59, 130, 246) # Bright Blue (Visited)
            else: color = (255, 255, 255) # Pure White (Unexplored)
            
            pygame.draw.rect(screen, color, rect)
            pygame.draw.rect(screen, (20, 20, 20), rect, 1) # Dark grid lines

    # Draw Robot
    cx, cy = robot_x * CELL_SIZE + GRID_OFFSET_X + CELL_SIZE // 2, robot_y * CELL_SIZE + GRID_OFFSET_Y + CELL_SIZE // 2
    robot_color = (250, 204, 21) if awaiting_arbiter else (34, 197, 94) # Bright Green
    pygame.draw.circle(screen, robot_color, (cx, cy), CELL_SIZE // 2 - 4)
    
    # Draw Heading
    dx, dy = directions[facing]
    pygame.draw.line(screen, (239, 68, 68), (cx, cy), (cx + dx * 16, cy + dy * 16), 5) # Bright Thick Red Line

    # UI Overlay
    screen.blit(font_bold.render("STRANDS-2B: MULTIMODAL AUTO-EXPLORE", True, (255, 255, 255)), (GRID_OFFSET_X, 18))
    status_text = "THINKING (Awaiting 4090)..." if awaiting_arbiter else ("AUTO" if auto_step else "MANUAL")
    screen.blit(font.render(f"[SPACE] Step  [A] Auto-Loop  Status: {status_text}", True, (250, 204, 21)), (GRID_OFFSET_X, HEIGHT - 35))
    
    # Important: Update the display BEFORE capturing so the model sees the current state
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
    # Trigger Network Request
    if step_now and not awaiting_arbiter:
        dx, dy = directions[facing]
        left_dir = (facing - 1) % 4
        right_dir = (facing + 1) % 4
        
        fx, fy = robot_x + dx, robot_y + dy
        lx, ly = robot_x + directions[left_dir][0], robot_y + directions[left_dir][1]
        rx, ry = robot_x + directions[right_dir][0], robot_y + directions[right_dir][1]
        
        dynamic_choices = {}
        if grid[fy][fx] == 0: dynamic_choices["move_forward"] = "The path ahead is clear."
        if grid[ly][lx] == 0: dynamic_choices["turn_left"] = "Open path to the left."
        if grid[ry][rx] == 0: dynamic_choices["turn_right"] = "Open path to the right."
        
        if len(dynamic_choices) == 0:
            dynamic_choices["uturn"] = "Dead end. Turn around."
            
        # --- THE REFLEX BYPASS ---
        if len(dynamic_choices) == 1:
            only_choice = list(dynamic_choices.keys())[0]
            print(f"\n[Client] Pos: ({robot_x}, {robot_y}) | Corridor/Corner Reflex. Bypassing AI...", flush=True)
            
            # Instantly drop the answer into the execution queue (0ms latency)
            result_queue.put(({"choice": only_choice, "confidence": 1.0}, 0.0))
            awaiting_arbiter = True 
            
        # --- THE SEMANTIC NAVIGATOR (Only fires at Intersections) ---
        else:
            img_b64 = capture_screen_to_b64(screen)
            print(f"\n[Client] Pos: ({robot_x}, {robot_y}) | Intersection detected! Firing Vision Payload with {len(dynamic_choices)} options...", flush=True)

            state_prompt = (
                "You are an autonomous robot navigating a maze. Look at the attached top-down map camera feed.\n"
                "MAP LEGEND:\n"
                "- Dark Gray = Walls\n"
                "- Pure White = Unexplored Openings (YOUR TARGET)\n"
                "- Bright Blue = Previously Visited Paths\n"
                "- Green Dot = You (The Robot)\n"
                "- Red Line = The direction you are facing\n\n"
                "Analyze the visual map to find the closest Pure White unexplored areas. Choose the direction that safely navigates you closer to them."
            )

            payload = {
                "state": state_prompt, 
                "images": [img_b64], 
                "choices": dynamic_choices
            }
            
            query_queue.put(payload)
            awaiting_arbiter = True
    # Process Network Response
    while not result_queue.empty():
        result, latency_ms = result_queue.get()
        chosen = result.get('choice', 'halt')
        conf = result.get('confidence', 0)
        
        print(f"[Client] >> 4090 VERDICT: {chosen.upper()} << (Conf: {conf*100:.1f}%) | Latency: {latency_ms:.1f}ms", flush=True)
        awaiting_arbiter = False

        # 3. Clean Execution - We know the choice is safe, so just drive.
        if chosen == "move_forward":
            robot_x, robot_y = robot_x + directions[facing][0], robot_y + directions[facing][1]
            visited_cells.add((robot_x, robot_y))
        elif chosen == "turn_left": 
            facing = (facing - 1) % 4
            robot_x, robot_y = robot_x + directions[facing][0], robot_y + directions[facing][1]
            visited_cells.add((robot_x, robot_y))
        elif chosen == "turn_right": 
            facing = (facing + 1) % 4
            robot_x, robot_y = robot_x + directions[facing][0], robot_y + directions[facing][1]
            visited_cells.add((robot_x, robot_y))
        elif chosen == "uturn": 
            facing = (facing + 2) % 4

pygame.quit()
session.close()
sys.exit()
