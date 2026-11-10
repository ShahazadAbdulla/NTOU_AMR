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
pygame.display.set_caption("Strands-2B Goal-Oriented Breadcrumb Demo")
font = pygame.font.SysFont("monospace", 13)
font_bold = pygame.font.SysFont("monospace", 14, bold=True)
clock = pygame.time.Clock()

# 19x11 Complex Maze (1=Wall, 0=Path, 2=Exit)
grid = [
    [1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1],
    [1,0,0,0,0,0,1,0,0,0,1,0,0,0,0,0,0,0,1],
    [1,0,1,1,1,0,1,0,1,0,1,0,1,1,1,1,1,0,1],
    [1,0,1,0,0,0,0,0,1,0,0,0,1,0,0,0,0,0,1],
    [1,0,1,0,1,1,1,1,1,1,1,1,1,0,1,1,1,1,1],
    [1,0,1,0,0,0,1,0,0,0,0,0,1,0,0,0,0,0,1],
    [1,0,1,1,1,0,1,0,1,1,1,0,1,1,1,1,1,0,1],
    [1,0,0,0,1,0,0,0,1,0,1,0,0,0,1,0,0,0,1],
    [1,1,1,0,1,1,1,1,1,0,1,1,1,0,1,0,1,1,1],
    [1,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,2,1], # EXIT at (17, 9)
    [1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1]
]

CELL_SIZE = 40
GRID_OFFSET_X = (WIDTH - (19 * CELL_SIZE)) // 2
GRID_OFFSET_Y = 50

visited_cells = set()
robot_x, robot_y = 1, 1 
visited_cells.add((robot_x, robot_y))
directions = [(0, -1), (1, 0), (0, 1), (-1, 0)]
facing = 1  
breadcrumbs = {} # Stores temporary colored dots for the AI to look at

def capture_screen_to_b64(surface):
    raw_str = pygame.image.tostring(surface, "RGB")
    img = Image.frombytes("RGB", surface.get_size(), raw_str)
    img.thumbnail((450, 450)) 
    buf = io.BytesIO()
    img.save(buf, format="JPEG", quality=75)
    return base64.b64encode(buf.getvalue()).decode("utf-8")

def render_scene(is_victory, is_awaiting, is_auto):
    screen.fill((25, 25, 28))
    
    # 1. Base Map
    for y, row in enumerate(grid):
        for x, cell in enumerate(row):
            rx, ry = x * CELL_SIZE + GRID_OFFSET_X, y * CELL_SIZE + GRID_OFFSET_Y
            rect = pygame.Rect(rx, ry, CELL_SIZE, CELL_SIZE)
            
            if cell == 1: color = (40, 40, 45) # Wall
            elif cell == 2: color = (217, 70, 239) # EXTREME MAGENTA (Exit)
            elif (x, y) in visited_cells: color = (59, 130, 246) # Visited
            else: color = (255, 255, 255) # Unexplored
            
            pygame.draw.rect(screen, color, rect)
            pygame.draw.rect(screen, (20, 20, 20), rect, 1)

    # 2. Intersection Breadcrumbs (Colored Dots)
    for (bx, by), b_color in breadcrumbs.items():
        cx = bx * CELL_SIZE + GRID_OFFSET_X + CELL_SIZE // 2
        cy = by * CELL_SIZE + GRID_OFFSET_Y + CELL_SIZE // 2
        pygame.draw.circle(screen, b_color, (cx, cy), CELL_SIZE // 3)
        pygame.draw.circle(screen, (0, 0, 0), (cx, cy), CELL_SIZE // 3, 2) # Black outline

    # 3. Robot & Heading
    cx, cy = robot_x * CELL_SIZE + GRID_OFFSET_X + CELL_SIZE // 2, robot_y * CELL_SIZE + GRID_OFFSET_Y + CELL_SIZE // 2
    robot_color = (250, 204, 21) if is_awaiting else (34, 197, 94)
    pygame.draw.circle(screen, robot_color, (cx, cy), CELL_SIZE // 2 - 4)
    dx, dy = directions[facing]
    pygame.draw.line(screen, (239, 68, 68), (cx, cy), (cx + dx * 16, cy + dy * 16), 5)

    # 4. UI
    screen.blit(font_bold.render("STRANDS-2B: BREADCRUMB NAVIGATION DEMO", True, (255, 255, 255)), (GRID_OFFSET_X, 18))
    status_text = "VICTORY!" if is_victory else ("THINKING (Looking at Dots)..." if is_awaiting else ("AUTO" if is_auto else "MANUAL"))
    screen.blit(font.render(f"[SPACE] Step  [A] Auto-Loop  Status: {status_text}", True, (250, 204, 21)), (GRID_OFFSET_X, HEIGHT - 35))
    
    pygame.display.flip()

# ---------------------------------------------------------
# ZENOH WORKER
# ---------------------------------------------------------
def strands_worker():
    print("[Thread] Worker initialized...", flush=True)
    while True:
        try:
            payload = query_queue.get(timeout=0.1)
            if payload is None: break
            
            t0 = time.perf_counter()
            replies = session.get("lab/robot/mission", payload=json.dumps(payload), timeout=15.0)
            
            received_reply = False
            for reply in replies:
                if reply.ok is not None:
                    dt = (time.perf_counter() - t0) * 1000
                    result_queue.put((json.loads(reply.ok.payload.to_string()), dt))
                    received_reply = True
            
            if not received_reply:
                print(f"[Thread] [!] Warning: No 'reply.ok' found.", flush=True)
            query_queue.task_done()
        except queue.Empty:
            time.sleep(0.05) 
        except Exception as e:
            print(f"[Thread] [!] Hard Exception: {e}", flush=True)

threading.Thread(target=strands_worker, daemon=True).start()

running = True
auto_step = False
awaiting_arbiter = False
victory = False
tick_timer = time.time()

while running:
    clock.tick(60) 
    render_scene(victory, awaiting_arbiter, auto_step)

    if victory:
        for event in pygame.event.get():
            if event.type == pygame.QUIT or (event.type == pygame.KEYDOWN and event.key in (pygame.K_ESCAPE, pygame.K_q)):
                running = False
        continue

    step_now = False
    for event in pygame.event.get():
        if event.type == pygame.QUIT: running = False
        if event.type == pygame.KEYDOWN:
            if event.key == pygame.K_SPACE and not awaiting_arbiter: step_now = True
            if event.key == pygame.K_a: auto_step = not auto_step
            if event.key in (pygame.K_ESCAPE, pygame.K_q): running = False

    if auto_step and not awaiting_arbiter and (time.time() - tick_timer > 0.4):
        step_now = True
        tick_timer = time.time()

    # ---------------------------------------------------------
    # HYBRID DECISION ENGINE (WITH BREADCRUMBS)
    # ---------------------------------------------------------
    if step_now and not awaiting_arbiter:
        left_dir = (facing - 1) % 4
        right_dir = (facing + 1) % 4
        
        fx, fy = robot_x + directions[facing][0], robot_y + directions[facing][1]
        lx, ly = robot_x + directions[left_dir][0], robot_y + directions[left_dir][1]
        rx, ry = robot_x + directions[right_dir][0], robot_y + directions[right_dir][1]
        
        dynamic_choices = {}
        breadcrumbs.clear()
        
        if grid[fy][fx] in (0, 2): 
            dynamic_choices["move_forward"] = "Take the path with the Cyan dot."
            breadcrumbs[(fx, fy)] = (0, 255, 255) # Cyan
            
        if grid[ly][lx] in (0, 2): 
            dynamic_choices["turn_left"] = "Take the path with the Yellow dot."
            breadcrumbs[(lx, ly)] = (255, 255, 0) # Yellow
            
        if grid[ry][rx] in (0, 2): 
            dynamic_choices["turn_right"] = "Take the path with the Orange dot."
            breadcrumbs[(rx, ry)] = (255, 165, 0) # Orange
        
        if len(dynamic_choices) == 0:
            dynamic_choices["uturn"] = "Dead end. Turn around."
            
        # [REFLEX BYPASS] Fast-track single options (0ms)
        if len(dynamic_choices) == 1:
            only_choice = list(dynamic_choices.keys())[0]
            breadcrumbs.clear() # Clear dots if we don't need AI
            print(f"\n[Client] Pos: ({robot_x}, {robot_y}) | Driver Reflex. Bypassing AI...", flush=True)
            result_queue.put(({"choice": only_choice, "confidence": 1.0}, 0.0))
            awaiting_arbiter = True 
            
        # [SEMANTIC NAVIGATOR] AI handles intersections visually
        else:
            print(f"\n[Client] Pos: ({robot_x}, {robot_y}) | INTERSECTION! Dropping Breadcrumbs and asking AI...", flush=True)
            
            # Force immediate screen render so the breadcrumbs physically appear in the image capture
            render_scene(victory, True, auto_step)
            img_b64 = capture_screen_to_b64(screen)

            state_prompt = (
                "You are an autonomous robot navigating a complex maze. Look at the attached top-down map camera feed.\n"
                "MAP LEGEND:\n"
                "- Dark Gray = Walls\n"
                "- Bright Blue = Visited Paths\n"
                "- Bright Magenta = THE EXIT DESTINATION (YOUR GOAL)\n"
                "- Green Dot = You (The Robot)\n"
                "- Colored Dots (Cyan/Yellow/Orange) = Your available movement options at this intersection.\n\n"
                "Analyze the visual map. Which colored dot sits on the path that leads most directly to the Bright Magenta Exit?"
            )

            payload = {
                "state": state_prompt, 
                "images": [img_b64], 
                "choices": dynamic_choices
            }
            
            query_queue.put(payload)
            awaiting_arbiter = True

    # ---------------------------------------------------------
    # EXECUTION
    # ---------------------------------------------------------
    while not result_queue.empty():
        result, latency_ms = result_queue.get()
        chosen = result.get('choice', 'halt')
        
        if latency_ms > 0:
            print(f"[Client] >> 4090 VERDICT: {chosen.upper()} << (Conf: {result.get('confidence', 0)*100:.1f}%) | Latency: {latency_ms:.1f}ms", flush=True)
            
        awaiting_arbiter = False
        breadcrumbs.clear() # Erase breadcrumbs after decision

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

        if grid[robot_y][robot_x] == 2:
            print(f"\n[!] VICTORY: The robot successfully navigated to the Magenta Exit!")
            victory = True

pygame.quit()
session.close()
sys.exit()
