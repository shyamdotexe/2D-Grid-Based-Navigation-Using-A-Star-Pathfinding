import sys
from queue import PriorityQueue

import pygame

pygame.init()
drone_img = pygame.image.load("drone.png")
drone_img = pygame.transform.scale(drone_img, (40, 40))

WIDTH = 600
PANEL_WIDTH = 200
TITLE_HEIGHT = 40

WIN = pygame.display.set_mode((WIDTH + PANEL_WIDTH, WIDTH + TITLE_HEIGHT))
pygame.display.set_caption("2D Drone Navigation Simulation")

pygame.font.init()
font = pygame.font.SysFont("terminal", 24)
title_font = pygame.font.SysFont("terminal", 28)

GRID_DARK = (30, 30, 30)
PANEL_DARK = (25, 25, 25)
HEADER_DARK = (15, 15, 15)
LINE_GREY = (60, 60, 60)

RED = (255, 80, 80)
GREEN = (0, 200, 0)
BLACK = (10, 10, 10)
WHITE = (60, 60, 60)
PURPLE = (180, 0, 255)
ORANGE = (255, 165, 40)
TURQUOISE = (0, 200, 200)
BUTTON_GREY = (50, 50, 50)
TEXT_COLOR = (220, 220, 220)



class Button: #for gui
    def __init__(self, x, y, width, height, text, callback):
        self.rect = pygame.Rect(x, y, width, height)
        self.text = text
        self.callback = callback

    def draw(self, win):
        pygame.draw.rect(win, BUTTON_GREY, self.rect, border_radius=8)
        label = font.render(self.text, True, TEXT_COLOR)
        lw, lh = label.get_size()
        win.blit(label, (self.rect.x + (self.rect.w - lw) // 2,
                         self.rect.y + (self.rect.h - lh) // 2))

    def click(self, pos):
        if self.rect.collidepoint(pos):
            self.callback()



class Node: # for each Cube in the grid
    def __init__(self, row, col, width, total_rows):
        self.row = row
        self.col = col
        self.x = col * width
        self.y = row * width
        self.color = BLACK
        self.width = width
        self.total_rows = total_rows
        self.neighbours = []

    def get_pos(self):
        return self.row, self.col

    def is_barrier(self):
        return self.color == WHITE

    def reset(self):
        self.color = BLACK

    def make_start(self):
        self.color = ORANGE

    def make_end(self):
        self.color = TURQUOISE

    def make_open(self):
        self.color = GREEN

    def make_closed(self):
        self.color = RED

    def make_barrier(self):
        self.color = WHITE

    def make_path(self):
        self.color = PURPLE

    def draw(self, win):
        pygame.draw.rect(win, self.color,
                         (self.x, self.y, self.width, self.width))

    def update_neighbors(self, grid):
        self.neighbours = []
        r, c = self.row, self.col

        if r < self.total_rows - 1 and not grid[r + 1][c].is_barrier():
            self.neighbours.append(grid[r + 1][c])
        if r > 0 and not grid[r - 1][c].is_barrier():
            self.neighbours.append(grid[r - 1][c])
        if c < self.total_rows - 1 and not grid[r][c + 1].is_barrier():
            self.neighbours.append(grid[r][c + 1])
        if c > 0 and not grid[r][c - 1].is_barrier():
            self.neighbours.append(grid[r][c - 1])

    def __lt__(self, other):
        return False


def h(p1, p2):
    x1, y1 = p1
    x2, y2 = p2
    return abs(x1 - x2) + abs(y1 - y2)



def reconstruct_path(came_from, current):
    path = []
    while current in came_from:
        current = came_from[current]
        path.append(current)
    return list(reversed(path))


def draw_full(win, grid, rows, width, drone_node, buttons, message):
    # Title bar
    pygame.draw.rect(win, HEADER_DARK, (0, 0, width + PANEL_WIDTH, TITLE_HEIGHT))
    title = title_font.render("2D Drone Navigation Simulation using A* Search Algorithm", True, TEXT_COLOR)
    win.blit(title, (20, 6))

    # Grid
    grid_surface = pygame.Surface((width, width))
    grid_surface.fill(GRID_DARK)

    for row in grid:
        for node in row:
            pygame.draw.rect(grid_surface, node.color,
                             (node.x, node.y, node.width, node.width))

    gap = width // rows
    for i in range(rows):
        pygame.draw.line(grid_surface, LINE_GREY, (0, i * gap), (width, i * gap))
    for j in range(rows):
        pygame.draw.line(grid_surface, LINE_GREY, (j * gap, 0), (j * gap, width))

    win.blit(grid_surface, (0, TITLE_HEIGHT))

    # Panel
    pygame.draw.rect(win, PANEL_DARK, (WIDTH, TITLE_HEIGHT, PANEL_WIDTH, width))
    for btn in buttons:
        btn.draw(win)

    # Drone
    if drone_node:
        win.blit(drone_img, (drone_node.x, drone_node.y + TITLE_HEIGHT))

    # Message
    if message:
        msg = font.render(message, True, TEXT_COLOR)
        win.blit(msg, (WIDTH + 10, width - 20))

    pygame.display.update()



def algorithm(draw_step, grid, start, end): # A* ALGORITHM
    count = 0
    open_set = PriorityQueue()
    open_set.put((0, count, start))
    came_from = {}

    g_score = {node: float("inf") for row in grid for node in row}
    g_score[start] = 0

    f_score = {node: float("inf") for row in grid for node in row}
    f_score[start] = h(start.get_pos(), end.get_pos())

    open_set_hash = {start}
    start.make_open()

    while not open_set.empty():

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                sys.exit()

        current = open_set.get()[2]
        open_set_hash.remove(current)

        if current == end:
            path = reconstruct_path(came_from, current)
            for node in path:
                if node not in (start, end):
                    node.make_path()
                draw_step()
            return end

        for neighbor in current.neighbours:
            temp_g = g_score[current] + 1

            if temp_g < g_score[neighbor]:
                came_from[neighbor] = current
                g_score[neighbor] = temp_g
                f_score[neighbor] = temp_g + h(neighbor.get_pos(), end.get_pos())

                if neighbor not in open_set_hash:
                    count += 1
                    open_set.put((f_score[neighbor], count, neighbor))
                    open_set_hash.add(neighbor)
                    neighbor.make_open()

        draw_step()

        if current != start:
            current.make_closed()

    return False

def construct_maze():
    pass #to be implemented later :)

def make_grid(rows, width):
    grid = []
    gap = width // rows
    for r in range(rows):
        row = []
        for c in range(rows):
            row.append(Node(r, c, gap, rows))
        grid.append(row)
    return grid

def get_clicked_pos(pos, rows, width):
    gap = width // rows
    x, y = pos
    return (y - TITLE_HEIGHT) // gap, x // gap



def main(win, width):
    rows = 40
    grid = make_grid(rows, width)

    start = None
    end = None
    drone_node = None
    message = None

    clock = pygame.time.Clock()


    def start_search():
        nonlocal message, drone_node
        if not start or not end:
            message = "Place Start & End!"
            return
        if start == end:
            message = "Start != End!"
            return

        for row in grid:
            for node in row:
                node.update_neighbors(grid)

        result = algorithm(lambda:
                           draw_full(win, grid, rows, width, drone_node, buttons, message),
                           grid, start, end)

        if result:
            message = "Path Found!"
            drone_node = result
        else:
            message = "No Path!"

    def clear_all():
        nonlocal grid, start, end, drone_node, message
        grid = make_grid(rows, width)
        start = end = drone_node = None
        message = None



    bx = WIDTH + 20
    bw = PANEL_WIDTH - 40
    bh = 40

    buttons = [
        Button(bx, TITLE_HEIGHT + 20, bw, bh, "Start Search", start_search),
        Button(bx, TITLE_HEIGHT + 80, bw, bh, "Clear Grid", clear_all),

    ]

    run = True
    while run:
        clock.tick(60)

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                run = False

            if event.type == pygame.MOUSEBUTTONDOWN:
                pos = pygame.mouse.get_pos()
                for b in buttons:
                    b.click(pos)

            # left-click grid
            if pygame.mouse.get_pressed()[0]:
                mx, my = pygame.mouse.get_pos()
                if 0 <= mx < WIDTH and my > TITLE_HEIGHT:
                    row, col = get_clicked_pos((mx, my), rows, width)
                    if 0 <= row < rows and 0 <= col < rows:
                        node = grid[row][col]

                        if not start:
                            if node == end:
                                message = "Start != End!"
                            else:
                                start = node
                                start.make_start()
                                drone_node = start
                        elif not end:
                            if node != start:
                                end = node
                                end.make_end()
                        elif node not in (start, end):
                            node.make_barrier()

            # right click erase
            if pygame.mouse.get_pressed()[2]:
                mx, my = pygame.mouse.get_pos()
                if 0 <= mx < WIDTH and my > TITLE_HEIGHT:
                    row, col = get_clicked_pos((mx, my), rows, width)
                    if 0 <= row < rows and 0 <= col < rows:
                        node = grid[row][col]
                        node.reset()
                        if node == start:
                            start = drone_node = None
                        elif node == end:
                            end = None

            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_SPACE:
                    start_search()
                if event.key == pygame.K_c:
                    clear_all()

        draw_full(win, grid, rows, width, drone_node, buttons, message)

    pygame.quit()

if __name__ == "__main__":
    main(WIN, WIDTH)
