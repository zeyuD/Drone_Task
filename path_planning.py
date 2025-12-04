import random
import heapq
import time
from typing import List, Tuple, Optional
import matplotlib.pyplot as plt
import numpy as np

Coord = Tuple[int, int]  # (row, col)


# ==========================
# 1. Maze (obstacle) generation
# ==========================
def generate_random_grid(
    m: int,
    n: int,
    start: Coord,
    goal: Coord,
    obstacle_prob: float = 0.3,
    max_tries: int = 100
) -> List[List[int]]:
    """Generate a random m x n grid with obstacles (0 = free, 1 = obstacle)."""
    for _ in range(max_tries):
        grid = []
        for r in range(m):
            row = []
            for c in range(n):
                if (r, c) in (start, goal):
                    row.append(0)
                else:
                    row.append(1 if random.random() < obstacle_prob else 0)
            grid.append(row)

        return grid

    raise ValueError("Could not generate valid grid.")

def visualize_grid(grid):
    """
    Display the grid:
    - 1 = black (obstacle)
    - 0 = white (free)
    """
    plt.figure(figsize=(6, 6))
    plt.imshow(grid, cmap="binary")  # binary colormap: 1=black, 0=white
    plt.title("Grid Visualization (Obstacles in Black)")
    plt.xlabel("Column")
    plt.ylabel("Row")
    plt.gca().invert_yaxis()  # match array indexing (0,0 at top-left)
    plt.show()

# ==========================
# 2. A* path planning
# ==========================
def heuristic(a: Coord, b: Coord) -> float:
    return abs(a[0] - b[0]) + abs(a[1] - b[1])


def get_neighbors(pos: Coord, m: int, n: int) -> List[Coord]:
    r, c = pos
    candidates = [(r-1, c), (r+1, c), (r, c-1), (r, c+1)]
    return [(x, y) for x, y in candidates if 0 <= x < m and 0 <= y < n]


def astar(grid: List[List[int]], start: Coord, goal: Coord) -> Optional[List[Coord]]:
    m, n = len(grid), len(grid[0])
    open_set = []
    heapq.heappush(open_set, (heuristic(start, goal), 0, start))

    came_from = {}
    g = {start: 0}
    visited = set()

    while open_set:
        f, cost, curr = heapq.heappop(open_set)
        if curr in visited:
            continue
        visited.add(curr)

        if curr == goal:
            # reconstruct path
            path = [curr]
            while curr in came_from:
                curr = came_from[curr]
                path.append(curr)
            return list(reversed(path))

        for nb in get_neighbors(curr, m, n):
            if grid[nb[0]][nb[1]] == 1:  # obstacle
                continue
            new_cost = cost + 1
            if nb not in g or new_cost < g[nb]:
                g[nb] = new_cost
                came_from[nb] = curr
                heapq.heappush(open_set, (new_cost + heuristic(nb, goal), new_cost, nb))

    return None


# ==========================
# 3. Visualization
# ==========================
def visualize_grid_and_path(grid: List[List[int]], path: List[Coord], start: Coord, goal: Coord):
    m, n = len(grid), len(grid[0])

    grid_np = np.array(grid)

    plt.figure(figsize=(8, 8))
    plt.imshow(grid_np, cmap="binary")  # 1=black, 0=white



    # Plot path
    if path:
        pr = [p[0] for p in path]
        pc = [p[1] for p in path]
        plt.plot(pc, pr, color="blue", linewidth=2, label="Path")

    # Start point
    plt.scatter(start[1], start[0], color="green", s=100, label="Start")

    # Goal point
    plt.scatter(goal[1], goal[0], color="red", s=100, label="Goal")

    plt.legend()
    plt.title("A* Path Planning on Random Grid")
    plt.gca().invert_yaxis()  # match grid indexing
    plt.savefig("path_planning_result.png")
    plt.show()


# ==========================
# 4. Demo
# ==========================
if __name__ == "__main__":
    m, n = 20, 20
    start = (0, 0)
    goal = (19, 19)
    num_trials = 0

    while True:
        start_time = time.time()
        grid = generate_random_grid(m, n, start, goal, obstacle_prob=0.25)
        grid_time = time.time()
        path = astar(grid, start, goal)
        path_time = time.time()
        num_trials += 1
        if path:
            break

    print(f"Path found after {num_trials} trials.")
    print(f"Grid generation time: {grid_time - start_time:.4f}s")
    print(f"Pathfinding time: {path_time - grid_time:.4f}s")
    # print("Path length:", len(path))
    visualize_grid_and_path(grid, path, start, goal)
    print("Grid:")
    for r in range(m):
        print(grid[m-1-r])
    print("Path coordinates:", path)
