import heapq
import math
import random
import time

def manhattan(a, b):
    return abs(a[0] - b[0]) + abs(a[1] - b[1])

def reconstruct_path(came_from, current):
    path = [current]
    while current in came_from:
        current = came_from[current]
        path.append(current)
    return path[::-1]

def ucs(env, start, goal, space_time=False, known_dynamic=True):
    start_time = time.time()
    initial = (start[0], start[1], 0) if space_time else start
    frontier = [(0, initial)]
    came_from = {}
    cost_so_far = {initial: 0}
    expanded = 0
    while frontier:
        _, current = heapq.heappop(frontier)
        expanded += 1
        if space_time:
            x, y, t = current
            if (x, y) == goal:
                path = reconstruct_path(came_from, current)
                pos_path = [(px, py) for px, py, _ in path]
                return pos_path, cost_so_far[current], expanded, time.time() - start_time
        else:
            if current == goal:
                path = reconstruct_path(came_from, current)
                return path, cost_so_far[current], expanded, time.time() - start_time
        if space_time:
            neighbors = env.get_neighbors(*current[:2]) + [current[:2]]
            for nx, ny in neighbors:
                new_t = current[2] + 1
                new_cost = cost_so_far[current] + (0 if (nx, ny) == current[:2] else env.grid[nx][ny])
                new_state = (nx, ny, new_t)
                if known_dynamic and env.is_blocked_at(nx, ny, new_t):
                    continue
                if new_state not in cost_so_far or new_cost < cost_so_far[new_state]:
                    cost_so_far[new_state] = new_cost
                    heapq.heappush(frontier, (new_cost, new_state))
                    came_from[new_state] = current
        else:
            for nx, ny in env.get_neighbors(*start if space_time else current):
                new_cost = cost_so_far[current] + env.grid[nx][ny]
                new_state = (nx, ny)
                if new_state not in cost_so_far or new_cost < cost_so_far[new_state]:
                    cost_so_far[new_state] = new_cost
                    heapq.heappush(frontier, (new_cost, new_state))
                    came_from[new_state] = current
    return None, float('inf'), expanded, time.time() - start_time

def a_star(env, start, goal, space_time=False, known_dynamic=True):
    start_time = time.time()
    initial = (start[0], start[1], 0) if space_time else start
    frontier = [(0, initial)]
    came_from = {}
    cost_so_far = {initial: 0}
    expanded = 0
    while frontier:
        _, current = heapq.heappop(frontier)
        expanded += 1
        if space_time:
            x, y, t = current
            if (x, y) == goal:
                path = reconstruct_path(came_from, current)
                pos_path = [(px, py) for px, py, _ in path]
                return pos_path, cost_so_far[current], expanded, time.time() - start_time
        else:
            if current == goal:
                path = reconstruct_path(came_from, current)
                return path, cost_so_far[current], expanded, time.time() - start_time
        if space_time:
            neighbors = env.get_neighbors(*current[:2]) + [current[:2]]
            for nx, ny in neighbors:
                new_t = current[2] + 1
                new_cost = cost_so_far[current] + (0 if (nx, ny) == current[:2] else env.grid[nx][ny])
                new_state = (nx, ny, new_t)
                if known_dynamic and env.is_blocked_at(nx, ny, new_t):
                    continue
                if new_state not in cost_so_far or new_cost < cost_so_far[new_state]:
                    cost_so_far[new_state] = new_cost
                    priority = new_cost + manhattan((nx, ny), goal)
                    heapq.heappush(frontier, (priority, new_state))
                    came_from[new_state] = current
        else:
            for nx, ny in env.get_neighbors(*current):
                new_cost = cost_so_far[current] + env.grid[nx][ny]
                new_state = (nx, ny)
                if new_state not in cost_so_far or new_cost < cost_so_far[new_state]:
                    cost_so_far[new_state] = new_cost
                    priority = new_cost + manhattan(new_state, goal)
                    heapq.heappush(frontier, (priority, new_state))
                    came_from[new_state] = current
    return None, float('inf'), expanded, time.time() - start_time

def simulated_annealing_replan(env, start, goal, max_iter=1000, initial_temp=100, cooling_rate=0.99):
    start_time = time.time()
    current_pos = start
    path = [start]
    T = initial_temp
    expanded = 0
    for i in range(max_iter):
        expanded += 1
        if current_pos == goal:
            cost = sum(env.grid[x][y] for _, (x, y) in enumerate(path[1:]))
            return path, cost, expanded, time.time() - start_time
        neighbors = env.get_neighbors(*current_pos)
        if not neighbors:
            break
        next_pos = random.choice(neighbors)
        delta = manhattan(next_pos, goal) - manhattan(current_pos, goal)
        if delta < 0 or random.random() < math.exp(-delta / T):
            path.append(next_pos)
            current_pos = next_pos
        T *= cooling_rate
    return None, float('inf'), expanded, time.time() - start_time

def sa_with_restarts(env, start, goal, restarts=10, **kwargs):
    total_exp = 0
    total_time = 0
    for _ in range(restarts):
        path, cost, exp, t = simulated_annealing_replan(env, start, goal, **kwargs)
        total_exp += exp
        total_time += t
        if path:
            return path, cost, total_exp, total_time
    return None, float('inf'), total_exp, total_time
