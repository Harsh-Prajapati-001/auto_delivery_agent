import argparse
from environment import Environment, MovingObstacle
from planners import ucs, a_star, sa_with_restarts

def get_env(map_name):
    if map_name == 'small':
        grid = [[1, 1, 1, 1, 1] for _ in range(5)]
        grid[1][1] = grid[1][3] = grid[3][1] = grid[3][3] = float('inf')
        return Environment(grid, 5, 5)
    elif map_name == 'medium':
        grid = [[random.randint(1, 5) for _ in range(10)] for _ in range(10)]
        grid[4][4] = grid[5][5] = float('inf')  # some obstacles
        return Environment(grid, 10, 10)
    elif map_name == 'large':
        grid = [[random.randint(1, 5) for _ in range(20)] for _ in range(20)]
        for i in range(20):
            if random.random() < 0.1:
                grid[random.randint(0,19)][i] = float('inf')
        return Environment(grid, 20, 20)
    elif map_name == 'dynamic':
        grid = [[1 for _ in range(10)] for _ in range(10)]
        grid[5][5] = float('inf')  # static
        obs = MovingObstacle([(3,3), (3,4), (3,5), (3,4)])
        return Environment(grid, 10, 10, [obs])
    else:
        raise ValueError("Invalid map")

def simulate_unpredictable(env, start, goal):
    path, cost, exp, t = a_star(env, start, goal, space_time=False, known_dynamic=False)
    if not path:
        return None, float('inf'), exp, t
    total_exp = exp
    total_time = t
    current = start
    sim_t = 0
    log = []
    replanned = False
    while current != goal:
        next_index = path.index(current) + 1 if current in path else len(path)
        if next_index >= len(path):
            break
        next_pos = path[next_index]
        if env.is_blocked_at(*next_pos, sim_t + 1):
            print(f"Obstacle at {next_pos} at time {sim_t+1}, replanning from {current}")
            new_path, new_cost, new_exp, new_t = sa_with_restarts(env, current, goal)
            total_exp += new_exp
            total_time += new_t
            if not new_path:
                return None, float('inf'), total_exp, total_time
            path = new_path
            replanned = True
            continue
        current = next_pos
        sim_t += 1
        log.append((sim_t, current, env.dynamic_obstacles[0].position_at(sim_t) if env.dynamic_obstacles else None))
    if replanned:
        with open('replan_log.txt', 'w') as f:
            for entry in log:
                f.write(f"Time {entry[0]}: Agent at {entry[1]}, Obstacle at {entry[2]}\n")
    cost = sum(env.grid[x][y] for x, y in path[1:])
    return path, cost, total_exp, total_time

def print_grid(env, agent_pos, t):
    obs_pos = set(env.dynamic_obstacles[0].position_at(t) for _ in env.dynamic_obstacles) if env.dynamic_obstacles else set()
    for i in range(env.rows):
        row = []
        for j in range(env.cols):
            if (i, j) == agent_pos:
                row.append('A')
            elif (i, j) in obs_pos:
                row.append('O')
            elif env.grid[i][j] == float('inf'):
                row.append('#')
            else:
                row.append(str(env.grid[i][j]) if env.grid[i][j] > 1 else '.')
        print(' '.join(row))
    print(f"Time: {t}")

def run_demo(env, start, goal, dynamic_mode):
    if dynamic_mode != 'unpredictable':
        print("Demo only for unpredictable mode on dynamic map")
        return
    path, cost, exp, t = simulate_unpredictable(env, start, goal)
    if path:
        print("Demo simulation (use for screenshots):")
        sim_t = 0
        for pos in path:
            print_grid(env, pos, sim_t)
            sim_t += 1
    else:
        print("No path found")

def run_experiment():
    maps = ['small', 'medium', 'large', 'dynamic']
    algorithms = ['ucs', 'a_star', 'sa']
    print("| Map | Algorithm | Path Cost | Nodes Expanded | Time (s) |")
    print("|---|-----------|-----------|----------------|----------|")
    for m in maps:
        env = get_env(m)
        start = (0, 0)
        goal = (env.rows - 1, env.cols - 1)
        for alg in algorithms:
            if m == 'dynamic':
                path, cost, exp, t = simulate_unpredictable(env, start, goal)
            else:
                if alg == 'ucs':
                    path, cost, exp, t = ucs(env, start, goal)
                elif alg == 'a_star':
                    path, cost, exp, t = a_star(env, start, goal)
                else:
                    path, cost, exp, t = sa_with_restarts(env, start, goal)
            print(f"| {m} | {alg} | {cost} | {exp} | {t:.4f} |")

parser = argparse.ArgumentParser()
parser.add_argument('--map', required=False)
parser.add_argument('--start', nargs=2, type=int)
parser.add_argument('--goal', nargs=2, type=int)
parser.add_argument('--algorithm', choices=['ucs', 'a_star', 'sa'])
parser.add_argument('--dynamic_mode', choices=['none', 'known', 'unpredictable'], default='none')
parser.add_argument('--experiment', action='store_true')
parser.add_argument('--demo', action='store_true')
args = parser.parse_args()

if args.experiment:
    run_experiment()
elif args.demo:
    env = get_env(args.map)
    run_demo(env, tuple(args.start), tuple(args.goal), args.dynamic_mode)
else:
    env = get_env(args.map)
    if args.dynamic_mode == 'unpredictable' and args.map == 'dynamic':
        path, cost, exp, t = simulate_unpredictable(env, tuple(args.start), tuple(args.goal))
    else:
        known = args.dynamic_mode == 'known' and args.map == 'dynamic'
        space_time = known
        if args.algorithm == 'ucs':
            path, cost, exp, t = ucs(env, tuple(args.start), tuple(args.goal), space_time, known)
        elif args.algorithm == 'a_star':
            path, cost, exp, t = a_star(env, tuple(args.start), tuple(args.goal), space_time, known)
        else:
            path, cost, exp, t = sa_with_restarts(env, tuple(args.start), tuple(args.goal))
    if path:
        print(f"Path: {path}")
        print(f"Cost: {cost}, Expanded: {exp}, Time: {t:.4f}")
    else:
        print("No path found")
