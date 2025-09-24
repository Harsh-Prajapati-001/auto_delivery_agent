import itertools

class Environment:
    def __init__(self, grid, rows, cols, dynamic_obstacles=None):
        self.grid = grid  # list of lists, inf for static obstacles
        self.rows = rows
        self.cols = cols
        self.dynamic_obstacles = dynamic_obstacles or []

    def is_valid(self, x, y):
        return 0 <= x < self.rows and 0 <= y < self.cols and self.grid[x][y] != float('inf')

    def get_neighbors(self, x, y):
        dirs = [(-1, 0), (1, 0), (0, -1), (0, 1)]
        return [(x + dx, y + dy) for dx, dy in dirs if self.is_valid(x + dx, y + dy)]

    def is_blocked_at(self, x, y, t):
        for obs in self.dynamic_obstacles:
            ox, oy = obs.position_at(t)
            if (x, y) == (ox, oy):
                return True
        return False

class MovingObstacle:
    def __init__(self, positions):
        self.positions = list(itertools.cycle(positions))  # cycle for periodic movement

    def position_at(self, t):
        return self.positions[t]
