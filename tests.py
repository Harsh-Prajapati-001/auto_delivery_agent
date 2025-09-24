from planners import a_star
from environment import Environment

# Simple test
grid = [[1, 1], [1, 1]]
env = Environment(grid, 2, 2)
path, cost, _, _ = a_star(env, (0, 0), (1, 1))
assert path == [(0, 0), (0, 1), (1, 1)] or path == [(0, 0), (1, 0), (1, 1)], "Path test failed"
assert cost == 2, "Cost test failed"
print("Tests passed")
