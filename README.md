## Project Description
CSA2001 - Fundamentals of AI and ML

Project Based Learning
Project 1

Design and implement an autonomous delivery agent that navigates a 2D grid city to deliver packages. The agent must:
- Model the environment (static obstacles, varying terrain costs, dynamic moving obstacles).
- Be rational: choose actions that maximize delivery efficiency under constraints (time, fuel).
- Implement uninformed (BFS/Uniform-cost), informed (A* with admissible heuristic), and a local search replanning strategy (e.g., hill-climbing with random restarts or simulated annealing) to handle dynamic obstacles / changing traffic costs.
- Compare algorithms experimentally on several map instances and report results (path cost, nodes expanded, time).
- Provide analysis describing when each method performs better and why.

Required deliverables
- Source code (well documented). Preferably Python (you may choose another language) with CLI to run each planner.
  - Encourage students to commit code to Git with README with instructions and dependencies. Tests and reproducibility are required.
  - Required at least one proof-of-concept of dynamic replanning (log showing obstacle appears and agent replans).
  - At least 4 test maps: small, medium, large, and one with dynamic obstacles (moving vehicles). Include grid file format.
- A short report (max 6 pages) containing: environment model, agent design, heuristics used, experimental results (tables + short plots), analysis and conclusion.
- A short recorded demo (5 min) or sequence of screenshots showing agents acting on dynamic map.

Constraints / assumptions
- Grid cells have integer movement cost ≥ 1 (different terrains).
- Moving obstacles occupy cells and move deterministically according to a known schedule (so agent can plan knowing future positions for one horizon) or unpredictably (for local search testing).
- Agent can move 4-connected (up/down/left/right). Diagonals optional (state in report).

## Installation
1. Clone the repository:git clone https://github.com/yourusername/autonomous-delivery-agent.git
cd autonomous-delivery-agent
2. Install dependencies (standard Python libraries, no external needed):pip install -r requirements.txt
3. (The file is empty since only built-in modules are used: heapq, random, math, time, argparse.)

## Usage
Run the planners via CLI:python main.py --map <map_name> --start <x> <y> --goal <x> <y> --algorithm <alg> [--dynamic_mode <mode>]
4. - `--map`: small, medium, large, dynamic
- `--start` and `--goal`: integers for row col (0-indexed)
- `--algorithm`: ucs, a_star, sa
- `--dynamic_mode`: none (default, static), known (space-time for deterministic), unpredictable (replan with SA)

Example:python main.py --map small --start 0 0 --goal 4 4 --algorithm a_star
5. For experiments (runs all algorithms on all maps, prints tables):python main.py --experiment
6. For demo (simulates movement on dynamic map with prints for screenshots):python main.py --demo --map dynamic --start 0 0 --goal 9 9 --dynamic_mode unpredictable
7.
## Reproducibility
Maps are hardcoded for simplicity. Random seed is not set for SA, but runs are reproducible by design (deterministic for UCS/A*).

## Dynamic Replanning Proof
When running with `--dynamic_mode unpredictable` on dynamic map, if replanning occurs, a `replan_log.txt` is generated showing time, agent position, and obstacle position.

## Report
See `report_template.md` for a template. Run `--experiment` to get results, then fill in.

