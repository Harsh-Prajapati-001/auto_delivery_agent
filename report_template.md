# Project Report

## Environment Model
2D grid with costs >=1, static obstacles (inf), dynamic moving (periodic cycle).

## Agent Design
Rational: Minimizes fuel cost (terrain sum), allows wait in space-time.

## Heuristics
Manhattan distance *1 (admissible).

## Experimental Results
[Run `python main.py --experiment` and paste tables here]

## Analysis
UCS expands more nodes in large maps. A* better with heuristic. SA for dynamic unpredictable, faster replan but suboptimal. UCS/A* better for static/known, SA for changing costs (stochastic escape locals).

## Conclusion
[Your conclusions]

(Note: No diagonals used.)
