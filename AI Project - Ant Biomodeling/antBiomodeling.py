"""
===============================================================================
 File:        antBiomodeling.py
 Author:      Dom Scordino
 Created:     2025-04-13
 Description:
   Simulates a swarm of autonomous "ant" robots exploring the Moon's surface
   to locate and retrieve an ice deposit. Each ant:
     - Moves with randomized direction adjustments.
     - Leaves behind microdot markers as it explores.
     - Picks up ice when within detection range.
     - Returns along its original path to deliver ice back to the lander.
     - Marks its return path as a goal trail others can follow.
===============================================================================
"""

import math
import random
import matplotlib.pyplot as plt

# -----------------------------------------------------------------------------
# Simulation Constants
# -----------------------------------------------------------------------------

NUM_ANTS = 20                         # Number of autonomous ants in the swarm
MAX_DISTANCE = 100                   # Max distance each ant can travel from lander (in meters)
MAX_STEPS = 1000                     # Safety cap on steps per ant
ANT_SPEED = 1                        # Distance covered per step (in meters)
ICE_LOCATION = (9, -9)             # (x, y) coordinates of the ice deposit
ICE_RADIUS = 1.0                     # Radius of the ice deposit
ICE_DETECTION_RANGE = 4             # Detection buffer range around the ice
DEGREES = [-3, 0, 3]                 # Angular deviations from current heading (degrees)
LOG_FILENAME = "ant_movements.log"  # File to record ant behavior

# -----------------------------------------------------------------------------
# Classes
# -----------------------------------------------------------------------------

class Microdot:
    """
    Represents a microdot marker dropped by an ant during exploration.
    These serve as breadcrumbs and are upgraded to goal trail markers after
    an ant discovers ice and returns.
    """
    def __init__(self, pos, is_goal=False):
        self.pos = pos                  # (x, y) coordinates
        self.is_goal = is_goal          # True if part of a goal trail
        self.to_ice = None              # Vector or hint to ice location
        self.to_lander = None           # Vector or hint back to lander


class Ant:
    """
    Models an autonomous ant exploring for ice and returning it to base.
    Ants use a randomized movement pattern and retrace their own paths after
    discovering ice. The first discoverer creates a goal trail others could use.
    """
    def __init__(self, id, lander_pos=(0, 0)):
        self.id = id
        self.lander_pos = lander_pos
        self.pos = lander_pos
        self.path = [lander_pos]             # Path taken during exploration
        self.retrieval_path = []             # Reverse of path to ice
        self.microdots = []                  # Markers dropped along path
        self.goal_path = []                  # Return path, marked as goal trail
        self.direction = random.randint(0, 359)
        self.distance_traveled = 0
        self.mode = "search"                 # 'search', 'return', 'idle', or 'lost'
        self.is_carrying_ice = False
        self.has_delivered_ice = False
        self.lost = False

    def move(self, log_file):
        """
        Perform one unit of movement in the current direction.
        Updates position, drops a microdot, checks for ice detection.
        """
        if self.mode in ["lost", "return", "idle"]:
            return

        # Abort if too far or out of range
        if self.distance_traveled >= MAX_DISTANCE:
            if self.distance_from(ICE_LOCATION) > 100:
                self.mode = "lost"
                self.lost = True
            return

        # Slight angular deviation (left, right, or straight)
        angle_change = random.choice(DEGREES)
        self.direction = (self.direction + angle_change) % 360
        rad = math.radians(self.direction)

        # Compute new (x, y) position
        dx = ANT_SPEED * math.cos(rad)
        dy = ANT_SPEED * math.sin(rad)
        new_pos = (self.pos[0] + dx, self.pos[1] + dy)

        # Update state
        self.pos = new_pos
        self.path.append(new_pos)
        self.microdots.append(Microdot(new_pos))
        self.distance_traveled += ANT_SPEED
        log_file.write(f"Ant {self.id} moved to {self.pos}\n")

        # Check for ice detection
        if self.detect_ice():
            self.pickup_ice(log_file)

        # Mark as lost if wandered too far from ice
        if self.distance_from(ICE_LOCATION) > 100:
            self.mode = "lost"
            self.lost = True
            log_file.write(f"Ant {self.id} is lost\n")

    def move_home(self):
        """
        Follows the ant's recorded path back to the lander.
        Called after the ant has picked up ice.
        """
        if self.mode != "return":
            return

        if not self.retrieval_path:
            self.pos = self.lander_pos
            self.deliver_ice()
            return

        self.pos = self.retrieval_path.pop(0)
        self.path.append(self.pos)

    def detect_ice(self):
        """
        Returns True if ant is within detection range of the ice deposit.
        """
        return self.distance_from(ICE_LOCATION) <= (ICE_RADIUS + ICE_DETECTION_RANGE)

    def pickup_ice(self, log_file):
        """
        Triggers when an ant reaches the ice. Sets return mode and builds retrieval path.
        """
        log_file.write(f"Ant {self.id} picked up ice at {self.pos}\n")
        self.is_carrying_ice = True
        self.mode = "return"
        self.retrieval_path = list(reversed(self.path))
        self.mark_goal_trail()

    def deliver_ice(self):
        """
        Completes the ant’s mission by delivering the ice to the lander.
        """
        if self.is_carrying_ice and self.pos == self.lander_pos:
            self.has_delivered_ice = True
            self.is_carrying_ice = False
            self.mode = "idle"

    def mark_goal_trail(self):
        """
        Converts the retrieval path into a visible goal trail for visualization
        or future extension into collaborative trail-following behavior.
        """
        for pos in self.retrieval_path:
            dot = Microdot(pos, is_goal=True)
            dot.to_ice = ICE_LOCATION
            dot.to_lander = self.lander_pos
            self.goal_path.append(dot)

    def distance_from(self, point):
        """
        Utility method to compute Euclidean distance from current position.
        """
        return math.dist(self.pos, point)


# -----------------------------------------------------------------------------
# Simulation Execution
# -----------------------------------------------------------------------------

# Initialize all ant agents
ants = [Ant(i) for i in range(NUM_ANTS)]

with open(LOG_FILENAME, "w") as log_file:
    # Search Phase
    for ant in ants:
        steps = 0
        while ant.mode == "search" and steps < MAX_STEPS:
            ant.move(log_file)
            steps += 1

    log_file.write("=== Beginning return phase ===\n")

    # Return Phase
    returning = [ant for ant in ants if ant.mode == "return"]
    if returning:
        max_return_steps = max(len(a.retrieval_path) for a in returning)
        for _ in range(max_return_steps):
            for ant in returning:
                if ant.retrieval_path:
                    ant.move_home()
                    log_file.write(f"Ant {ant.id} returning via {ant.pos}\n")
                elif ant.pos == ant.lander_pos and not ant.has_delivered_ice:
                    ant.deliver_ice()
                    log_file.write(f"Ant {ant.id} delivered ice to lander\n")
    else:
        log_file.write("No ants discovered the ice.\n")


# -----------------------------------------------------------------------------
# Visualization
# -----------------------------------------------------------------------------

plt.figure(figsize=(10, 8))

for ant in ants:
    x, y = zip(*ant.path)
    if ant.has_delivered_ice:
        plt.plot(x, y, label=f'Ant {ant.id} (Delivered Ice)', linewidth=2)
    elif ant.lost:
        plt.plot(x, y, '--', label=f'Ant {ant.id} (Lost)', linewidth=1)
    else:
        plt.plot(x, y, label=f'Ant {ant.id}', linewidth=1)

# Visualize goal trails (for ants that found the ice)
for ant in ants:
    if ant.goal_path:
        x_goal = [dot.pos[0] for dot in ant.goal_path]
        y_goal = [dot.pos[1] for dot in ant.goal_path]
        plt.plot(x_goal, y_goal, 'k--', alpha=0.5)

# Mark lander and ice location
plt.scatter([0], [0], color='black', label='Lander')
ice_circle = plt.Circle(ICE_LOCATION, ICE_RADIUS, color='blue', alpha=0.5, label='Ice Deposit')
plt.gca().add_patch(ice_circle)

plt.title('Ant Ice Retrieval Simulation')
plt.xlabel('X Position (m)')
plt.ylabel('Y Position (m)')
plt.legend()
plt.grid(True)
plt.axis('equal')
plt.show()

"""
===============================================================================
 End of File: antBiomodeling.py
===============================================================================
"""
