# ASSUMPTIONS MADE:
# The things I had to assume for this assignment are that:
# if an ant discovers the Ice, it does not pick up the ice (it puts the ice's location on the goal path and goes home)
# there are no obstacles on the land because the instructions never said where to put the obstacles
# the minutes it would take to hunt for the ice and come back to the lander is equal to the meters traveled

import random
import math
import time
from collections import deque
import threading
import matplotlib.pyplot as plt

global goalMicroDots
global IceFinder
global returnHomeMicroDots


class MicroDot:
	"""
	Represents a coordinate point in 2D space that an ant visits.

	Attributes:
	xPosition (float): The x-coordinate of the microdot.
	yPosition (float): The y-coordinate of the microdot.

	Used for:
	- Storing path history of ants.
	- Defining waypoints for navigation.
	"""

	def __init__(self, xPosition, yPosition):
		self.xPosition = xPosition
		self.yPosition = yPosition


class Lander:
	"""
	Represents the base or home location for all ants.

	Attributes:
	    xPosition (int): The fixed x-coordinate of the lander (always 0).
	    yPosition (int): The fixed y-coordinate of the lander (always 0).
	    ants (list): A list of Ant objects deployed from this lander.

	Used for:
	    - The origin and return destination for ants.
	    - Housing and launching ants into the simulation.
	"""

	def __init__(self):
		self.xPosition = 0
		self.yPosition = 0
		self.ants = []


class Ice:
	"""
	Represents the ice resource that ants must find and report.

	Attributes:
	    xPosition (float): The x-coordinate of the ice.
	    yPosition (float): The y-coordinate of the ice.
	    radius (int): The detection radius within which ants can discover the ice.
	    quantity (int): The number of ice pieces available to be collected.
	    found (int): Flag indicating whether the ice has already been discovered (0 = no, 1 = yes).

	Used for:
	    - Ants to detect and report the location.
	    - Triggering goal path behavior in the simulation.
	"""

	def __init__(self, xPosition=20, yPosition=-30):
		self.xPosition = xPosition
		self.yPosition = yPosition
		self.radius = 1
		self.quantity = ICE_QUANTITY
		self.found = 0


class Ant:
	"""
	Represents an autonomous agent (ant) that searches for ice and returns to the lander.

	Attributes:
	    xPosition (float): The ant's current x-coordinate.
	    yPosition (float): The ant's current y-coordinate.
	    id (int): Unique identifier for the ant.
	    direction (float): Current angle (in degrees) that the ant is facing.
	    distanceTravelled (int): Total number of moves taken by the ant.
	    pathHistory (list of MicroDot): Trail of microdots showing the ant's movement history.
	    returnPath (list of MicroDot): Reverse path generated when returning to the lander.

	    # Behavior Flags (mode values 0 or 1):
	    huntingForIceOffPathMode (int): Random search mode.
	    onGoalPathToIceMode (int): Ant is following known path to ice.
	    discoveredIceMode (int): Ant just discovered the ice and is about to mark it.
	    obtainingIceMode (int): Ant is navigating directly to the ice to collect it.
	    goingHomeWithIceMode (int): Ant is returning to the lander with collected ice.
	    goingHomeWithoutIceMode (int): Ant is returning to the lander without ice.
	    needsToRechargeMode (int): Placeholder for future energy exhaustion feature.
	    isHomeMode (int): Indicates if the ant has finished its journey and returned home.
	    hasIceMode (int): Whether the ant currently possesses ice.
	    iceFinder (int): Flag to indicate if this ant was the first to discover the ice.

	Used for:
	    - Navigating the simulation space.
	    - Interacting with the ice resource and environment.
	    - Coordinating shared behavior through goal paths and discovery logic.
	"""

	def __init__(self, xPosition, yPosition, id):
		self.xPosition = xPosition
		self.yPosition = yPosition
		self.id = id
		self.direction = 0
		self.distanceTravelled = 0
		self.pathHistory = []
		self.returnPath = []
		self.huntingForIceOffPathMode = 1
		self.onGoalPathToIceMode = 0
		self.discoveredIceMode = 0
		self.obtainingIceMode = 0
		self.goingHomeWithIceMode = 0
		self.goingHomeWithoutIceMode = 0
		self.needsToRechargeMode = 0
		self.isHomeMode = 0
		self.hasIceMode = 0
		self.iceFinder = 0

	def setModes(self, huntingForIceOffPathMode, onGoalPathToIceMode, discoveredIceMode, obtainingIceMode, goingHomeWithIceMode, goingHomeWithoutIceMode, needsToRechargeMode, isHomeMode):
		"""
		Sets the behavioral mode flags for the ant.

		Parameters:
		    huntingForIceOffPathMode (int): 1 if the ant is randomly searching for ice.
		    onGoalPathToIceMode (int): 1 if the ant is following a known path to the ice.
		    discoveredIceMode (int): 1 if the ant has just discovered the ice.
		    obtainingIceMode (int): 1 if the ant is currently going to collect ice.
		    goingHomeWithIceMode (int): 1 if the ant is returning home with ice.
		    goingHomeWithoutIceMode (int): 1 if the ant is returning home without ice.
		    needsToRechargeMode (int): 1 if the ant needs to recharge (not used in logic).
		    isHomeMode (int): 1 if the ant has completed its mission and returned home.

		Purpose:
		    Updates all internal mode flags to reflect the ant’s current behavioral state.
		"""

		self.huntingForIceOffPathMode = huntingForIceOffPathMode
		self.onGoalPathToIceMode = onGoalPathToIceMode
		self.discoveredIceMode = discoveredIceMode
		self.obtainingIceMode = obtainingIceMode
		self.goingHomeWithIceMode = goingHomeWithIceMode
		self.goingHomeWithoutIceMode = goingHomeWithoutIceMode
		self.needsToRechargeMode = needsToRechargeMode
		self.isHomeMode = isHomeMode

	def move(self, lock, positions):
		"""
		Moves the ant in a random direction and records its position.

		Parameters:
		    lock (threading.Lock): A lock used to ensure thread-safe access to shared data.
		    positions (dict): A shared dictionary tracking each ant's movement coordinates.

		Behavior:
		    - Turns randomly by ±5 degrees or not at all.
		    - Moves one unit in the current direction.
		    - Appends current position to path history and shared position log.
		"""

		TURN_SIZE = 5
		self.direction += random.choice([-TURN_SIZE, 0, TURN_SIZE])  # this is how the ants should move per assignment description
		if self.direction > 360:
			self.direction -= 360
		self.xPosition += self.distanceTravelled * math.cos(self.direction)
		self.yPosition += self.distanceTravelled * math.sin(self.direction)
		self.pathHistory.append(MicroDot(self.xPosition, self.yPosition))  # records microdot to path history
		with lock:  # lock the positions list
			positions[self.id].append((self.xPosition, self.yPosition))  # adds ant position to positions list
			print(f"Ant {self.id} is at {self.xPosition}, {self.yPosition}\n")  # prints ant position
		self.distanceTravelled += 1

	def moveToPoint(self, xGoalDot, yGoalDot, lock, positions):  # moves ant to goal dot
		"""
		Moves the ant directly to a specified coordinate point.

		Parameters:
		    xGoalDot (float): The x-coordinate of the target point.
		    yGoalDot (float): The y-coordinate of the target point.
		    lock (threading.Lock): A lock used to ensure thread-safe access to shared data.
		    positions (dict): A shared dictionary tracking each ant's movement coordinates.

		Behavior:
		    - Rotates the ant to face the target.
		    - Updates the ant’s position directly to the given point.
		    - Records the position in both local and shared history.
		"""

		self.turnToPoint(xGoalDot, yGoalDot)  # turn ant towards goal dot
		self.xPosition = xGoalDot  # move your xPosition to the goal dot
		self.yPosition = yGoalDot  # move your yPosition to the goal dot
		self.pathHistory.append(MicroDot(self.xPosition, self.yPosition))  # records microdot to path history
		with lock:
			positions[self.id].append((self.xPosition, self.yPosition))  # adds ant position to positions list
			print(f"Ant {self.id} is at {self.xPosition}, {self.yPosition}\n")  # prints ant position
		self.distanceTravelled += 1  # add to accumulator

	def turnToPoint(self, xNext, yNext):
		"""
		Rotates the ant to face toward a specific point in space.

		Parameters:
		    xNext (float): The x-coordinate of the point to face.
		    yNext (float): The y-coordinate of the point to face.

		Behavior:
		    - Calculates the directional angle between the ant's current location and the target.
		    - Adjusts the ant’s `direction` to point toward that location.
		"""

		xCurrent = self.xPosition  # set the x position of the ant to temporary variable
		yCurrent = self.yPosition  # set the y position of the ant to temporary variable
		currentAngle = math.degrees(math.atan2(yCurrent, xCurrent))  # find the current angle of the ant
		targetAngle = math.degrees(math.atan2(yNext - yCurrent, xNext - xCurrent))  # find the angle between the direction the ant is facing and the ice
		angleDifference = (targetAngle - currentAngle + 180) % 360 - 180  # find the difference between the target angle and the current angle
		if angleDifference > 0:
			self.direction += angleDifference  # turn right towards the target location (xNext, yNext)
		else:
			self.direction -= angleDifference  # turn left towards the target location (xNext, yNext)

	def dropGoalMicroDot(self):
		"""
		Drops a microdot at the ant's current position for path tracing.

		Effects:
		    - Appends the current location to both `goalMicroDotsToIce` and `goalMicroDotsToHome`.
		    - Also records the location in the ant's own `pathHistory`.

		Purpose:
		    Marks significant locations that can later be used for goal-oriented navigation by other ants.
		"""

		goalMicroDotsToIce.append(MicroDot(self.xPosition, self.yPosition))  # adds ice to goal microdot stack
		goalMicroDotsToHome.append(MicroDot(self.xPosition, self.yPosition))  # adds ant to goal microdot to home list
		self.pathHistory.append(MicroDot(self.xPosition, self.yPosition))  # records microdot to path history

	def goingHome(self):
		"""
		Navigates the ant back to the lander, either with or without ice.

		Behavior:
		    - If the ant has no ice, it retraces its own path in reverse to return.
		    - If the ant has ice, it follows the shared `goalMicroDotsToHome` path.
		    - Each movement step is recorded and synchronized with the global position log.

		Effects:
		    - Sets the `isHomeMode` flag to 1 once the ant reaches the lander.
		    - Updates mode flags accordingly to reflect completion of the journey.
		"""

		if self.hasIceMode == 0:  # if ant does not have ice
			print(f"Ant {self.id} is going home without Ice\n")
			if not self.returnPath:  # if return path is empty
				self.returnPath = self.pathHistory[::-1]  # create return path from path history
				self.returnPath.append(MicroDot(Lander.xPosition, Lander.yPosition))  # add Lander to return path
				self.turnToPoint(self.returnPath[1].xPosition, self.returnPath[1].yPosition)  # turn ant around to previous dot
			for dot in range(len(self.returnPath)):  # for each dot in the return microdot stack
				if self.xPosition == self.returnPath[dot].xPosition and self.yPosition == self.returnPath[dot].yPosition:  # if you're at a return dot
					if dot + 1 < len(self.returnPath):  # if you're not at the last return dot
						self.moveToPoint(self.returnPath[dot + 1].xPosition, self.returnPath[dot + 1].yPosition, lock, positions)  # moves ant to next dot
					else:
						self.moveToPoint(Lander.xPosition, Lander.yPosition, lock, positions)  # move to the Lander
						print(f"Ant {self.id} has returned home\n")
						self.setModes(0, 0, 0, 0, 0, 0, 0, 1)  # set modes to 0 except isHomeMode = 1
						break
		else:
			print(f"Ant {self.id} is going home with Ice\n")
			for dot in range(len(goalMicroDotsToHome)):  # for each dot in the return microdot stack
				if self.xPosition == goalMicroDotsToHome[dot].xPosition and self.yPosition == goalMicroDotsToHome[dot].yPosition:  # if you're at a return dot
					if dot + 1 < len(goalMicroDotsToHome):
						self.moveToPoint(goalMicroDotsToHome[dot + 1].xPosition, goalMicroDotsToHome[dot + 1].yPosition, lock, positions)  # moves ant to next dot
					else:
						self.moveToPoint(Lander.xPosition, Lander.yPosition, lock, positions)  # move to the Lander
						print(f"Ant {self.id} has returned home\n")
						self.setModes(0, 0, 0, 0, 0, 0, 0, 1)  # set modes to 0 except isHomeMode = 1
						break


def antMovement(Ant, lock, positions):
	"""
	Simulates the full behavioral lifecycle of a single ant in the environment.

	This function governs how an ant:
	- Randomly explores the environment in search of ice.
	- Detects the ice and records its location.
	- Follows known goal paths to reach the ice.
	- Collects ice and returns home using either its own path or a shared goal path.
	- Logs its position in a shared dictionary for visualization and tracking.

	Parameters:
	    Ant (Ant): The ant object to simulate.
	    lock (threading.Lock): A threading lock used to safely update shared data structures.
	    positions (dict): A shared dictionary mapping ant IDs to their list of visited (x, y) coordinates.

	Behavior Modes:
	    - huntingForIceOffPathMode: Randomly walks in search of ice.
	    - discoveredIceMode: First discoverer drops a microdot and returns.
	    - onGoalPathToIceMode: Follows known path toward the ice.
	    - obtainingIceMode: Collects ice if available.
	    - goingHomeWithIceMode: Returns home along shared goal path.
	    - goingHomeWithoutIceMode: Returns home by retracing path history.

	Loop Exit:
	    - Simulation ends for the ant when isHomeMode is set to 1.

	Effects:
	    - Updates global path structures (goalMicroDotsToIce and goalMicroDotsToHome).
	    - Decrements Ice.quantity when ice is collected.
	    - Appends positional data to the shared `positions` dictionary for plotting.
	"""

	goalMicroDotsToHome = list(goalMicroDotsToIce)[::-1]
	while Ant.isHomeMode == 0:
		Ant.direction = random.randint(0, 360)
		Ant.move(lock, positions)
		while Ant.distanceTravelled < 20 or (Ant.xPosition != 0 or Ant.yPosition != 0) and Ant.isHomeMode == 0 and Ant.pathHistory != []:
			while Ant.distanceTravelled < 500:
				if Ant.huntingForIceOffPathMode == 1:  # if ant is hunting for ice off path
					Ant.move(lock, positions)  # moves ant in a random direction
					distanceToIce = math.hypot(Ant.xPosition - Ice.xPosition, Ant.yPosition - Ice.yPosition)  # checks if ant is in range of ice
					if distanceToIce <= Ice.radius + 10:  # if ant is in range of ice
						if Ice.found == 0:  # if ice has not been found yet
							Ice.found = 1  # set ice.found to 1
							Ant.setModes(0, 0, 1, 0, 0, 0, 0, 0)  # huntingForIceOffPathMode = 0 and discoveredIceMode = 1
							Ant.iceFinder = 1  # set iceFinder to 1
						else:  # if ice has been found already
							Ant.obtainingIceMode = 1  # obtainingIceMode = 1 (gets ice, returns to spot where it detected ice, and adds it on goal return path)
					else:  # if not in range of ice
						for dot in range(len(goalMicroDotsToIce)):  # checks if ant is in range of goal dot
							distanceToGoalDot = math.hypot(Ant.xPosition - goalMicroDotsToIce[dot].xPosition, Ant.yPosition - goalMicroDotsToIce[dot].yPosition)
							if distanceToGoalDot <= 2:  # if in range of goal dot
								Ant.moveToPoint(goalMicroDotsToIce[dot].xPosition, goalMicroDotsToIce[dot].yPosition)  # moves ant to goal dot
								Ant.onGoalPathToIceMode = 1  # onGoalPathToIceMode = 1 (turns ant to path, puts ant on the path)
							else:  # if not in range of goal dot
								Ant.huntingForIceOffPathMode = 1  # huntingForIceOffPathMode = 1 (moves ant in a random direction)

				if Ant.discoveredIceMode == 1:
					Ant.turnToPoint(Ant.pathHistory[-2].xPosition, Ant.pathHistory[-2].yPosition)  # turn ant around to previous dot
					goalMicroDotsToIce.append(MicroDot(20, -30))  # adds ice to goal microdot stack
					goalMicroDotsToHome.append(MicroDot(20, -30))  # adds dot to goal microdot to home list
					Ant.dropGoalMicroDot()  # adds ice to goal microdot stack
					Ant.setModes(0, 0, 0, 0, 0, 1, 0, 0)  # discoveredIceMode = 0 and goingHomeWithoutIceMode = 1
					print(f"Ant {Ant.id} has discovered Ice\n")

				if Ant.onGoalPathToIceMode == 1:
					for dot in range(len(goalMicroDotsToIce)):
						if Ant.xPosition != goalMicroDotsToIce[-1].xPosition and Ant.yPosition != goalMicroDotsToIce[-1].yPosition:  # if you are at a goal dot, and not at the last goal dot
							Ant.moveToPoint(goalMicroDotsToIce[dot + 1].xPosition, goalMicroDotsToIce[dot + 1].yPosition)  # moves ant to goal dot
							print(f"Ant {Ant.id} is moving onto Goal Path\n")
						else:  # if you are at the last goal dot (ice @ 20,-30)
							Ant.setModes(0, 0, 0, 1, 0, 0, 0, 0)  # set modes to 0 except obtainingIceMode = 1

				if Ant.obtainingIceMode == 1:
					Ant.turnToPoint(20, -30)  # turn towards the ice
					Ant.moveToPoint(20, -30, lock, positions)  # move to the ice
					Ant.hasIceMode = 1  # obtain Ice and set hasIceMode to 1
					print(f"Ant {Ant.id} has obtained Ice\n")
					Ant.obtainingIceMode = 0  # set obtainingIceMode to 0
					Ice.quantity -= 1  # subtract 1 from the ice quantity
					if len(goalMicroDotsToHome) > 1:  # Check if there are at least 2 elements
						Ant.turnToPoint(goalMicroDotsToHome[1].xPosition, goalMicroDotsToHome[1].yPosition)  # turn around to your last dot that is not the ice
						Ant.moveToPoint(goalMicroDotsToHome[1].xPosition, goalMicroDotsToHome[1].yPosition, lock, positions)  # move to the previous dot
					Ant.setModes(0, 0, 0, 0, 1, 0, 0, 0)  # set modes to 0 except goingHomeWithIceMode = 1

				if Ant.goingHomeWithIceMode == 1:
					if Ant.xPosition == goalMicroDotsToHome[-1].xPosition and Ant.yPosition == goalMicroDotsToHome[-1].yPosition:  # if you're at the last return dot
						Ant.setModes(0, 0, 0, 0, 0, 0, 0, 1)  # set modes to 0 except isHomeMode = 1
					else:
						for dot in range(len(goalMicroDotsToHome)):  # for each dot in the return microdot stack
							if Ant.xPosition == goalMicroDotsToHome[dot].xPosition and Ant.yPosition == goalMicroDotsToHome[dot].yPosition and Ant.xPosition != Lander.xPosition and Ant.yPosition != Lander.yPosition:  # if you're at a return dot and not at the last return dot
								Ant.moveToPoint(goalMicroDotsToHome[dot + 1].xPosition, goalMicroDotsToHome[dot + 1].yPosition)  # moves ant to goal dot
							else:
								Ant.moveToPoint(Lander.xPosition, Lander.yPosition)  # moves ant to home
								Ant.isHomeMode = 1  # set isHomeMode to 1

				if Ant.goingHomeWithoutIceMode == 1:
					Ant.goingHome()
					break
			else:
				Ant.goingHome()


ICE_QUANTITY = 9  # number of ice pieces
Lander = Lander()  # create Lander object
Ice = Ice()  # create Ice object
for i in range(1, ICE_QUANTITY + 1, 1):  # create Ant objects
	Lander.ants.append(Ant(0, 0, i))  # add Ant objects to Lander object
goalMicroDotsToIce = deque()  # create goal microdot stack
goalMicroDotsToHome = []  # create goal microdot to home list

lock = threading.Lock()  # create lock to lock positions list while adding a position to it
positions = {ant.id: [(ant.xPosition, ant.yPosition)] for ant in Lander.ants}  # create positions list
threads = []  # create threads list of ants to move
for Ant in Lander.ants:  # for each ant in the Lander object
	thread = threading.Thread(target=antMovement, args=(Ant, lock, positions))  # create a thread for the ant
	threads.append(thread)  # add the thread to the threads list
	thread.start()  # start the thread

for thread in threads:
	thread.join()

# Plotting the movements
plt.figure()
for ant_id, pos in positions.items():
	x, y = zip(*pos)
	plt.plot(x, y, label=f'Ant {ant_id}')
# Plot the Lander's position
plt.scatter(Lander.xPosition, Lander.yPosition, color='red', marker='o', s=100, label='Lander')

# Plot the Ice's position
plt.scatter(Ice.xPosition, Ice.yPosition, color='blue', marker='x', s=100, label='Ice')

plt.xlabel('X Position')
plt.ylabel('Y Position')
plt.title('Ant Movements')
plt.legend()
plt.legend(bbox_to_anchor=(1, 1), loc='upper left')  # Move the legend to the top left corner
plt.show()
print(f"Ice pieces remaining: {Ice.quantity}")
print("All ants are home")
