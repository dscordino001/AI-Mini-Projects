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
  def __init__(self, xPosition, yPosition):
    self.xPosition = xPosition
    self.yPosition = yPosition


class Lander:
  def __init__(self):
    self.xPosition = 0
    self.yPosition = 0
    self.ants = []

class Ice:
  def __init__(self, xPosition=20, yPosition=-30):
    self.xPosition = xPosition
    self.yPosition = yPosition
    self.radius = 1
    self.quantity = ICE_QUANTITY
    self.found = 0


class Ant:
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

  def setModes(self, huntingForIceOffPathMode, onGoalPathToIceMode, discoveredIceMode,
               obtainingIceMode, goingHomeWithIceMode, goingHomeWithoutIceMode, needsToRechargeMode,
               isHomeMode):
    self.huntingForIceOffPathMode = huntingForIceOffPathMode
    self.onGoalPathToIceMode = onGoalPathToIceMode
    self.discoveredIceMode = discoveredIceMode
    self.obtainingIceMode = obtainingIceMode
    self.goingHomeWithIceMode = goingHomeWithIceMode
    self.goingHomeWithoutIceMode = goingHomeWithoutIceMode
    self.needsToRechargeMode = needsToRechargeMode
    self.isHomeMode = isHomeMode

  def move(self, lock, positions):
    TURN_SIZE = 5
    self.direction += random.choice([-TURN_SIZE, 0, TURN_SIZE]) # this is how the ants should move per assignment description
    if self.direction > 360:
      self.direction -= 360
    self.xPosition += self.distanceTravelled * math.cos(self.direction)
    self.yPosition += self.distanceTravelled * math.sin(self.direction)
    self.pathHistory.append(MicroDot(self.xPosition, self.yPosition))  # records microdot to path history
    with lock: # lock the positions list
      positions[self.id].append((self.xPosition, self.yPosition)) # adds ant position to positions list
      print(f"Ant {self.id} is at {self.xPosition}, {self.yPosition}\n") # prints ant position
    self.distanceTravelled += 1

  def moveToPoint(self, xGoalDot, yGoalDot, lock, positions):  # moves ant to goal dot
    self.turnToPoint(xGoalDot, yGoalDot)  # turn ant towards goal dot
    self.xPosition = xGoalDot  # move your xPosition to the goal dot
    self.yPosition = yGoalDot  # move your yPosition to the goal dot
    self.pathHistory.append(MicroDot(self.xPosition, self.yPosition))  # records microdot to path history
    with lock:
      positions[self.id].append((self.xPosition, self.yPosition)) # adds ant position to positions list
      print(f"Ant {self.id} is at {self.xPosition}, {self.yPosition}\n") # prints ant position
    self.distanceTravelled += 1  # add to accumulator

  def turnToPoint(self, xNext, yNext):
    xCurrent = self.xPosition # set the x position of the ant to temporary variable
    yCurrent = self.yPosition # set the y position of the ant to temporary variable
    currentAngle = math.degrees(math.atan2(yCurrent, xCurrent)) # find the current angle of the ant
    targetAngle = math.degrees(math.atan2(yNext - yCurrent, xNext - xCurrent)) # find the angle between the direction the ant is facing and the ice
    angleDifference = (targetAngle - currentAngle + 180) % 360 - 180 # find the difference between the target angle and the current angle
    if angleDifference > 0:
      self.direction += angleDifference # turn right towards the target location (xNext, yNext)
    else:
      self.direction -= angleDifference # turn left towards the target location (xNext, yNext)

  def dropGoalMicroDot(self):
    goalMicroDotsToIce.append(MicroDot(self.xPosition, self.yPosition)) # adds ice to goal microdot stack
    goalMicroDotsToHome.append(MicroDot(self.xPosition, self.yPosition)) # adds ant to goal microdot to home list
    self.pathHistory.append(MicroDot(self.xPosition, self.yPosition))  # records microdot to path history

  def goingHome(self):
    if self.hasIceMode == 0: # if ant does not have ice
      print(f"Ant {self.id} is going home without Ice\n")
      if not self.returnPath:  # if return path is empty
        self.returnPath = self.pathHistory[::-1]  # create return path from path history
        self.returnPath.append(MicroDot(Lander.xPosition, Lander.yPosition)) # add Lander to return path
        self.turnToPoint(self.returnPath[1].xPosition, self.returnPath[1].yPosition)  # turn ant around to previous dot
      for dot in range(len(self.returnPath)):  # for each dot in the return microdot stack
        if self.xPosition == self.returnPath[dot].xPosition and self.yPosition == self.returnPath[dot].yPosition: # if you're at a return dot
          if dot + 1 < len(self.returnPath): # if you're not at the last return dot
            self.moveToPoint(self.returnPath[dot + 1].xPosition, self.returnPath[dot + 1].yPosition, lock, positions)  # moves ant to next dot
          else:
            self.moveToPoint(Lander.xPosition, Lander.yPosition, lock, positions)  # move to the Lander
            print(f"Ant {self.id} has returned home\n")
            self.setModes(0, 0, 0, 0, 0, 0, 0, 1)  # set modes to 0 except isHomeMode = 1
            break
    else:
      print(f"Ant {self.id} is going home with Ice\n")
      for dot in range(len(goalMicroDotsToHome)):  # for each dot in the return microdot stack
        if self.xPosition == goalMicroDotsToHome[dot].xPosition and self.yPosition == goalMicroDotsToHome[dot].yPosition: # if you're at a return dot
          if dot + 1 < len(goalMicroDotsToHome):
            self.moveToPoint(goalMicroDotsToHome[dot + 1].xPosition, goalMicroDotsToHome[dot + 1].yPosition, lock, positions)  # moves ant to next dot
          else:
            self.moveToPoint(Lander.xPosition, Lander.yPosition, lock, positions)  # move to the Lander
            print(f"Ant {self.id} has returned home\n")
            self.setModes(0, 0, 0, 0, 0, 0, 0, 1)  # set modes to 0 except isHomeMode = 1
            break


def antMovement(Ant, lock, positions):
  goalMicroDotsToHome = list(goalMicroDotsToIce)[::-1]
  while Ant.isHomeMode == 0:
    Ant.direction = random.randint(0, 360)
    Ant.move(lock, positions)
    while Ant.distanceTravelled < 20 or (Ant.xPosition != 0 or Ant.yPosition != 0) and Ant.isHomeMode == 0 and Ant.pathHistory != []:
      while Ant.distanceTravelled < 500:
        if Ant.huntingForIceOffPathMode == 1: # if ant is hunting for ice off path
          Ant.move(lock, positions) # moves ant in a random direction
          distanceToIce = math.hypot(Ant.xPosition - Ice.xPosition, Ant.yPosition - Ice.yPosition) # checks if ant is in range of ice
          if distanceToIce <= Ice.radius + 10: # if ant is in range of ice
            if Ice.found == 0: # if ice has not been found yet
              Ice.found = 1 # set ice.found to 1
              Ant.setModes(0,0,1,0,0,0,0,0) # huntingForIceOffPathMode = 0 and discoveredIceMode = 1
              Ant.iceFinder = 1 # set iceFinder to 1
            else: # if ice has been found already
              Ant.obtainingIceMode = 1 # obtainingIceMode = 1 (gets ice, returns to spot where it detected ice, and adds it on goal return path)
          else: # if not in range of ice
            for dot in range(len(goalMicroDotsToIce)): # checks if ant is in range of goal dot
              distanceToGoalDot = math.hypot(Ant.xPosition - goalMicroDotsToIce[dot].xPosition, Ant.yPosition - goalMicroDotsToIce[dot].yPosition)
              if distanceToGoalDot <= 2: # if in range of goal dot
                Ant.moveToPoint(goalMicroDotsToIce[dot].xPosition, goalMicroDotsToIce[dot].yPosition) # moves ant to goal dot
                Ant.onGoalPathToIceMode = 1  # onGoalPathToIceMode = 1 (turns ant to path, puts ant on the path)
              else: # if not in range of goal dot
                Ant.huntingForIceOffPathMode = 1 # huntingForIceOffPathMode = 1 (moves ant in a random direction)

        if Ant.discoveredIceMode == 1:
          Ant.turnToPoint(Ant.pathHistory[-2].xPosition, Ant.pathHistory[-2].yPosition) # turn ant around to previous dot
          goalMicroDotsToIce.append(MicroDot(20, -30)) # adds ice to goal microdot stack
          goalMicroDotsToHome.append(MicroDot(20, -30)) # adds dot to goal microdot to home list
          Ant.dropGoalMicroDot() # adds ice to goal microdot stack
          Ant.setModes(0, 0, 0, 0, 0, 1, 0, 0) # discoveredIceMode = 0 and goingHomeWithoutIceMode = 1
          print(f"Ant {Ant.id} has discovered Ice\n")

        if Ant.onGoalPathToIceMode == 1:
          for dot in range(len(goalMicroDotsToIce)):
            if Ant.xPosition != goalMicroDotsToIce[-1].xPosition and Ant.yPosition != goalMicroDotsToIce[-1].yPosition: # if you are at a goal dot, and not at the last goal dot
              Ant.moveToPoint(goalMicroDotsToIce[dot + 1].xPosition, goalMicroDotsToIce[dot + 1].yPosition) # moves ant to goal dot
              print(f"Ant {Ant.id} is moving onto Goal Path\n")
            else: # if you are at the last goal dot (ice @ 20,-30)
              Ant.setModes(0, 0, 0, 1, 0, 0, 0, 0) # set modes to 0 except obtainingIceMode = 1

        if Ant.obtainingIceMode == 1:
          Ant.turnToPoint(20, -30) # turn towards the ice
          Ant.moveToPoint(20, -30, lock, positions) # move to the ice
          Ant.hasIceMode = 1 # obtain Ice and set hasIceMode to 1
          print(f"Ant {Ant.id} has obtained Ice\n")
          Ant.obtainingIceMode = 0 # set obtainingIceMode to 0
          Ice.quantity -= 1 # subtract 1 from the ice quantity
          if len(goalMicroDotsToHome) > 1:  # Check if there are at least 2 elements
            Ant.turnToPoint(goalMicroDotsToHome[1].xPosition, goalMicroDotsToHome[1].yPosition)  # turn around to your last dot that is not the ice
            Ant.moveToPoint(goalMicroDotsToHome[1].xPosition, goalMicroDotsToHome[1].yPosition, lock, positions)  # move to the previous dot
          Ant.setModes(0, 0, 0, 0, 1, 0, 0, 0) # set modes to 0 except goingHomeWithIceMode = 1

        if Ant.goingHomeWithIceMode == 1:
          if Ant.xPosition == goalMicroDotsToHome[-1].xPosition and Ant.yPosition == goalMicroDotsToHome[-1].yPosition: # if you're at the last return dot
            Ant.setModes(0, 0, 0, 0, 0, 0, 0, 1) # set modes to 0 except isHomeMode = 1
          else:
            for dot in range(len(goalMicroDotsToHome)): # for each dot in the return microdot stack
              if Ant.xPosition == goalMicroDotsToHome[dot].xPosition and Ant.yPosition == goalMicroDotsToHome[dot].yPosition and Ant.xPosition != Lander.xPosition and Ant.yPosition != Lander.yPosition: # if you're at a return dot and not at the last return dot
                Ant.moveToPoint(goalMicroDotsToHome[dot + 1].xPosition, goalMicroDotsToHome[dot + 1].yPosition) # moves ant to goal dot
              else:
                Ant.moveToPoint(Lander.xPosition, Lander.yPosition) # moves ant to home
                Ant.isHomeMode = 1 # set isHomeMode to 1

        if Ant.goingHomeWithoutIceMode == 1:
          Ant.goingHome()
          break
      else:
        Ant.goingHome()

ICE_QUANTITY = 9 # number of ice pieces
Lander = Lander() # create Lander object
Ice = Ice() # create Ice object
for i in range(1, 10, 1): # create Ant objects
    Lander.ants.append(Ant(0, 0, i)) # add Ant objects to Lander object
goalMicroDotsToIce = deque() # create goal microdot stack
goalMicroDotsToHome = []  # create goal microdot to home list

lock = threading.Lock() # create lock to lock positions list while adding a position to it
positions = {ant.id: [(ant.xPosition, ant.yPosition)] for ant in Lander.ants} # create positions list
threads = [] # create threads list of ants to move
for Ant in Lander.ants: # for each ant in the Lander object
    thread = threading.Thread(target=antMovement, args=(Ant, lock, positions)) # create a thread for the ant
    threads.append(thread) # add the thread to the threads list
    thread.start() # start the thread

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
plt.legend(bbox_to_anchor=(1,1), loc='upper left') # Move the legend to the top left corner
plt.show()
print(f"Ice pieces remaining: {Ice.quantity}")
print("All ants are home")




