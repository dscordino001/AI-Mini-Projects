# Constants
import math
import random
import pandas as pd
import matplotlib.pyplot as plt

# Iterations for the simulation and chopping percentile
ITERATIONS = 1000
CHOPPING_PERCENTILE = 80
DECIMAL_PLACES = 2
solutionCounter = 0

# Constants for attribute range values
MIN_LENGTH = 10.00
MAX_LENGTH = 120.00
MIN_DIAMETER = 2.00
MAX_DIAMETER = 5.00
MIN_STIFFNESS = 10.00
MAX_STIFFNESS = 1.00
MIN_ROUGHNESS = 0.01
MAX_ROUGHNESS = 20.00


class SnakeBot:
    def __init__(self):
        self.length = round(random.uniform(ADJUSTED_MIN_LENGTH, ADJUSTED_MAX_LENGTH), DECIMAL_PLACES)
        self.diameter = round(random.uniform(ADJUSTED_MIN_DIAMETER, ADJUSTED_MAX_DIAMETER), DECIMAL_PLACES)
        self.stiffness = round(random.uniform(ADJUSTED_MAX_STIFFNESS, ADJUSTED_MIN_STIFFNESS), DECIMAL_PLACES)
        self.roughness = round(random.uniform(ADJUSTED_MIN_ROUGHNESS, ADJUSTED_MAX_ROUGHNESS), DECIMAL_PLACES)
        self.t1 = ((self.diameter ** 2) / (self.length ** 3)) * (math.log(self.stiffness) + 10.2) + (
                ((self.length ** 2) * self.roughness) / self.stiffness) + 30
        self.t2 = (self.roughness / (self.stiffness ** 2)) + (self.diameter * self.length) + (
                self.stiffness / (self.roughness ** 2)) + 40
        self.goodness = self.t1 * self.t2


# Constants for Adjusted attribute range values
ADJUSTED_MIN_LENGTH = 10.00
ADJUSTED_MAX_LENGTH = 120.00
ADJUSTED_MIN_DIAMETER = 2.00
ADJUSTED_MAX_DIAMETER = 5.00
ADJUSTED_MIN_STIFFNESS = 10.00
ADJUSTED_MAX_STIFFNESS = 1.00
ADJUSTED_MIN_ROUGHNESS = 0.01
ADJUSTED_MAX_ROUGHNESS = 20.00


class AdjustedSnakeBot:
    def __init__(self):
        self.length = round(random.uniform(ADJUSTED_MIN_LENGTH, ADJUSTED_MAX_LENGTH), DECIMAL_PLACES)
        self.diameter = round(random.uniform(ADJUSTED_MIN_DIAMETER, ADJUSTED_MAX_DIAMETER), DECIMAL_PLACES)
        self.stiffness = round(random.uniform(ADJUSTED_MAX_STIFFNESS, ADJUSTED_MIN_STIFFNESS), DECIMAL_PLACES)
        self.roughness = round(random.uniform(ADJUSTED_MIN_ROUGHNESS, ADJUSTED_MAX_ROUGHNESS), DECIMAL_PLACES)
        self.t1 = ((self.diameter ** 2) / (self.length ** 3)) * (math.log(self.stiffness) + 10.2) + (
                ((self.length ** 2) * self.roughness) / self.stiffness) + 30
        self.t2 = (self.roughness / (self.stiffness ** 2)) + (self.diameter * self.length) + (
                self.stiffness / (self.roughness ** 2)) + 40
        self.goodness = self.t1 * self.t2


best_goodness_values = []
while solutionCounter < 1:
    for z in range(ITERATIONS):
        # generate population of AdjustedSnakeBots
        adjustedSnakeBots = [AdjustedSnakeBot() for i in range(900000)]
        # generate population of SnakeBots
        snakeBots = [SnakeBot() for i in range(100000)]


        # create a list for all snakeBots generated
        snakeBotPopulation = adjustedSnakeBots + snakeBots

        # create goodnessData frame for SnakeBots
        goodnessData = {
            "goodness": [snakeBot.goodness for snakeBot in snakeBotPopulation]
        }

        # add the best snakeBot to the bestInPopulation list
        bestInPopulation = [max(snakeBotPopulation, key=lambda x: x.goodness)]
        best_goodness_values.append((z, bestInPopulation[0].goodness))

        retainedPopulation = []
        # for score in goodnessData["goodness"] that is above the 80% percentile, add them to the retained list
        for snakeBot in adjustedSnakeBots:
            if snakeBot.goodness > sorted(goodnessData["goodness"])[
                int(len(goodnessData["goodness"]) * (.01 * CHOPPING_PERCENTILE))]:
                retainedPopulation.append(snakeBot)

        # create a DataFrame for the retained population
        df = pd.DataFrame({
            "length": [snakeBot.length for snakeBot in retainedPopulation],
            "diameter": [snakeBot.diameter for snakeBot in retainedPopulation],
            "stiffness": [snakeBot.stiffness for snakeBot in retainedPopulation],
            "roughness": [snakeBot.roughness for snakeBot in retainedPopulation],
            "goodness": [snakeBot.goodness for snakeBot in retainedPopulation]
        })

        # Check if retainedPopulation is not empty before calculating min and max values
        if retainedPopulation:
            ADJUSTED_MIN_LENGTH = round(min([snakeBot.length for snakeBot in retainedPopulation]), DECIMAL_PLACES)
            ADJUSTED_MAX_LENGTH = round(max([snakeBot.length for snakeBot in retainedPopulation]), DECIMAL_PLACES)
            ADJUSTED_MIN_DIAMETER = round(min([snakeBot.diameter for snakeBot in retainedPopulation]), DECIMAL_PLACES)
            ADJUSTED_MAX_DIAMETER = round(max([snakeBot.diameter for snakeBot in retainedPopulation]), DECIMAL_PLACES)
            ADJUSTED_MIN_STIFFNESS = round(min([snakeBot.stiffness for snakeBot in retainedPopulation]), DECIMAL_PLACES)
            ADJUSTED_MAX_STIFFNESS = round(max([snakeBot.stiffness for snakeBot in retainedPopulation]), DECIMAL_PLACES)
            ADJUSTED_MIN_ROUGHNESS = round(min([snakeBot.roughness for snakeBot in retainedPopulation]), DECIMAL_PLACES)
            ADJUSTED_MAX_ROUGHNESS = round(max([snakeBot.roughness for snakeBot in retainedPopulation]), DECIMAL_PLACES)
            ADJUSTED_MIN_GOODNESS = round(min([snakeBot.goodness for snakeBot in retainedPopulation]), DECIMAL_PLACES)
            ADJUSTED_MAX_GOODNESS = round(max([snakeBot.goodness for snakeBot in retainedPopulation]), DECIMAL_PLACES)

            # Print the goodnessData frame of the retained population
            print(f"\nIteration {z}")
            print(df)
            print("")
            print("Adjusted Ranges")
            print(f"Length Range: {ADJUSTED_MIN_LENGTH} - {ADJUSTED_MAX_LENGTH}")
            print(f"Diameter Range: {ADJUSTED_MIN_DIAMETER} - {ADJUSTED_MAX_DIAMETER}")
            print(f"Stiffness Range: {ADJUSTED_MAX_STIFFNESS} - {ADJUSTED_MIN_STIFFNESS}")
            print(f"Roughness Range: {ADJUSTED_MIN_ROUGHNESS} - {ADJUSTED_MAX_ROUGHNESS}")
            print(f"Goodness Range: {ADJUSTED_MIN_GOODNESS} - {ADJUSTED_MAX_GOODNESS}")
        else:
            solutionCounter += 1
            print("No retained population to adjust ranges.")
            print("")
            print(f"Solution found after {z} rounds")
            print(f"Length: {ADJUSTED_MIN_LENGTH}")
            print(f"Diameter: {ADJUSTED_MIN_DIAMETER}")
            print(f"Stiffness: {ADJUSTED_MAX_STIFFNESS}")
            print(f"Roughness: {ADJUSTED_MIN_ROUGHNESS}")
            print(f"Goodness: {ADJUSTED_MIN_GOODNESS}")
            break

# Plot the best goodness values from each iteration
iterations, goodness_values = zip(*best_goodness_values)
plt.figure(figsize=(10, 6))
plt.plot(iterations, goodness_values, marker='o', linestyle='-', color='b')

plt.xlabel('Iteration')
plt.ylabel('Best Goodness Value')
plt.title('Best Goodness Value from Each Iteration')
plt.grid(True)
plt.show()
