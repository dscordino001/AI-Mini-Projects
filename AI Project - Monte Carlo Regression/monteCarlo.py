import math
import random
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

# Constants
MIN_LENGTH = 150
MAX_LENGTH = 500
MIN_RADIUS = 12
MAX_RADIUS = 50
MIN_THRUSTERS = 10
MAX_THRUSTERS = 22
MIN_WARP_POWER = 1
MAX_WARP_POWER = 8


class RandomShip:
    def __init__(self):
        self.length = random.randint(MIN_LENGTH, MAX_LENGTH)
        self.radius = random.randint(MIN_RADIUS, MAX_RADIUS)
        self.thruster_count = random.randint(MIN_THRUSTERS, MAX_THRUSTERS)
        self.warp_power = random.randint(MIN_WARP_POWER, MAX_WARP_POWER)
        self.nacelle_separation = min((1.1 * self.radius), (118 * math.sqrt(self.warp_power)))
        self.mass = (self.length * self.radius * .1) + (self.warp_power * 5) + (self.thruster_count * 2)
        self.weapons_capability = (self.mass * 110) + (self.radius * 11) + (self.warp_power * 57) + (
                (self.thruster_count ** 2) * 6)
        self.maneuverability = (11 / (self.length ** 2)) + (21 / (self.radius ** 3)) + (self.nacelle_separation * 7) + (
                self.thruster_count * 8) + (6 / (self.mass ** 3))


class SpecificShip:
    def __init__(self, name, length, radius, thruster_count, warp_power):
        self.name = name
        self.length = length
        self.radius = radius
        self.thruster_count = thruster_count
        self.warp_power = warp_power
        self.nacelle_separation = min((1.1 * radius), (118 * math.sqrt(warp_power)))
        self.mass = (length * radius * .1) + (warp_power * 5) + (thruster_count * 2)
        self.weapons_capability = (self.mass * 110) + (radius * 11) + (warp_power * 57) + (
                (thruster_count ** 2) * 6)
        self.maneuverability = (11 / (length ** 2)) + (21 / (radius ** 3)) + (self.nacelle_separation * 7) + (
                thruster_count * 8) + (6 / (self.mass ** 3))


# Generate 200+ random ships
randomShips = [RandomShip() for i in range(200)]

# Create SpecificShip instances for all possible min and max values
# ships are named. this is so they get lines connected to them in a certain order
# this will create a surrounding shape around the random ships
specificShips = [
    SpecificShip("Ship01", MIN_LENGTH, MIN_RADIUS, MIN_THRUSTERS, MIN_WARP_POWER),
    SpecificShip("Ship02", MIN_LENGTH, MIN_RADIUS, MIN_THRUSTERS, MAX_WARP_POWER),
    SpecificShip("Ship03", MAX_LENGTH, MIN_RADIUS, MIN_THRUSTERS, MIN_WARP_POWER),
    SpecificShip("Ship04", MAX_LENGTH, MIN_RADIUS, MIN_THRUSTERS, MAX_WARP_POWER),
    SpecificShip("Ship05", MAX_LENGTH, MAX_RADIUS, MIN_THRUSTERS, MIN_WARP_POWER),
    SpecificShip("Ship06", MAX_LENGTH, MAX_RADIUS, MIN_THRUSTERS, MAX_WARP_POWER),
    SpecificShip("Ship07", MAX_LENGTH, MAX_RADIUS, MAX_THRUSTERS, MAX_WARP_POWER),
    SpecificShip("Ship08", MAX_LENGTH, MAX_RADIUS, MAX_THRUSTERS, MIN_WARP_POWER),
    SpecificShip("Ship09", MIN_LENGTH, MAX_RADIUS, MAX_THRUSTERS, MAX_WARP_POWER),
    SpecificShip("Ship10", MIN_LENGTH, MAX_RADIUS, MAX_THRUSTERS, MIN_WARP_POWER),
    SpecificShip("Ship11", MIN_LENGTH, MAX_RADIUS, MIN_THRUSTERS, MAX_WARP_POWER),
    SpecificShip("Ship12", MIN_LENGTH, MAX_RADIUS, MIN_THRUSTERS, MIN_WARP_POWER),
    SpecificShip("Ship13", MAX_LENGTH, MIN_RADIUS, MAX_THRUSTERS, MAX_WARP_POWER),
    SpecificShip("Ship14", MAX_LENGTH, MIN_RADIUS, MAX_THRUSTERS, MIN_WARP_POWER),
    SpecificShip("Ship15", MIN_LENGTH, MIN_RADIUS, MAX_THRUSTERS, MAX_WARP_POWER),
    SpecificShip("Ship16", MIN_LENGTH, MIN_RADIUS, MAX_THRUSTERS, MIN_WARP_POWER),
]

# Create a DataFrame for the Monte-Carlo analysis
data = {
    'Length': [ship.length for ship in randomShips],
    'Radius': [ship.radius for ship in randomShips],
    'Thruster Count': [ship.thruster_count for ship in randomShips],
    'Warp Power': [ship.warp_power for ship in randomShips],
    'Nacelle Separation': [ship.nacelle_separation for ship in randomShips],
    'Mass': [ship.mass for ship in randomShips],
    'Weapons Capability': [ship.weapons_capability for ship in randomShips],
    'Maneuverability': [ship.maneuverability for ship in randomShips]
}

# Add specific ships to the DataFrame
specific_data = {
    'Name': [ship.name for ship in specificShips],
    'Length': [ship.length for ship in specificShips],
    'Radius': [ship.radius for ship in specificShips],
    'Thruster Count': [ship.thruster_count for ship in specificShips],
    'Warp Power': [ship.warp_power for ship in specificShips],
    'Nacelle Separation': [ship.nacelle_separation for ship in specificShips],
    'Mass': [ship.mass for ship in specificShips],
    'Weapons Capability': [ship.weapons_capability for ship in specificShips],
    'Maneuverability': [ship.maneuverability for ship in specificShips]
}

# Create and Print the DataFrame as a table
dataFrame = pd.DataFrame(data)
specificDataFrame = pd.DataFrame(specific_data)
print(dataFrame)
print(specificDataFrame)

specific_points = specificDataFrame[['Weapons Capability', 'Maneuverability']].values # Extract the specific ship points
specific_points = specific_points[np.argsort(specificDataFrame['Name'])] # Sort the points by name
specific_points = np.vstack([specific_points, specific_points[0]]) # Add the first point to the end to close the shape

# Plot the random ship points and plot the specific ship points
plt.scatter(dataFrame['Weapons Capability'], dataFrame['Maneuverability'], label='Random Ships', color='blue', alpha=0.5)
plt.scatter(specificDataFrame['Weapons Capability'], specificDataFrame['Maneuverability'], label='Min and Max Ships',
            color='red', marker='x')

# Plot lines connecting the specific ships
plt.plot(specific_points[:, 0], specific_points[:, 1], color='orange', linestyle='--')

x = dataFrame['Weapons Capability'] # Extract the Weapons Capability values and set it as the x values
y = dataFrame['Maneuverability'] # Extract the Maneuverability values and set it as the y values
m, b = np.polyfit(x, y, 1) # Calculate the linear regression line

coefficients = np.polyfit(x, y, 2) #generate the coefficients for the polynomial regression line
polynomial = np.poly1d(coefficients) # Calculate the polynomial regression line

# Generate x values for the polynomial regression line
x_poly = np.linspace(x.min(), x.max(), 500)
y_poly = polynomial(x_poly)

# Calculate the exponential regression line
log_y = np.log(y)
coefficients = np.polyfit(x, log_y, 1)
exponential = np.exp(coefficients[1]) * np.exp(coefficients[0] * x)

# Generate x values for the exponential line
x_exponential = np.linspace(x.min(), x.max(), 500)
y_exponential = np.exp(coefficients[1]) * np.exp(coefficients[0] * x_exponential)

# Plot the linear regression line
plt.plot(x, m * x + b, color='pink', linestyle='-', linewidth=2, label=f'Linear Regression: \ny = {m}x + {b}')

# Plot the polynomial regression line
plt.plot(x_poly, y_poly, color='green', linestyle='-', linewidth=2, label=f'Polynomial Regression: {polynomial}')

# Plot the exponential regression line
plt.plot(x_exponential, y_exponential, color='purple', linestyle='-', linewidth=2, label=f'Exponential Regression: \ny = e^({coefficients[0]}x) * {np.exp(coefficients[1])}')

# print the equation for the linear, polynomial, exponential regression lines
print(f'Linear Regression:\n y = {m}x + {b}')
print(f'Polynomial Regression: {polynomial}')
print(f'Exponential Regression: \n y = e^({coefficients[0]}x) * {np.exp(coefficients[1])}')
print(f'Linear Regression value for Weapons Capability of 250000: {m * 250000 + b}')
# polynomial regression line value for if weapons_capability is 250000
print(f'Polynomial Regression value for Weapons Capability of 250000: {polynomial(250000)}')
# exponential regression line value for if weapons_capability is 250000
print(f'Exponential Regression value for Weapons Capability of 250000: {np.exp(coefficients[1]) * np.exp(coefficients[0] * 250000)}')

# Add labels and title to the plot
plt.xlabel('Weapons Capability')
plt.ylabel('Maneuverability')
plt.title('Weapons Capability vs Maneuverability of Ships')
plt.legend()
plt.show()

