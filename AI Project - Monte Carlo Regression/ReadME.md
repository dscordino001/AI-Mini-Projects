Monte-Carlo Solutions and Regression: A new spacecraft is being designed, but we need to optimize a few factors for tradeoffs, and project the best solutions for further research.

1. length varies from 150m to 500m
2. radius (of center body) varies from 12m to 50m
3. thruster count goes from 10 to 22 thrusters
4. warp power goes from 1 to 8 warp factor
5. nacelle separation = min(1.1*radius, 118*SQRT(warp power))
6. mass=length*radius*0.1+5*warp power+2*thruster count
7. Weapon Capability, w= 110*mass+11*radius+57*warppower-6*thrustercount^2
8. Maneuverability, m = 11/length^2+21/radius^3+7*nacelleseperation+8*thrustercount+6/mass^3

Task A: Do a Monte-Carlo analysis as a table for 200+ possible spacecraft using the variables, then plot the solution points with Weapons Capability on the X axis and Maneuverability on the Y axis.
Task B: Do min-max analysis for independent variables and make a plot showing them with the Monte-Carlo points.
Task C: Do a linear regression analysis on the Monte-Carlo data and plot the ensuing line on the Monte-Carlo points plot as above.
Task D: Do a polynomial regression analysis on the Monte-Carlo data and plot the ensuing curve on the Monte-Carlo points plot as above.
Task E: Do an exponential regression analysis on the Monte-Carlo data and plot the ensuing curve on the Monte-Carlo points plot as above.
Task F: Looking at the 3 regression analysis plots above, assume I have a Weapons Capability score of 250000, what maneuverability ratings should I get from each plot?
