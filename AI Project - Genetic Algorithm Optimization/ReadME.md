a. A company is designing a new snake-bot to crawl into collapsed buildings or tunnels and look for people.
b. Variables and initial ranges:
i. Length: L: 10 cm to 120 cm
ii. Diameter: D: 2 cm to 5 cm
iii. Stiffness: S: 1 = rigid, 10=super loose
iv. Skin Roughness: R: 0= slippery, 20= sandpaper rough
c. The company will test each potential design but the test scoring can be approximated using these functions:
i. T_1 = (D^2 / L^3) x (ln(S) + 10.2) + L^2 x R/S + 30
ii. T_2 = R / S^2 + D*L + S / R^2 + 40
iii. Snake-bot Goodness, F = T_1 * T_2
d. Describe with equations how you would use a genetic algorithm to optimize the snake-bot design.
e. Show explicitly how the first round of iterations would run in pseudo code.
f. What are the randomizations at what granularity?
g. Describe how you would know you are done?
h. Show how you would narrow the scope of variables to achieve optimization and also avoid local maximums.
