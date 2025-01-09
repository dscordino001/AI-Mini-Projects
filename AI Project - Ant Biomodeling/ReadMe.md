There are 100 ant bots tasked with finding an ice deposit on the Moon. The ant bots are dropped off all together by a lander (call this location the ‘ant hole’). Assume all the ants move at 1 meter (1 unit) a minute. They can detect ice from 2 meters away. Assume the ice deposit is located at (+20m, -30m) from the lander and is 1 meter in diameter.
Each ant bot follows these rules:
  1. Start at a random direction from the lander, 0 to 360 degrees in 1-degree increments. Once angle is picked, walk 1 meter. Each ant can go 500m before it needs to go back to the lander to recharge.
  2. For each succeeding movement:
    a. Drop microdots to mark a normal path (followed only by the ant that dropped it).
    b. Go one of (random) for 1 meter: right 5 degrees, straight, or left 5 degrees
    c. If ant hits an obstacle, skip to next step
    d. If the ant hits the ice or gets within 2 meters of the ice edge, it follows normal path back to hole, triggering each microdot to send a special goal trail signal of any ant that gets within 2 meters of it.
      i. the microdot is now a special goal trail signal marker and provides direction to ice or a marker that goes to the ice, and also provides direction to a marker that leads back to the lander
    e. If the ant on a normal search gets within 2 meters of a special marker, it goes into retrieve mode, follows the markers to the ice, collects a piece, and brings it back to the lander following the special marker trail each way.
    f. If the ant has gone 500m:
      i. In normal mode, follow normal markers back to lander
      ii. In retrieve mode, follow special markers back to lander.
