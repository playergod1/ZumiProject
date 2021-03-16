# Advanced Zumi Class
Contains Functions like:
- Simple commands like forward, reverse ...
- Random Walker: Driving Random on street with top cam
- Drive to Point (x, y): Navigates Zumi through path to nearest point to (x, y)

Location of the cars is done with color mapping via top cam and a mask to prefilter irrelevant colors out.

## Random Walker
The random walker function drives randomly through on the street. The zumi checks if some obstacles are in front, so a forward command would result in a crash. If nothing is in front, the car drives a small step forward. If something is infront, the program searchs a random angle, where no obstacle is in front. The obstacle detection only works for street/non street and is implemented with a image mask.

## Drive to Point
This function drives the zumi to a griven point. The zumi drives stepwise to every point in a predefined path until the nearest path to the wanted point is given. 

