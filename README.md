# Autonomous Solar Panel Cleaning Robot Intenship

P-Solar is an autonomous solar panel cleaning robot. Its mission is to optimize the panels’ performance by doing regular cleanings without resort to an operator to manipulate it. Solar panels are most performing when they are 100% clean allowing it to utilize the light efficiently. The robot’s importance becomes evident in deserted or windy/rainy areas where it is hard to hire someone to clean and where we need panel cleaning more often.
Given that the company just finished developing the first version of the robot, there are many potential features to be developed, tested, and implemented on the next versions:
- Developing the trajectory-generating algorithm according to a pattern chosen by the company and simulating it using the panel’s corners coordinates as input.
- Developing the trajectory-generating algorithm in the presence of obstacles with known coordinates.
- Trajectory tracking to have traceability of the swept (cleaned) areas and the non-swept areas.
- Refining the panel’s GPS coordinates for seamless integration into the trajectory algorithm.

## Algorithm1: Simple Trajectory planner
This algorithm aims to generate the robot’s trajectory through multiple checkpoints that draw the pattern in figure 2. This pattern was chosen after trials of the actual robot that proved that a first pattern could not be implemented due to the risks of errors leading to the robot failing to find the correct checkpoint or falling off the solar panel.

## Algorithm2: Trajectory Planner with Obstacles
This algorithm is dedicated to solar panels that present obstacles interrupting the pattern's continuity. I developed the first version of the algorithm that considers panels with aligned obstacles having the same width. During the configuration and setup of the environment, an operator is responsible for saving the panel and the obstacles coordinates. This data is saved in a Json file and delivered to the algorithm which processes them. The algorithm sorts the obstacles based on ascendant ‘x’, then, it “scans” the panel and simultaneously divides the panel into small sections limited by the obstacles. For each section, the planned trajectory is same as in algorithm 1.

## Algorithm3: Trajectory Tracking 
This algorithm aims to track the swept area by the robot in order to keep traceability of the cleaned areas, be aware of the non-cleaned areas, and save them for a second sweep. It is also useful to keep track of the robot’s performance.

## Algorithm4: Coordinates Correction
This algorithms aims to correct the coordinates delivered by the robot’s station: the GPS base sends (x,y,z) coordinates, given the incline of the panel and the fact that the robot uses the (x,y) out of the (x,y,z) the resulting shape is not a rectangle but rather an irregular quadrilaterial shape. We need to correct that shape to avoid undesired behaviors of the robot. In order to do so we fix one side of the resulting shape and build a rectangle out of it using orthogonals.
