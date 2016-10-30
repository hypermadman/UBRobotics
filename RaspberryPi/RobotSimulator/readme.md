#Eurobot Robot Simulator by UBRobotics

##Features
Create and place a two-wheeled robot 

Measure arena distances and angles in centimeters

Save/Load configurations

###1: PLACING A ROBOT

Fill in the 'Primary Robot Controller' window with the required data. Heading is the angle of the robot relative to North on the monitor. Clockwise is negative.
Track is the distance between the two wheels (similar to gauge in trains).

Once the information is filled, press 'Build Primary Robot' to place the robot on the arena. 'Save configuration' and 'Load configuration' buttons in the 'Global Settings' window will save the robot data.

You can press 'Flip sides' to flip the robot between the two starting areas, symmetrical to each other down the middle of the length of the arena.

###2: SET UP THE ARENA

First of all, the arena image should be called 'board.jpg' the program will rotate the image to fit in landscape.

Make sure when the MAIN WINDOW loads that the arena is in landscape, with the audience-side facing up on the monitor. If not, adjust with a photo editor.

Should be done first. Under the 'Global Settings' window, adjust datum position until the LARGE DOT aligns with the bottom right corner of the arena (within the walls).

Then adjust the length and width such that the red rectangle is in line with the inside of the arena boundaries. The rectangle will represent the 2 x 3 meter arena and is a reference to convert from pixels to centimeters.

###3: MEASUREMENTS ON THE ARENA

Left click on the arena to give the co-ordinates relative to the datum point.

Left click, then right click another place to draw a line between the two points. You will get the horizontal length, vertical length, diagonal length and the angle from North.

Left click, right click, then middle click on three different points of the arena to draw two lines between them to find the angle between them.

All measurements are output to the shell console.

###4: SAVING AND LOADING

Robot and arena data are saved by the 'Save configuration' button.

When using 'Load configuration', you must 'Update Arena' THEN 'Build primary robot' to refresh the arena and robot to the loaded configurations.
