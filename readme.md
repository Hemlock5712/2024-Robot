![Hemlock's Gray Matter - 2024 Robot Code](./docs/images/Github%20Header.png)

## Key Concepts

### State Machine

Most of our code for this year follows a state machine style, while running in the Command Based framework.
Depending on what buttons are pressed by the drivers, the robot will go into different modes.
Using sensors on the robot, the code determines what the best thing to do in that mode is.

For example, if the driver tells the robot they want to score in the amp, but the robot doesn't have a game piece,
the robot will stay in intake mode, until the game piece is in the scoring position for amp. Then, with no
additional effort from the driver, the robot will move into amp mode.

The same is true for all of the different modes.

### SmartController

Building on top of the state machine, we have a singleton called the "SmartController". This is what actually handles
all of the state setting, as well as calculating shooting parameters for the different targets when relevant. Pretty
much all of the subsystems in the robot rely heavily on the SmartController to function.