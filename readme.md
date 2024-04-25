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

In order for anything to happen on the robot, the drivers press buttons, which
tell the code that they are requesting for an action to happen. If the robot isn't
in a state that can handle that, it will either do something to prepare for that mode,
or will remain in the intake mode.

### "Smart" Commands

The backbone of every subsystem is a default command that's running, which handles
all of the normal operating procedures of the robot. For example, `SmartMagazine`
is constantly watching the values of the line breaks, so if a game piece touches one of them, it will automatically run and move the game piece to the loaded state.

This allows us to not have to juggle several commands for every subsystem, and instead handle the logic in one centralized place, while keeping that logic out
of the subsystem itself to allow it to be overridden.

During autonomous, we often run similar commands, with slight variations since
it's in a much more predictable state during autonomous.