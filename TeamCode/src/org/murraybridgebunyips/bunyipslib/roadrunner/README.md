# RoadRunner with BunyipsLib

RoadRunner v0.5 (as of 2023) has been fully implemented into BunyipsLib, meaning that you can use
RoadRunner in your code without having to set up anything more than dependencies.<br><br>
Accessing tuning OpModes is as simple as extending whichever OpMode you want to use, and then
calling `super.runOpMode()` in your `runOpMode()` method.<br><br>
The tuning OpModes accept instances of RobotConfig, allowing you to pass in your own configurations
that you would use with BunyipsOpMode. Additional interfaces such as RoadRunnerDrive have been
implemented throughout BunyipsLib to ensure that you can use RoadRunner in your own code without
having to worry about the implementation details of different drive types.<br><br>
An example of tuning OpModes can be found in the /example/ directory under debug/.
