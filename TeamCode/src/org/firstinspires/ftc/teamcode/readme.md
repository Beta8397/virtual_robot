## TeamCode Module

This package contains all maintained code for FTC robots constructed after 2021 using Java and
Kotlin (>29/01/23).

## Naming convention

All OpModes should include the robot name in their Driver Station name and group
to avoid confusion.

'_independent' packages do not rely on common classes and objects and can be run solely on
the FIRST SDKs resources. These classes are now only used for learning purposes and
proper OpModes should derive
from [BunyipsOpMode ecosystem](https://github.com/Murray-Bridge-Bunyips/BunyipsFTC/tree/stable/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/example).

## File structure

OpModes are organised based on their package name and should only be run on their respective robot.

All non-active archived code is stored in [archived/](../../../../../archived/) and is not built.  
This code is not maintained and may be using deprecated or broken functionality.
