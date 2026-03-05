# Updates afer 03/03/2026

All of the TODO's in the code should be blue squiggly lines, you can also search for them.

## Shooter Subsystem

No longer turret, it is a fixed shooter with an adjustable hood. This means for aiming we will need to
add code to the drive subsystem potentially to allow for aiming the robot itself. This also means no contstant aiming. Possibly add aiming to a trigger or something that the driver can hold down to lock on, and let go to drive normally?

## Drive Subsystem

After looking into it, we can probably use the "joystickDriveAtAngle" command to allow for aiming the robot. We can probably swap between that and the default drive command with a buttonn press in Robot Container.
