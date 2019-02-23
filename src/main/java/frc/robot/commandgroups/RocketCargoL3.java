package frc.robot.commandgroups;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.Robot;
import frc.robot.commands.AutoLiftElevator;
import frc.robot.commands.AutoRotateIntakeWrist;

public class RocketCargoL3 extends CommandGroup {
    private final double maxHeight = Robot.elevator.maxHeight;
    private final double angle = 29.08;    //Check

    public RocketCargoL3() {
        addParallel(new AutoLiftElevator(maxHeight));
        addParallel(new AutoRotateIntakeWrist(angle));
    }
}
