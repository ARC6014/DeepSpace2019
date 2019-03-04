package frc.robot.commandgroups;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.Robot;
import frc.robot.commands.LiftElevator;
import frc.robot.commands.RotateIntakeWrist;

public class RocketCargoL3 extends CommandGroup {
    private final double maxHeight = 192;
    //private final double angle = 29.08;    //Check

    public RocketCargoL3() {
        addParallel(new LiftElevator(maxHeight));
        //addParallel(new RotateIntakeWrist(angle));
    }
}
