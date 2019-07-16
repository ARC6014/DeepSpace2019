package frc.robot.commandgroups.motionmagic;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.commands.motionmagic.SetElevatorHeight;

public class RocketCargoL3 extends CommandGroup {
    private final double maxHeight = 192;
    //private final double angle = 29.08;    //Check

    public RocketCargoL3() {
        addParallel(new SetElevatorHeight(maxHeight));
        //addParallel(new RotateIntakeWrist(angle));
    }
}
