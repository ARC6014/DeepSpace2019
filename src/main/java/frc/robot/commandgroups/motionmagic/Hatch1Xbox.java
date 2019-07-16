package frc.robot.commandgroups.motionmagic;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.commands.RotateIntakeWrist;

public class Hatch1Xbox extends CommandGroup {

    public Hatch1Xbox() {
        addSequential(new RotateIntakeWrist(70),0.5);
        addSequential(new RocketHatchL1());
    }
}
