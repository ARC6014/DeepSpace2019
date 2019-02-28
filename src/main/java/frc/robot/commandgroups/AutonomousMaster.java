package frc.robot.commandgroups;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.Robot;
import frc.robot.commands.AutonomousDrive;
import frc.robot.commands.AutonomousReturn;

public class AutonomousMaster extends CommandGroup{
    //Works for a single object placement only
    // start: 0 1L, 1 1C, 2 1R, 3 2L, 4 2R, 5 3C, assumes robot won't fall off, thus, use 0, 1, 2 preferrably
    // targetID: 0XX L Rocket, 1XX RRocket, 2XX CargoShip
    // {0/1}0{0->2} moves front to back
    // 20X L, 21X R
    // 2{0/1}{0->3} moves front to back
    public AutonomousMaster(int start, int target1, int target2) {
        addSequential(new AutonomousDrive(start, target1));
        //Vision
        //Drop
        addSequential(new AutonomousReturn(target1));
        //Pickup

        boolean onLeft;
        if ((0 <= start && start < 100) || (200 <= start && start < 210)) {
            onLeft = true;
        } else if ((100 <= start && start < 200) || (210 <= start && start < 220)) {
            onLeft = false;
        } else {
            onLeft = false; //ERROR!!!!!!}
        }

        if(onLeft) {
            addSequential(new AutonomousDrive(10, target2));
        } else {
            addSequential(new AutonomousDrive(11, target2));
        }

        //Vision
        //Drop
    }

    int start, target1, target2;
}
