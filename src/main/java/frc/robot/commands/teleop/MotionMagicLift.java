package frc.robot.commands.teleop;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.MotionMagicElevator;

public class MotionMagicLift extends Command {
    private boolean switcher = false;
    private boolean resetStatus = false;

    public MotionMagicLift() {
        requires(Robot.motionMagicElevator);
    }

    @Override
    protected void initialize() {
        if(Robot.motionMagicElevator.motionMagicStateMachine == MotionMagicElevator.MotionMagicStateMachine.MANUAL) {

            //TODO: Manual Control Method

        } else if (Robot.motionMagicElevator.motionMagicStateMachine == MotionMagicElevator.MotionMagicStateMachine.MOTION_MAGIC) {

            //TODO: Adjust Levels

        } else { }


    }

    @Override
    protected void execute() {

    }

    @Override
    protected boolean isFinished() {
        return false;
    }

    @Override
    protected void end() {

    }

    @Override
    protected void interrupted() { end(); }
}
