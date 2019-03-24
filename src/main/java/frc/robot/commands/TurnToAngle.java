package frc.robot.commands;

import edu.wpi.first.wpilibj.command.TimedCommand;
import frc.robot.Robot;

public class TurnToAngle extends TimedCommand {
    private double speed;
    private double angle;

    public TurnToAngle(double timeout, double rotationSpeed, double angle) {
        super(timeout);
        requires(Robot.drive);
        this.speed = rotationSpeed;
        this.angle = angle;
    }

    @Override
    public void initialize() {
        Robot.drive.setAngle(angle);
        Robot.drive.driveStateMachine = Robot.drive.driveStateMachine.PID;
    }

    @Override
    public void execute() {
        Robot.drive.PIDDrive(0);
    }

    @Override
    public void interrupted() {
        end();
    }

    @Override
    public void end() {
        Robot.drive.driveStateMachine = Robot.drive.driveStateMachine.MANUAL;
        Robot.drive.arcadeDrive(0,0);
    }

}
