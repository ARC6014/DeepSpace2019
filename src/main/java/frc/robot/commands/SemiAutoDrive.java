package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.Pathfinding;
import frc.robot.subsystems.Drive;
import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.Waypoint;
import jaci.pathfinder.modifiers.TankModifier;
import jaci.pathfinder.followers.EncoderFollower;

//Semi auto drive for once generated path;
public class SemiAutoDrive extends Command{
    public SemiAutoDrive() {
        // Use requires() here to declare subsystem dependencies
        requires(Robot.pathfinding);
        requires(Robot.drive);
    }

    // Called just before this Command runs the first time
    @Override
    protected void initialize() {

        double distance = 0.0, angle = 0.0; //TODO: Implement vision integration

        Trajectory path = Robot.pathfinding.newSimplePath(distance, angle);

        TankModifier modifier = new TankModifier(path).modify(robotFrontalWidth);

        Trajectory[] trajectories = new Trajectory[] {
                modifier.getLeftTrajectory(),
                modifier.getRightTrajectory()
        };

        left = new EncoderFollower(modifier.getLeftTrajectory());
        right = new EncoderFollower(modifier.getRightTrajectory());

        left.configureEncoder((int) (Robot.drive.getLeftEncoderRev()*encoderTicksPerRev), encoderTicksPerRev, wheelDiameter);
        right.configureEncoder((int) (Robot.drive.getRightEncoderRev()*encoderTicksPerRev), encoderTicksPerRev, wheelDiameter);

        left.configurePIDVA(1.0, 0.0, 0.0, 1/maxV, 0); //TODO: Implement PIDVA
        right.configurePIDVA(1.0, 0.0, 0.0, 1/maxV, 0); //TODO: Implement PIDVA

    }

    // Called repeatedly when this Command is scheduled to run
    @Override
    protected void execute() {
        //TO PROCESS
        double outputLeft = left.calculate((int) (encoderTicksPerRev*Robot.drive.getLeftEncoderRev()));
        double outputRight = right.calculate((int) (encoderTicksPerRev*Robot.drive.getRightEncoderRev()));
        Robot.drive.tankDrive(outputRight, outputLeft);

    }

    // Make this return true when this Command no longer needs to run execute()
    @Override
    protected boolean isFinished() {
        double distance = 0.0; //TODO:ADD DISTANCE MEASUREMENT
        return distance == 0.0;
    }

    // Called once after isFinished returns true
    @Override
    protected void end() {}

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    @Override
    protected void interrupted() {
        end();
    }

    EncoderFollower left, right;
    final double robotFrontalWidth = Robot.robotFrontalWidth;
    final double maxV = Robot.maxV;
    final int encoderTicksPerRev = Robot.encoderTicksPerRev;
    final double wheelDiameter = Robot.wheelDiameter;
}
