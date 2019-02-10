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

//Semi auto drive for once generated path; TODO: Implement contiuous generation
public class SemiAutoDrive extends Command{
    public SemiAutoDrive() {
        // Use requires() here to declare subsystem dependencies
    }

    // Called just before this Command runs the first time
    @Override
    protected void initialize() {

        //TODO: Integrate Pathfinding and Drive objects
        Pathfinding = new Pathfinding();
        Drive = new Drive();

        double distance = 0.0, angle = 0.0; //TODO: Implement vision integration

        Trajectory path = Pathfinding.newPath(distance, angle);

        TankModifier modifier = new TankModifier(path).modify(robotFrontalWidth);

        Trajectory[] trajectories = new Trajectory[] {
                modifier.getLeftTrajectory(),
                modifier.getRightTrajectory()
        };


        left = new EncoderFollower(modifier.getLeftTrajectory());
        right = new EncoderFollower(modifier.getRightTrajectory());

        left.configureEncoder((int) (Drive.getLeftEncoderRev()*encoderTicksPerRev), encoderTicksPerRev, wheelDiameter);
        right.configureEncoder((int) (Drive.getRightEncoderRev()*encoderTicksPerRev), encoderTicksPerRev, wheelDiameter);

        left.configurePIDVA(1.0, 0.0, 0.0, 1/maxV, 0);
        right.configurePIDVA(1.0, 0.0, 0.0, 1/maxV, 0);

    }

    // Called repeatedly when this Command is scheduled to run
    @Override
    protected void execute() {
        //TO PROCESS
        double outputLeft = left.calculate((int) (encoderTicksPerRev*Drive.getLeftEncoderRev()));
        double outputRight = right.calculate((int) (encoderTicksPerRev*Drive.getRightEncoderRev()));
        Drive.tankDrive(outputRight, outputLeft);

    }

    // Make this return true when this Command no longer needs to run execute()
    @Override
    protected boolean isFinished() {
        //TODO
        return false;
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

    Pathfinding Pathfinding; //TODO
    Drive Drive; //TODO
    EncoderFollower left, right;
    final double robotFrontalWidth = 0.0; //TODO: SET WIDTH
    final double maxV = 0.0; //TODO: SET MAX VELOCITY
    final int encoderTicksPerRev = 0; //TODO
    final double wheelDiameter = 0.0; //TODO
}
