/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.followers.EncoderFollower;
import jaci.pathfinder.modifiers.TankModifier;
import jaci.pathfinder.Waypoint;



// Call to return to the nearest input after inputting a piece
public class AutonomousReturn extends Command {
    public AutonomousReturn(int startID, int finishID) {
        this.start = startID;
        this.target = finishID;
    }

    // Called just before this Command runs the first time
    @Override
    protected void initialize() {

        Waypoint[] waypoints = new Waypoint [3];

        //TODO: start waypoint
        //TODO:  middle waypoint
        //TODO: finish waypoint

        Trajectory path = Robot.pathfinding.newComplexPath(waypoints);

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
        double outputLeft = left.calculate((int) (encoderTicksPerRev*Robot.drive.getLeftEncoderRev()));
        double outputRight = right.calculate((int) (encoderTicksPerRev*Robot.drive.getRightEncoderRev()));
        Robot.drive.tankDrive(outputRight, outputLeft);
    }

    // Make this return true when this Command no longer needs to run execute()
    @Override
    protected boolean isFinished() {
        return false;
    } //TODO: FINISH RECOGNITION

    // Called once after isFinished returns true
    @Override
    protected void end() { //TODO
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    @Override
    protected void interrupted() {
        end();
    }

    int target, start;
    EncoderFollower left, right;
    final double robotFrontalWidth = 0.0; //TODO: SET WIDTH
    final double maxV = 0.0; //TODO: SET MAX VELOCITY
    final int encoderTicksPerRev = 0; //TODO
    final double wheelDiameter = 0.0; //TODO
}
