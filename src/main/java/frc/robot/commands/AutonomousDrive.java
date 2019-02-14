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


public class AutonomousDrive extends Command {
    //Works for a single object placement only
    // startPositionID: 0 1L, 1 1C, 2 1R, 3 2L, 4 2R, 5 3C, assumes robot won't fall off, thus, use 0, 1, 2 preferrably
    //TODO: You also need to empirically add in an adjustment for the slant of the Level 1 platform
    // targetID: 0XX L Rocket, 1XX RRocket, 2XX CargoShip
    // {0/1}0{0->2} moves front to back
    // 20X L, 21X R
    // 2{0/1}{0->3} moves front to back
    // objectID: 0 for HatchPanel, 1 for Sphere
    public AutonomousDrive(int startPositionID, int targetID) {
        this.start = startPositionID;
        this.target = targetID;
    }

    // Called just before this Command runs the first time
    @Override
    protected void initialize() {

        Waypoint[] waypoints = new Waypoint [3];

//TODO: CHECK UNITS
        Waypoint startWaypoint;
        if (start == 0) {
            startWaypoint = new Waypoint(294.5, 167, 0);
        } else if (start == 1) {
            startWaypoint = new Waypoint(411.5, 167, 0);
        } else if (start == 2) {
            startWaypoint = new Waypoint(528.5, 167, 0);
        } else if (start == 3) {
            startWaypoint = new Waypoint(294.5, 61, 0);
        } else if (start == 4) {
            startWaypoint = new Waypoint(528.5, 61, 0);
        } else if (start == 5) {
            startWaypoint = new Waypoint(411.5, 61, 0);
        } else {
            startWaypoint = new Waypoint(0,0,0); //ERROR
        }
        waypoints[0] = startWaypoint;

        // TODO: Improve middleCheckpoint
        Waypoint middleCheckpoint;
        if (target == 200 || target == 210) {
            middleCheckpoint = new Waypoint(411.5,411.5,0);
        } else if ((0 <= target && target < 100) || (200 <= target && target < 210)) {
            middleCheckpoint = new Waypoint(206,411.5,0);
        } else if ((100 <= target && target < 200) || (210 <= target && target < 220)){
            middleCheckpoint = new Waypoint(617,411.5,0);
        } else {
            middleCheckpoint = new Waypoint(0,0,0); //ERROR
        }
        waypoints[1] = middleCheckpoint;

        Waypoint finalWaypoint;
        switch(target) {
            case 0:
                finalWaypoint = new Waypoint(35,541,330);
                break;
            case 1:
                finalWaypoint = new Waypoint(50,582,270);
                break;
            case 2:
                finalWaypoint = new Waypoint(35,623,210);
                break;
            case 100:
                finalWaypoint = new Waypoint(788,541,30);
                break;
            case 101:
                finalWaypoint = new Waypoint(773,582,90);
                break;
            case 102:
                finalWaypoint = new Waypoint(788,623,150);
                break;
            case 200:
                finalWaypoint = new Waypoint(384,559,0);
                break;
            case 201:
                finalWaypoint = new Waypoint(338,662,90);
                break;
            case 202:
                finalWaypoint = new Waypoint(338,717.5,90);
                break;
            case 203:
                finalWaypoint = new Waypoint(338,772.8,90);
                break;
            case 210:
                finalWaypoint = new Waypoint(439,559,0);
                break;
            case 211:
                finalWaypoint = new Waypoint(485,662,270);
                break;
            case 212:
                finalWaypoint = new Waypoint(485,717.5,270);
                break;
            case 213:
                finalWaypoint = new Waypoint(485,772.8,270);
                break;
            default:
                finalWaypoint = new Waypoint(0,0,0); //ERROR
                break;
        }
        waypoints[2] = finalWaypoint;
        //End point add

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
    protected boolean isFinished() { //TODO: ADD COMPLETION DETECTION; WHEN VISION DETECTED
        return false;
    }

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

    int start, target;
    EncoderFollower left, right;
    final double robotFrontalWidth = 0.0; //TODO: SET WIDTH
    final double maxV = 0.0; //TODO: SET MAX VELOCITY
    final int encoderTicksPerRev = 0; //TODO
    final double wheelDiameter = 0.0; //TODO
}

