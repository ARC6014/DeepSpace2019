/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import java.lang.*;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.followers.EncoderFollower;
import jaci.pathfinder.modifiers.TankModifier;
import jaci.pathfinder.Waypoint;
import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.PathfinderFRC;



public class AutonomousDrive extends Command {
    //Works for a single object placement only
    // startPositionID: 0 1L, 1 1C, 2 1R, 3 2L, 4 2R, 5 3C, assumes robot won't fall off, thus, use 0, 1, 2 preferrably
    // Start 10 for left loading 11 for right loading
    // TODO: You also need to empirically add in an adjustment for the slant of the Level 1 platform
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

        if (!Robot.usePathWeaver) {
            Waypoint[] waypoints = new Waypoint[3];

            Waypoint startWaypoint;
            if (start == 0) {
                startWaypoint = new Waypoint(2.945, 1.67, 0);
            } else if (start == 1) {
                startWaypoint = new Waypoint(4.115, 1.67, 0);
            } else if (start == 2) {
                startWaypoint = new Waypoint(5.285, 1.67, 0);
            } else if (start == 3) {
                startWaypoint = new Waypoint(2.945, 0.61, 0);
            } else if (start == 4) {
                startWaypoint = new Waypoint(5.285, 0.61, 0);
            } else if (start == 5) {
                startWaypoint = new Waypoint(4.115, 0.61, 0);
            } else if (start == 10) {
                startWaypoint = new Waypoint(0.559, 0.45, 180);
            } else if (start == 11) {
                startWaypoint = new Waypoint(7.67, 0.45, 180);
            } else {
                startWaypoint = new Waypoint(0, 0, 0); //ERROR
            }
            waypoints[0] = startWaypoint;

            //TODO: SEE IF YOU NEED A PREMIDPOINT FOR THE SIDES

            Waypoint middleCheckpoint;
            if (target == 200 || target == 210) {
                middleCheckpoint = new Waypoint(4.115, 4.115, 0);
            } else if ((0 <= target && target < 100) || (200 <= target && target < 210)) {
                middleCheckpoint = new Waypoint(2.06, 4.115, 0);
            } else if ((100 <= target && target < 200) || (210 <= target && target < 220)) {
                middleCheckpoint = new Waypoint(6.17, 4.115, 0);
            } else {
                middleCheckpoint = new Waypoint(0, 0, 0); //ERROR
            }
            waypoints[1] = middleCheckpoint;

            Waypoint finalWaypoint;
            switch (target) {
                case 0:
                    finalWaypoint = new Waypoint(0.35, 5.41, 330);
                    break;
                case 1:
                    finalWaypoint = new Waypoint(0.50, 5.82, 270);
                    break;
                case 2:
                    finalWaypoint = new Waypoint(0.35, 6.23, 210);
                    break;
                case 100:
                    finalWaypoint = new Waypoint(7.88, 5.41, 30);
                    break;
                case 101:
                    finalWaypoint = new Waypoint(7.73, 5.82, 90);
                    break;
                case 102:
                    finalWaypoint = new Waypoint(7.88, 6.23, 150);
                    break;
                case 200:
                    finalWaypoint = new Waypoint(3.84, 5.59, 0);
                    break;
                case 201:
                    finalWaypoint = new Waypoint(3.38, 6.62, 90);
                    break;
                case 202:
                    finalWaypoint = new Waypoint(3.38, 7.175, 90);
                    break;
                case 203:
                    finalWaypoint = new Waypoint(3.38, 7.728, 90);
                    break;
                case 210:
                    finalWaypoint = new Waypoint(4.39, 5.59, 0);
                    break;
                case 211:
                    finalWaypoint = new Waypoint(4.85, 6.62, 270);
                    break;
                case 212:
                    finalWaypoint = new Waypoint(4.85, 7.175, 270);
                    break;
                case 213:
                    finalWaypoint = new Waypoint(4.85, 7.728, 270);
                    break;
                default:
                    finalWaypoint = new Waypoint(0, 0, 0); //ERROR
                    break;
            }
            waypoints[2] = finalWaypoint;
            //End point add

            Trajectory path = Robot.pathfinding.newComplexPath(waypoints);

            TankModifier modifier = new TankModifier(path).modify(robotFrontalWidth);


            left = new EncoderFollower(modifier.getLeftTrajectory());
            right = new EncoderFollower(modifier.getRightTrajectory());

        } else {
            String pathName;
            if (false) {}
//            else if () {}
//            else if () {}
//            else if () {}  //TODO: IMPLEMENT NAMES OF PRECREATED PATHS AND CONDITIONS FROM PATHWEAVER
//            else if () {}
//            else if () {}
            else {pathName = "";}  // Path not found, TODO redirect to regular pathfinder
            left = new EncoderFollower(PathfinderFRC.getTrajectory(pathName + ".left"));
            right = new EncoderFollower(PathfinderFRC.getTrajectory(pathName + ".right"));
        }

        left.configureEncoder((int) (Robot.drive.getLeftEncoderRev()*encoderTicksPerRev), encoderTicksPerRev,
                wheelDiameter);
        right.configureEncoder((int) (Robot.drive.getRightEncoderRev()*encoderTicksPerRev), encoderTicksPerRev,
                wheelDiameter);

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
        return (left.isFinished() && right.isFinished());
    }

    // Called once after isFinished returns true
    @Override
    protected void end() {
        Robot.drive.tankDrive(0,0);
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    @Override
    protected void interrupted() {
        end();
    }

    int start, target;
    EncoderFollower left, right;
    final double robotFrontalWidth = Robot.robotFrontalWidth;
    final double maxV = Robot.maxV;
    final int encoderTicksPerRev = Robot.encoderTicksPerRev;
    final double wheelDiameter = Robot.wheelDiameter;
}

