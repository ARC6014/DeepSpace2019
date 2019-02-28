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

//Works for a single object placement only
// startID: 0XX L Rocket, 1XX RRocket, 2XX CargoShip
// {0/1}0{0->2} moves front to back
// 20X L, 21X R
// 2{0/1}{0->3} moves front to back

// Call to return to the nearest input after inputting a piece
public class AutonomousReturn extends Command {
    public AutonomousReturn(int startID) {
        this.start = startID;
    }

    // Called just before this Command runs the first time
    @Override
    protected void initialize() {

        Waypoint[] waypoints = new Waypoint [4];

        Waypoint startWaypoint;
        switch(start) {
            case 0:
                startWaypoint = new Waypoint(0.35,5.41,330);
                break;
            case 1:
                startWaypoint = new Waypoint(0.50,5.82,270);
                break;
            case 2:
                startWaypoint = new Waypoint(0.35,6.23,210);
                break;
            case 100:
                startWaypoint = new Waypoint(7.88,5.41,30);
                break;
            case 101:
                startWaypoint = new Waypoint(7.73,5.82,90);
                break;
            case 102:
                startWaypoint = new Waypoint(7.88,6.23,150);
                break;
            case 200:
                startWaypoint = new Waypoint(3.84,5.59,0);
                break;
            case 201:
                startWaypoint = new Waypoint(3.38,6.62,90);
                break;
            case 202:
                startWaypoint = new Waypoint(3.38,7.175,90);
                break;
            case 203:
                startWaypoint = new Waypoint(3.38,7.728,90);
                break;
            case 210:
                startWaypoint = new Waypoint(4.39,5.59,0);
                break;
            case 211:
                startWaypoint = new Waypoint(4.85,6.62,270);
                break;
            case 212:
                startWaypoint = new Waypoint(4.85,7.175,270);
                break;
            case 213:
                startWaypoint = new Waypoint(4.85,7.728,270);
                break;
            default:
                startWaypoint = new Waypoint(0,0,0); //ERROR
                break;
        }
        waypoints[0] = startWaypoint;

        boolean onLeft;
        if ((0 <= start && start < 100) || (200 <= start && start < 210)) {
            onLeft = true;
        } else if ((100 <= start && start < 200) || (210 <= start && start < 220)) {
            onLeft = false;
        } else {
            onLeft = false; //ERROR!!!!!!}
        }

        //TODO: FIRST PREMIDPOINT

        Waypoint preMidPoint;
        if (onLeft && ((start % 10) > 0)) {
            preMidPoint = new Waypoint(1.905,5.842,180);
        } else if ((!onLeft) && ((start % 10) > 0)) {
            preMidPoint = new Waypoint(5.7,5.842,180);
        } else if ((start == 200) || (start == 210)) {
            preMidPoint =  new Waypoint(3.8,3.81, 0);
        } else if (onLeft && (start == 0)) {
            preMidPoint = new Waypoint(0.85,4.91,330);
        } else if ((!onLeft) && (start == 100)) {
            preMidPoint = new Waypoint(7.38,4.91,30);
        } else {
            preMidPoint = new Waypoint(0,0,0); //ERROR!!
        }
        waypoints[1] = preMidPoint;

        Waypoint midWaypoint;
        if (onLeft) {
            midWaypoint = new Waypoint(1.6,3.8,180);
        } else {
            midWaypoint = new Waypoint(6.03,3.8,180);
        }
        waypoints[2] = midWaypoint;


        Waypoint finWaypoint;
        if (onLeft) {
            finWaypoint = new Waypoint(0.559,0.45,180);
        } else {
            finWaypoint = new Waypoint(7.67,0.45,180);
        }
        waypoints[3] = finWaypoint;

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
    protected boolean isFinished() { //TODO: FINISH RECOGNITION
        return false;
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

    int start;
    EncoderFollower left, right;
    final double robotFrontalWidth = Robot.robotFrontalWidth;
    final double maxV = Robot.maxV;
    final int encoderTicksPerRev = Robot.encoderTicksPerRev;
    final double wheelDiameter = Robot.wheelDiameter;
}
