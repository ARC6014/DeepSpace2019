/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.Waypoint;
import jaci.pathfinder.modifiers.TankModifier;



/**
 * An example subsystem.  You can replace me with your own Subsystem.
 */
public class Pathfinding extends Subsystem {
    // Put methods for controlling this subsystem
    // here. Call these from Commands.

    @Override
    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        // setDefaultCommand(new MySpecialCommand());
    }

    //Set the width between the two sides' wheels
    public void setRobotWidth(double robotFrontalWidth) {
        this.robotFrontalWidth = robotFrontalWidth;
    }

    // takes angle as reference from present forward direction, in cw direction, in degrees
    public Trajectory[] newPath(double distance, double angle) {
        return newPath(distance, angle, 0);
    }


    public Trajectory[] newPath(double distance, double angle, int destinationID) {
        Waypoint[] waypoints = new Waypoint[] {
                new Waypoint(0, 0, 0),
                new Waypoint(Math.sin(Pathfinder.d2r(angle)) * distance, Math.cos(Pathfinder.d2r(angle)) * distance, Pathfinder.d2r(angle))
        };

        if (destinationID != 0) {
            // Implement obstacles if needed
        }

        // Third variable movement time interval, max vel., acc., and jerk are 4-6, respectively; input
        Trajectory.Config configuration = new Trajectory.Config(Trajectory.FitMethod.HERMITE_QUINTIC, Trajectory.Config.SAMPLES_HIGH, 0.1, 1, 1, 1);

        Trajectory path =  Pathfinder.generate(waypoints, configuration);

        TankModifier modifier = new TankModifier(path).modify(robotFrontalWidth);

        Trajectory[] trajectories = new Trajectory[] {
                modifier.getLeftTrajectory(),
                modifier.getRightTrajectory()
        };

        return trajectories;

    }

    private double robotFrontalWidth = 0; // Defaults to 0
}
