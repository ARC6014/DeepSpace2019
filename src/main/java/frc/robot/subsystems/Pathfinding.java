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

//TODO: Add handlers for autonomous

/**
 * An example subsystem.  You can replace me with your own Subsystem.
 */
public class Pathfinding extends Subsystem {
    // Put methods for controlling this subsystem
    // here. Call these from Commands.

    public Pathfinding() {}

    @Override
    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        // setDefaultCommand(new MySpecialCommand());
    }

    // takes angle as reference from present forward direction, in cw direction, in degrees
    public Trajectory newSimplePath(double distance, double angle) {
        return newSimplePath(distance, angle, 0);
    }


    public Trajectory newSimplePath(double distance, double angle, int destinationID) {
        Waypoint[] waypoints = new Waypoint[] {
                new Waypoint(0, 0, 0),
                new Waypoint(Math.sin(Pathfinder.d2r(angle)) * distance, Math.cos(Pathfinder.d2r(angle)) * distance, Pathfinder.d2r(angle))
        };

        if (destinationID != 0) {
            // Implement obstacles if needed
        }

        // Third variable movement time interval, max vel., acc., and jerk are 4-6, respectively; TODO
        Trajectory.Config configuration = new Trajectory.Config(Trajectory.FitMethod.HERMITE_QUINTIC, Trajectory.Config.SAMPLES_HIGH, 0.1, 1, 1, 1);

        Trajectory path =  Pathfinder.generate(waypoints, configuration);

        return path;

    }

    public Trajectory newComplexPath(Waypoint[] waypoints) {
        Trajectory.Config configuration = new Trajectory.Config(Trajectory.FitMethod.HERMITE_QUINTIC, Trajectory.Config.SAMPLES_HIGH, 0.1, 1, 1, 1);

        Trajectory path =  Pathfinder.generate(waypoints, configuration);

        return path;
    }

}
