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
    public void pathfinding() {
        // Set the default command for a subsystem here.
        // setDefaultCommand(new MySpecialCommand());
    }

//    public void pathfinding(double robotFrontalWidth) {
//        this.robotFrontalWidth = robotFrontalWidth;
//    }
//
//    // takes angle as reference from present forward direction, in cw direction, in degrees
//    public WayPoint[] newPath(double distance, double angle, int destinationID = 0) {
//        Waypoint[] waypoints = new Waypoint[] {
//                new Waypoint(0, 0, 0);
//                new Waypoint(sin(Pathfinder.d2r(angle)) * distance, cos(Pathfinder.d2r(angle)) * distance, Pathfinder.d2r(angle));
//        };
//
//        if (destinationID != 0) {
//            // Implement obstacles if needed
//        }
//
//    }

    private double robotFrontalWidth;
}
