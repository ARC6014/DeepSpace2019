package frc.robot.library;
import org.opencv.core.Mat;

import java.math.*;

class Constants {

    //Natural
    public static final double gravity = -9.81;

    //Technical
    public static final double maxLaunchSpeed = 0;
    public static final double minLaunchAngle = 0;
}

public class LaunchPhysics {

    //
    //Helper Commands
    //

    public double angleToRadian(double angle) {
        return Math.toRadians(angle);
    }

    //
    //Commands:
    //

    public double returnLaunchHeight(double targetHeight, double targetDistance, double launchAngle, double launchSpeed) {
        double angle = angleToRadian(launchAngle);
        return targetHeight - (Constants.gravity * targetDistance) / (2 * launchSpeed * Math.cos(angle)) - (Math.tan(angle) * targetDistance);
    }

    public double returnLaunchSpeed(double targetHeight, double targetDistance, double launchAngle, double launchHeight) {
        double angle = angleToRadian(launchAngle);
        return (Constants.gravity * targetDistance) / (2 * Math.cos(angle) * (targetHeight - launchHeight - Math.tan(angle) * targetDistance));
    }
}


