package frc.robot.library.baseclasses;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.command.PIDSubsystem;
import frc.robot.RobotMap;


public abstract class DrivePIDBase extends PIDSubsystem {

    public DrivePIDBase(double Kp, double Ki, double Kd) {
        super(Kp, Ki, Kd);
    }

    AHRS navx = new AHRS(RobotMap.navx);
    public VictorSPX driveRightMaster;
    public VictorSPX driveLeftMaster;
    private double maxSpeed = 1.0;
    public double outPID;

    //
    //NAVX Methods
    //

    public double getHeading() {
        return navx.getYaw();
    }

    public void resetNavx() {
        navx.reset();
    }

    //
    //Helper Methods
    //

    public void setMaxSpeed (double speed) {
        maxSpeed = speed;
    }

    private double limit(double speed) {
        if (speed >= 1.0) {
            return 1.0;
        }
        if (speed <= -1.0) {
            return -1.0;
        }
        return speed;
    }

    //
    //Movement Methods
    //

    public void tankDrive(double rightSpeed, double leftSpeed) {
        driveRightMaster.set(ControlMode.PercentOutput, rightSpeed*maxSpeed);
        driveLeftMaster.set(ControlMode.PercentOutput, leftSpeed*maxSpeed);
    }

    public void arcadeDrive(double speed, double rotation) {
        double left, right;
        double maxInput = Math.copySign(Math.max(Math.abs(speed), Math.abs(rotation)), speed);
        if (speed >= 0.0) {

            if (rotation >= 0.0) {
                left = maxInput;
                right = speed - rotation;
            } else {
                left = speed + rotation;
                right = maxInput;
            }
        } else {
            if (rotation >= 0.0) {
                left = speed + rotation;
                right = maxInput;
            } else {
                left = maxInput;
                right = speed - rotation;
            }
        }

        driveRightMaster.set(ControlMode.PercentOutput, limit(right) * maxSpeed);
        driveLeftMaster.set(ControlMode.PercentOutput, limit(left) * maxSpeed);
    }

    public void PIDDrive(double speed) {
        arcadeDrive(speed, outPID);
    }

    public double getLeftMotorSet() {
        return driveLeftMaster.getMotorOutputPercent();
    }

    public double getRightMotorSet() {
        return driveRightMaster.getMotorOutputPercent();
    }

}
