package frc.robot.subsystems.baseclasses;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import edu.wpi.first.wpilibj.command.PIDSubsystem;

public abstract class PIDSubsystemsBase extends PIDSubsystem {

    public PIDSubsystemsBase(double Kp, double Ki, double Kd, double Kf, double KPeriod) {
        super(Kp, Ki, Kd, Kf, KPeriod);
    }

    public VictorSPX masterMotor;
    public double outPID;

    //
    //Common Methods
    //

    public void setMotorSpeedManual(double speed) {
        masterMotor.set(ControlMode.PercentOutput, speed);
    }

    public void setMotorSpeedPID() {
        masterMotor.set(ControlMode.PercentOutput, outPID);
    }

    public double getMotorSpeed() {
        return masterMotor.getMotorOutputPercent();
    }

    //
    //Rotatory Wrist
    //

    public void setWristF(double PID_WRIST_STALL_OUTPUT, double CURRENT_WRIST_ANGLE) {
        this.getPIDController().setF(PID_WRIST_STALL_OUTPUT * Math.cos(Math.toRadians(CURRENT_WRIST_ANGLE)));
    }

}
