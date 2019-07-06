package frc.robot.subsystems;

//
//Example Elevator Subsystem
//Using Motion Magic with PID
//
//Useful Sources:
//
//https://phoenix-documentation.readthedocs.io/en/latest/ch14_MCSensor.html
//https://phoenix-documentation.readthedocs.io/en/latest/ch14a_BringUpRemoteSensors.html
//http://www.ctr-electronics.com/downloads/api/java/html/index.html
//


import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.teleop.MotionMagicLift;

class ElevatorConst {

    //TalonSRX Constants
    public static final int CAN_TIMEOUT = 10;
    public static final int NO_WAIT = 0;
    public static final double PEAK_OUTPUT = 0.8;
    public static final double CALIBRATION_SPEED = 0.20;

    //Gearbox & Elevator Constants
    public static final double BASE_TO_INTAKE_HEIGHT = 41.5;
    public static final double OUTPUT_RATIO = 2.5;
    public static final double SPROCKET_TEETH = 22;
    public static final double CHAIN_PITCH = 0.250 * 2.54;
    public static final double MAX_ELEVATOR_HEIGHT = 194.05;
    public static final double UPPER_LIMIT = MAX_ELEVATOR_HEIGHT - BASE_TO_INTAKE_HEIGHT;
    public static final double LOWER_LIMIT = 0.0;

    //Motion Magic Constants
    public static final int VELOCITY_LIMIT = 0;   //TODO: Determine units per 100ms
    public static final int ACCEL_LIMIT = 0;  //TODO: Determine
    //https://phoenix-documentation.readthedocs.io/en/latest/ch14_MCSensor.html

    //PID Constants
    public static final double PROPORTIONAL = 0;
    public static final double INTEGRAL = 0;
    public static final double DERIVATIVE = 0;
    public static final double FEED_FORWARD = 0;

    //Tolerance
    public static final double TOLERANCE = 3;

}


public class MotionMagicElevator extends Subsystem {

    TalonSRX elevatorMotor = new TalonSRX(0);
    DigitalInput elevatorSwitch = new DigitalInput(0);

    boolean calibrated;
    double tolerance;

    public enum MotionMagicStateMachine{
        DISABLED,
        MANUAL,
        MOTION_MAGIC
    }

    public MotionMagicStateMachine motionMagicStateMachine = MotionMagicStateMachine.DISABLED;

    public MotionMagicElevator() {
        elevatorMotor.configFactoryDefault();

        elevatorMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, ElevatorConst.CAN_TIMEOUT);
        setPIDConstants(ElevatorConst.PROPORTIONAL, ElevatorConst.INTEGRAL, ElevatorConst.DERIVATIVE, ElevatorConst.FEED_FORWARD);
        setAbsoluteTolerance(ElevatorConst.TOLERANCE);
        //elevatorMotor.setSensorPhase(true);
        //elevatorMotor.setInverted(true);

        elevatorMotor.configForwardSoftLimitEnable(false, ElevatorConst.NO_WAIT); //False until after calibration
        elevatorMotor.configReverseSoftLimitEnable(false, ElevatorConst.NO_WAIT);

        elevatorMotor.setNeutralMode(NeutralMode.Brake);

        elevatorMotor.configPeakOutputForward(ElevatorConst.PEAK_OUTPUT, ElevatorConst.CAN_TIMEOUT);
        elevatorMotor.configPeakOutputReverse(-ElevatorConst.PEAK_OUTPUT, ElevatorConst.CAN_TIMEOUT);
        elevatorMotor.configNominalOutputForward(0, ElevatorConst.CAN_TIMEOUT);
        elevatorMotor.configPeakOutputReverse(0, ElevatorConst.CAN_TIMEOUT);

        elevatorMotor.configMotionCruiseVelocity(ElevatorConst.VELOCITY_LIMIT, ElevatorConst.CAN_TIMEOUT);
        elevatorMotor.configMotionAcceleration(ElevatorConst.ACCEL_LIMIT, ElevatorConst.CAN_TIMEOUT);
    }

    @Override
    protected void initDefaultCommand() {
        setDefaultCommand(new MotionMagicLift());
    }

    @Override
    public void periodic() {
        if(!calibrated && !getElevatorSwitchStatus()) {
            elevatorMotor.setSelectedSensorPosition(0,0, ElevatorConst.CAN_TIMEOUT);
            calibrated = true;

            elevatorMotor.configForwardSoftLimitThreshold((int)cmToTicks(ElevatorConst.UPPER_LIMIT), ElevatorConst.NO_WAIT);
            elevatorMotor.configReverseSoftLimitThreshold((int)cmToTicks(ElevatorConst.LOWER_LIMIT), ElevatorConst.NO_WAIT);
            elevatorMotor.configForwardSoftLimitEnable(true, ElevatorConst.NO_WAIT); //False until after calibration
            elevatorMotor.configReverseSoftLimitEnable(true, ElevatorConst.NO_WAIT);
        }

        SmartDashboard.putNumber("Elevator Setpoint", getSetPoint() + ElevatorConst.BASE_TO_INTAKE_HEIGHT);
        SmartDashboard.putBoolean("ElevatorBottomSwitch", getElevatorSwitchStatus());
        SmartDashboard.putNumber("ElevatorHeight", getElevatorHeight());
        SmartDashboard.putNumber("ElevatorMotor",elevatorMotor.getMotorOutputPercent());
        SmartDashboard.putString("ElevatorState", motionMagicStateMachine.toString());
        SmartDashboard.putNumber("ElevatorVelocity",getVelocity());
    }

    //
    //Configuration Commands
    //

    public void setPIDConstants(double proportional, double integral, double derivative, double feedForward) {
        //Using main PID Loop index(0)
        elevatorMotor.config_kP(0, proportional, ElevatorConst.CAN_TIMEOUT);
        elevatorMotor.config_kI(0, integral, ElevatorConst.CAN_TIMEOUT);
        elevatorMotor.config_kD(0, derivative, ElevatorConst.CAN_TIMEOUT);
        elevatorMotor.config_kF(0,feedForward, ElevatorConst.CAN_TIMEOUT);

    }

    public void setAbsoluteTolerance(double tolerance) {
        this.tolerance = tolerance;
    }

    //
    //Movement Commands
    //

    public void setSetPoint(double setPoint) {
        if (setPoint < ElevatorConst.BASE_TO_INTAKE_HEIGHT) {
            elevatorMotor.set(ControlMode.MotionMagic, 0);
        }
        else if (setPoint > ElevatorConst.MAX_ELEVATOR_HEIGHT) {
            elevatorMotor.set(ControlMode.MotionMagic, cmToTicks(ElevatorConst.MAX_ELEVATOR_HEIGHT - ElevatorConst.BASE_TO_INTAKE_HEIGHT));
        }
        else {
            elevatorMotor.set(ControlMode.MotionMagic, cmToTicks(setPoint - ElevatorConst.BASE_TO_INTAKE_HEIGHT));
        }
    }

    public void calibrationMovement() {
        elevatorMotor.set(ControlMode.PercentOutput, ElevatorConst.CALIBRATION_SPEED);
    }

    public void setUncalibrated() {
        this.calibrated = false;
    }

    //
    //Helper Commands
    //

    public double getSetPoint() {
        return ticksToCm(elevatorMotor.getClosedLoopTarget(0));
    }

    public double getVelocity() {
        return elevatorMotor.getSelectedSensorVelocity(0);
    }

    public boolean isOnTarget() {
        double error = ticksToCm(elevatorMotor.getClosedLoopError(0));
        return (Math.abs(error) < tolerance);
    }

    public boolean isLiftOnTarget() {
        double error = (getSetPoint() + ElevatorConst.BASE_TO_INTAKE_HEIGHT) - getElevatorHeight();
        return (Math.abs(error) < tolerance);
    }

    public boolean getElevatorSwitchStatus() {
        return !elevatorSwitch.get();
    }

    public boolean getCalibrated() {
        return calibrated;
    }


    //
    //Calculations
    //

    public double getElevatorHeight() {
        return ElevatorConst.BASE_TO_INTAKE_HEIGHT + (elevatorMotor.getSelectedSensorPosition() / ElevatorConst.OUTPUT_RATIO) * ElevatorConst.SPROCKET_TEETH * ElevatorConst.CHAIN_PITCH * 2;
    }

    public double ticksToCm(double ticks) {
        return (ticks / ElevatorConst.OUTPUT_RATIO) * ElevatorConst.SPROCKET_TEETH * ElevatorConst.CHAIN_PITCH * 2;
    }

    public double cmToTicks(double cm) {
        return cm / (ElevatorConst.SPROCKET_TEETH * ElevatorConst.CHAIN_PITCH * 2) * ElevatorConst.OUTPUT_RATIO;
    }

}
