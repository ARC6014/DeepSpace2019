package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.library.baseclasses.DrivePIDBase;
import frc.robot.library.baseclasses.EncoderBase;

public class ExamplePIDDrive extends DrivePIDBase {
    //Constants
    public double WHEEL_DIAMETER = 4 * 2.54;

    //Create motor objects
    VictorSPX rightmotor = new VictorSPX(0);
    VictorSPX leftmotor = new VictorSPX(0);

    EncoderBase newEncoder = new EncoderBase(1,2);

    public ExamplePIDDrive() {
        super(0,0,0);
        //Configure PID Properties
        //setAbsoluteTolerance(5);
        //getPIDController().setInputRange(-180.0,180.0);
        //getPIDController().setOutputRange(-1,1);
        //getPIDController().setContinuous(true);

        //Make the motors follow "BaseClass Masters"
        rightmotor.follow(driveRightMaster);
        leftmotor.follow(driveLeftMaster);

        resetNavx();
    }

    @Override
    protected void initDefaultCommand() {
        //setDefaultCommand(new TeleopDrive());
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("MotorDistance", newEncoder.getEncoderDistance(WHEEL_DIAMETER));
    }

    //
    //PID Controls
    //

    @Override
    protected double returnPIDInput() {
        return getHeading();
    }

    @Override
    protected void usePIDOutput(double output) {
        this.outPID = output;
    }

    public void setAngle(double angle) {
        this.setSetpoint(angle);
    }

}
