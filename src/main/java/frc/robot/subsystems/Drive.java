/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.command.PIDSubsystem;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotMap;
import frc.robot.commands.teleop.TeleopDrive;

/**
 * An example subsystem.  You can replace me with your own Subsystem.
 */
public class Drive extends PIDSubsystem {


  AHRS navx = new AHRS(RobotMap.navx);

  VictorSPX driveFrontRightMotor = new VictorSPX(RobotMap.driveFrontRightMotor);
  VictorSPX driveRearRightMotor = new VictorSPX(RobotMap.driveRearRightMotor);
  VictorSPX driveFrontLeftMotor = new VictorSPX(RobotMap.driveFrontLeftMotor);
  VictorSPX driveRearLeftMotor = new VictorSPX(RobotMap.driveRearLeftMotor);

  Encoder driveRightEncoder = new Encoder(RobotMap.driveRightEncoderA, RobotMap.driveRightEncoderB, false, Encoder.EncodingType.k4X);
  Encoder driveLeftEncoder = new Encoder(RobotMap.driveLeftEncoderA, RobotMap.driveLeftEncoderB, false, Encoder.EncodingType.k4X);

  private final int leftEncoderCPR = 2048 * 4;  //Check the encoder values
  private final int rightEncoderCPR = 2048 * 4; //Check the encoder values
  private final double wheelDiameter = 4 * 2.54;

  private double maxSpeed = 1.0;

  public enum DriveStateMachine{
    DISABLED,
    MANUAL,
    PATHFINDING
  }
  public DriveStateMachine driveStateMachine = DriveStateMachine.MANUAL;

  private double outPID = 0;

  public Drive() {
    super(0,0,0);
    setAbsoluteTolerance(5);
    getPIDController().setInputRange(-180.0,180.0);
    getPIDController().setOutputRange(-1,1);
    getPIDController().setContinuous(true);

    driveFrontRightMotor.setInverted(true);
    driveRearRightMotor.setInverted(true);

    driveRearRightMotor.follow(driveFrontRightMotor);
    driveRearLeftMotor.follow(driveFrontLeftMotor);
  }

  @Override
  public void initDefaultCommand() {
    setDefaultCommand(new TeleopDrive());
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("DriveRightMotor", driveFrontRightMotor.getMotorOutputPercent());
    SmartDashboard.putNumber("DriveLeftMotor", driveFrontLeftMotor.getMotorOutputPercent());

    SmartDashboard.putNumber("Heading",getHeading());
    SmartDashboard.putNumber("RightEncoderDistance",getRightDistance());
    SmartDashboard.putNumber("LeftEncoderDistance",getLeftDistance());
  }


  public double getHeading() {
    return navx.getYaw();
  }

  public void resetNavx() {
    navx.reset();
  }

  public void resetEncoders() {
    driveLeftEncoder.reset();
    driveRightEncoder.reset();
  }


  public double getRightEncoderRev() {
    return driveRightEncoder.get() / (double)rightEncoderCPR;
  }

  public double getLeftEncoderRev() {
    return driveLeftEncoder.get() / (double)leftEncoderCPR;
  }

  public double getRightDistance() {
    return getRightEncoderRev() * wheelDiameter * Math.PI;
  }

  public double getLeftDistance() {
    return getLeftEncoderRev() * wheelDiameter * Math.PI;
  }



  public void setAngle(double angle) {
    this.setSetpoint(angle);
  }

  @Override
  protected double returnPIDInput() {
    return getHeading();
  }

  @Override
  protected void usePIDOutput(double output) {
    this.outPID = output;
  }

  public void PIDDrive(double speed) {
    arcadeDrive(speed, outPID);
  }



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

  public void tankDrive(double rightSpeed, double leftSpeed) {
    driveFrontRightMotor.set(ControlMode.PercentOutput, rightSpeed*maxSpeed);
    driveFrontLeftMotor.set(ControlMode.PercentOutput, leftSpeed*maxSpeed);
    //SmartDashboard.putNumber("DriveRight", rightSpeed * maxSpeed);
    //SmartDashboard.putNumber("DriveLeft", leftSpeed * maxSpeed);
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
    driveFrontRightMotor.set(ControlMode.PercentOutput, limit(right) * maxSpeed);
    driveFrontLeftMotor.set(ControlMode.PercentOutput, limit(left) * maxSpeed);
  }
}
