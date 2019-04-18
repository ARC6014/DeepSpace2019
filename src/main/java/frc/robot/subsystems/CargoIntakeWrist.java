/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.command.PIDSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotMap;
import frc.robot.commands.teleop.PIDIntakeWrist;

/**
 * An example subsystem.  You can replace me with your own Subsystem.
 */
public class CargoIntakeWrist extends PIDSubsystem {
  double outPID = 0;

  VictorSPX cargoIntakeWristLeftMotor = new VictorSPX(RobotMap.cargoIntakeWristLeftMotor);
  VictorSPX cargoIntakeWristRightMotor = new VictorSPX(RobotMap.cargoIntakeWristRightMotor);

  Encoder cargoIntakeWristEncoder = new Encoder(RobotMap.cargoIntakeWristEncoderA, RobotMap.cargoIntakeWristEncoderB, false, Encoder.EncodingType.k4X);
  DigitalInput cargoIntakeWristBottomSwitch = new DigitalInput(RobotMap.cargoIntakeWristBottomSwitch);

  boolean switchPressed = false;
  private static final double maxWristAngle = 114;
  //private static final double feedForward = 0.1;
  private final int encoderCPR = 2048;
  private final double outputRatio = 5.0;

  private static final double wristStallOutput = -0.205;
  private static final double armMass = 5;
  private static final double dToCM = 0.325;
  private static final double stallTorque = 0.71;
  private static final double g = 9.81;
  private static final double armWeight = g * armMass;
  private static final double noOfMotors = 2;
  private static final double gearRatio = 208.33;
  private static final double feedForward =armWeight * dToCM / stallTorque * noOfMotors * gearRatio;
  private static boolean atStart = true;
  public double startFallAngle;

  public enum CargoIntakeWristStateMachine{
    DISABLED,
    MANUAL,
    PID
  }
  public CargoIntakeWristStateMachine cargoIntakeWristStateMachine = CargoIntakeWristStateMachine.MANUAL;

  public CargoIntakeWrist() {
    super(0.14,0,0.65,wristStallOutput * Math.cos(Math.toRadians(maxWristAngle)),0.02);
    setAbsoluteTolerance(1);
    getPIDController().setInputRange(0,maxWristAngle);
    getPIDController().setOutputRange(-1,1);
    setSetpoint(maxWristAngle);
    cargoIntakeWristLeftMotor.setInverted(true);
    cargoIntakeWristLeftMotor.follow(cargoIntakeWristRightMotor);
  }
  @Override
  public void initDefaultCommand() {
    setDefaultCommand(new PIDIntakeWrist());
  }

  @Override
  public void periodic() {
    if(getCargoSwitchStatus()){
      switchPressed=true;
      resetEncoder();
    }
    setF();

    SmartDashboard.putNumber("Intake Setpoint",getSetpoint());
    SmartDashboard.putNumber("StartAngle", startFallAngle);
    SmartDashboard.putNumber("CargoWristMotor", cargoIntakeWristRightMotor.getMotorOutputPercent());
    SmartDashboard.putNumber("WristAngle", getWristAngle());
    SmartDashboard.putBoolean("CargoIntakeWristSwitch", getCargoSwitchStatus());
    SmartDashboard.putNumber("CargoStallSpeed", stallSpeed);
    SmartDashboard.putNumber("CargoStallCalc", Math.cos(Math.toRadians(getWristAngle())));
  }

  public void setF() {
    this.getPIDController().setF(wristStallOutput * Math.cos(Math.toRadians(getWristAngle())));
  }

  public void resetEncoder() {
    cargoIntakeWristEncoder.reset();
  }

  public double getEncoderRev() {
    return cargoIntakeWristEncoder.get() / (double)encoderCPR;
  }

  public double getWristAngle() {
    //return getEncoderRev() / outputRatio * 360;
    if(switchPressed) {
      return getEncoderRev() / outputRatio * 360;
    }
    return maxWristAngle + (getEncoderRev() / outputRatio * 360);
  }

  public boolean getCargoSwitchStatus() {
    return !cargoIntakeWristBottomSwitch.get();
  }



  @Override
  protected double returnPIDInput() {
    return getWristAngle();
  }

  @Override
  public void usePIDOutput(double output) {
    this.outPID = output;
  }
  public void setWristAngle(double angle) {
    this.setSetpoint(angle);
  }

  public void PIDRotate() {
    setWristSpeed(-outPID);
  }


  double stallSpeed = wristStallOutput * Math.cos(Math.toRadians(getWristAngle()));

  public double getMotorSet() {
    return cargoIntakeWristRightMotor.getMotorOutputPercent();
  }
  public void setMotorDirect(double percent) {
    cargoIntakeWristRightMotor.set(ControlMode.PercentOutput, percent);
  }


  public void setWristSpeed(double speed) {

    double stallSpeed = wristStallOutput * Math.cos(Math.toRadians(getWristAngle()));
    cargoIntakeWristRightMotor.set(ControlMode.PercentOutput, (speed * 0.3) + wristStallOutput * Math.cos(Math.toRadians(getWristAngle())));



    /*
    if (getWristAngle() < 70) {
      cargoIntakeWristRightMotor.set(ControlMode.PercentOutput, (speed * 0.3) - 0.145);
    } else if (speed > 0.0 && getWristAngle() > 70) {
      cargoIntakeWristRightMotor.set(ControlMode.PercentOutput, (speed ) * 0.5);
    } else if (speed > 0.0) {
      cargoIntakeWristRightMotor.set(ControlMode.PercentOutput, (speed ) * 0.3 - 0.145);
    } else if (speed <= 0.0) {
      cargoIntakeWristRightMotor.set(ControlMode.PercentOutput, 0.0);
    }
    */

    /*
    if (getWristAngle() < 70) {
      cargoIntakeWristRightMotor.set(ControlMode.PercentOutput, (speed * 0.2) - 0.145);
    } else if (speed > 0.0 ) {
      cargoIntakeWristRightMotor.set(ControlMode.PercentOutput, (speed ) * 0.2 - 0.145);
    } else if (speed <= 0.0) {
      cargoIntakeWristRightMotor.set(ControlMode.PercentOutput, 0.0);
    }
    */

    /*
    if (speed < 0 && getWristAngle() < 5) {
      cargoIntakeWristRightMotor.set(ControlMode.PercentOutput, getWristAngle()/5 * speed);
    } else {
      cargoIntakeWristRightMotor.set(ControlMode.PercentOutput, speed);
    }
    */
  }
}
