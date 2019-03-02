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
import edu.wpi.first.wpilibj.command.PIDSubsystem;
import edu.wpi.first.wpilibj.Encoder;
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

  double maxWristAngle = 130;
  private final int encoderCPR = 2048 * 4;
  private final double outputRatio = 5.0;

  public enum CargoIntakeWristStateMachine{
    DISABLED,
    MANUAL,
    PID
  }
  public CargoIntakeWristStateMachine cargoIntakeWristStateMachine = CargoIntakeWristStateMachine.MANUAL;

  public CargoIntakeWrist() {
    super(0,0,0);
    setAbsoluteTolerance(1);
    getPIDController().setInputRange(0,maxWristAngle);
    getPIDController().setOutputRange(-1,1);
    cargoIntakeWristLeftMotor.setInverted(true);
    cargoIntakeWristLeftMotor.follow(cargoIntakeWristRightMotor);
  }
  @Override
  public void initDefaultCommand() {
    setDefaultCommand(new PIDIntakeWrist());
  }

  @Override
  public void periodic() {
    if(cargoIntakeWristBottomSwitch.get()){
      resetEncoder();
    }


    SmartDashboard.putNumber("CargoWristMotor", cargoIntakeWristRightMotor.getMotorOutputPercent());
    SmartDashboard.putNumber("WristAngle", getWristAngle());
    SmartDashboard.putBoolean("CargoIntakeParallel", getCargoSwitchStatus());
  }

  public void resetEncoder() {
    cargoIntakeWristEncoder.reset();
  }

  public double getEncoderRev() {
    return cargoIntakeWristEncoder.get() / (double)encoderCPR;
  }

  public double getWristAngle() {
    return getEncoderRev() / outputRatio * 360;
  }

  public boolean getCargoSwitchStatus() {
    return cargoIntakeWristBottomSwitch.get();
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
    setWristSpeed(outPID);
  }

  public void setWristSpeed(double speed) {
    if (speed < 0 && getWristAngle() < 5) {
      cargoIntakeWristRightMotor.set(ControlMode.PercentOutput, getWristAngle()/5 * speed);
    } else {
      cargoIntakeWristRightMotor.set(ControlMode.PercentOutput, speed);
    }
  }
}
