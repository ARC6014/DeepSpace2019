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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotMap;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.command.PIDSubsystem;

/**
 * An example subsystem.  You can replace me with your own Subsystem.
 */
public class Elevator extends PIDSubsystem {
  private double outPID = 0;

  VictorSPX elevatorMotor = new VictorSPX(RobotMap.elevatorMotor);

  Encoder elevatorEncoder = new Encoder(RobotMap.elevatorEncoderA, RobotMap.elevatorEncoderB, false, Encoder.EncodingType.k4X);
  DigitalInput elevatorBottomSwitch = new DigitalInput(RobotMap.elevatorBottomSwitch);

  private final double chainPitch = 0.250 * 2.54;
  private final int sprocketTeeth = 22;
  private final double outputRatio = 2.5;
  private final int encoderCPR = 2048 * 4; //Check the encoder values
  private final double baseToIntakeHeight = 39.12; //Measure base height from the ground to the elevator.
  private final double maxHeight = 194.05; //Check


  public Elevator() {
    super(0,0,0);
    setAbsoluteTolerance(1);
    getPIDController().setInputRange(baseToIntakeHeight,maxHeight);
    getPIDController().setOutputRange(-1,1);
  }

  @Override
  public void initDefaultCommand() { }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("ElevatorHeight", elevatorHeightCm());
    SmartDashboard.putNumber("ElevatorMotor",elevatorMotor.getMotorOutputPercent());
  }

  public void resetEncoder() {
    elevatorEncoder.reset();
  }

  public double getEncoderRev() {
    return elevatorEncoder.get() / (double)encoderCPR;
  }

  public double elevatorHeightCm() {
    return baseToIntakeHeight + (getEncoderRev() / outputRatio) * sprocketTeeth * chainPitch * 2;
  }

  public void setHeight(double heightCm) {
    this.setSetpoint(heightCm);
  }

  @Override
  protected double returnPIDInput() {
    return elevatorHeightCm();
  }

  @Override
  protected void usePIDOutput(double output) {
    this.outPID = output;
  }

  public void PIDLift() {
    setElevatorSpeed(outPID);
  }

  public void setElevatorSpeed(double speed) {
    elevatorMotor.set(ControlMode.PercentOutput, speed);
  }

}
