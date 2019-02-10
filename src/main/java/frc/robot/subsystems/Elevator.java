/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotMap;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.command.PIDSubsystem;

/**
 * An example subsystem.  You can replace me with your own Subsystem.
 */
public class Elevator extends PIDSubsystem {
  private double outPID = 0;

  VictorSPX elevatorRightMotor = new VictorSPX(RobotMap.elevatorRightMotor);
  VictorSPX elevatorLeftMotor = new VictorSPX(RobotMap.elevatorLeftMotor);

  Encoder elevatorEncoder = new Encoder(RobotMap.elevatorEncoderA, RobotMap.elevatorEncoderB, false, Encoder.EncodingType.k4X);

  private final double chainPitch = 0.250 * 2.54;
  private final int sprocketTeeth = 22;
  private final double outputRatio = 2.5;
  private final int encoderCPR = 2048 * 4; //Check the encoder values
  private final double baseToIntakeHeight = 30; //Measure base height from the ground to the elevator.
  private final double maxheight = 196; //Check


  public Elevator() {
    super(0,0,0);
    setAbsoluteTolerance(1);
    getPIDController().setInputRange(baseToIntakeHeight,maxheight);
    getPIDController().setOutputRange(-1,1);
    elevatorLeftMotor.follow(elevatorRightMotor);
  }

  @Override
  public void initDefaultCommand() { }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("ElevatorHeight", elevatorHeight_cm());
  }

  public void reset() {
    elevatorEncoder.reset();
  }

  public double getEncoderRev() {
    return elevatorEncoder.get() / (double)encoderCPR;
  }

  public double elevatorHeight_cm() {
    return baseToIntakeHeight + (getEncoderRev() / outputRatio) * sprocketTeeth * chainPitch * 2;
  }



  public void setHeight(double height_cm) {
    this.setSetpoint(height_cm);
  }

  @Override
  protected double returnPIDInput() {
    return elevatorHeight_cm();
  }

  @Override
  protected void usePIDOutput(double output) {
    this.outPID = output;
  }

  public void PIDLift() {
    setElevatorSpeed(outPID);
  }

  public void setElevatorSpeed(double speed) {
    elevatorRightMotor.set(ControlMode.PercentOutput, speed);
  }

}
