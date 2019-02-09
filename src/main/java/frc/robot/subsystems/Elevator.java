/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import frc.robot.RobotMap;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.command.PIDSubsystem;

/**
 * An example subsystem.  You can replace me with your own Subsystem.
 */
public class Elevator extends PIDSubsystem {
  private double outPID = 0;
  private final double chainPitch = 0.25;
  private final double sprocketTeeth = 22;
  private final double outputRatio = 2.5;
  private final double encoderPPR = 2048;
  private final double baseHeight = 30; //Measure base height from the ground to the elevator.

  Encoder elevatorEncoder = new Encoder(RobotMap.elevatorEncoderA, RobotMap.elevatorEncoderB, false, Encoder.EncodingType.k4X);
  VictorSPX elevatorRightMotor = new VictorSPX(RobotMap.elevatorRightMotor);
  VictorSPX elevatorLeftMotor = new VictorSPX(RobotMap.elevatorLeftMotor);


  public Elevator() {
    super(0,0,0);
    setAbsoluteTolerance(1);
    getPIDController().setOutputRange(-1,1);
    elevatorLeftMotor.follow(elevatorRightMotor);
  }

  @Override
  public void initDefaultCommand() { }

  @Override
  public double returnPIDInput() {
    return elevatorHeight_cm();
  }

  public void setHeight(double height_cm) {
    this.setSetpoint(height_cm);
  }

  @Override
  public void usePIDOutput(double output) {
    this.outPID = output;
  }

  public double elevatorHeight_cm() {
    return baseHeight + (elevatorEncoder.get()/encoderPPR) / outputRatio * sprocketTeeth * chainPitch * 2.54 * 2;
  }

  public void reset() {
    elevatorEncoder.reset();
  }

}
