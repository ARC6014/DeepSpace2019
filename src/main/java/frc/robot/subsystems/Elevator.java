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
import frc.robot.commands.teleop.PIDElevator;
import edu.wpi.first.wpilibj.PowerDistributionPanel;

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
  private final int encoderCPR = 2048; //Check the encoder values
  public final double baseToIntakeHeight = 41.5; //Measure base height from the ground to the elevator.
  public final double maxHeight = 194.05; //Check

//TODO: Integrate level calls for elevator; 8 levels total; possibly +/-
  public enum ElevatorStateMachine{
    DISABLED,
    MANUAL,
    PID

  }

  public ElevatorStateMachine elevatorStateMachine = ElevatorStateMachine.PID;


  public Elevator() {
    super(0.03,0,0.06,0.22,0.02);
    setAbsoluteTolerance(1);
    getPIDController().setInputRange(baseToIntakeHeight,maxHeight);
    getPIDController().setOutputRange(-0.8,0.8);
    elevatorMotor.setInverted(true);
    setSetpoint(baseToIntakeHeight);
    elevatorEncoder.setReverseDirection(true);
  }

  @Override
  public void initDefaultCommand() {
    setDefaultCommand(new PIDElevator());
  }



  @Override
  public void periodic() {
    if (getElevatorSwitchStatus()){
      resetEncoder();
      if(getSetpoint()<baseToIntakeHeight+10) {
        setSetpoint(baseToIntakeHeight);
      }
    }
    SmartDashboard.putNumber("Elevator Setpoint", getSetpoint());
    SmartDashboard.putBoolean("ElevatorBottomSwitch", getElevatorSwitchStatus());
    SmartDashboard.putNumber("ElevatorHeight", elevatorHeightCm());
    SmartDashboard.putNumber("ElevatorMotor",elevatorMotor.getMotorOutputPercent());
    SmartDashboard.putString("ElevatorState", elevatorStateMachine.toString());
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

  public boolean getElevatorSwitchStatus() {
    return !elevatorBottomSwitch.get();
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

  public void setElevatorSpeedManual(double speed) {
    elevatorMotor.set(ControlMode.PercentOutput,speed);
  }

  public double getMotorSet() {
    return elevatorMotor.getMotorOutputPercent();
  }
  public void setMotorDirect(double percent) {
    elevatorMotor.set(ControlMode.PercentOutput, percent);
  }

  public void setElevatorSpeed(double speed) {
    /*if (speed < 0.2 && elevatorHeightCm() < baseToIntakeHeight + 10) {
      elevatorMotor.set(ControlMode.PercentOutput, (elevatorHeightCm() - baseToIntakeHeight)/5 * speed);
    } else {
      elevatorMotor.set(ControlMode.PercentOutput, speed);
    }*/
    if (speed < 0 && elevatorHeightCm() < baseToIntakeHeight + 30) {
      elevatorMotor.set(ControlMode.PercentOutput, 0);
    } else {
      elevatorMotor.set(ControlMode.PercentOutput, speed);
    }
  }


}
