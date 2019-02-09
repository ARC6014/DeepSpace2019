/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.command.PIDSubsystem;
import com.kauailabs.navx.frc.AHRS;
import frc.robot.RobotMap;

/**
 * An example subsystem.  You can replace me with your own Subsystem.
 */
public class Drive extends PIDSubsystem {
  private double outPID = 0;
  AHRS mxp = new AHRS(SPI.Port.kMXP);

  VictorSPX driveRearRightMotor = new VictorSPX(RobotMap.driveRearRightMotor);
  VictorSPX driveFrontRightMotor = new VictorSPX(RobotMap.driveFrontRightMotor);
  VictorSPX driveRearLeftMotor = new VictorSPX(RobotMap.driveRearLeftMotor);
  VictorSPX driveFrontLeftMotor = new VictorSPX(RobotMap.driveFrontLeftMotor);

  Encoder driveRightEncoder = new Encoder(RobotMap.driveRightEncoderA, RobotMap.driveRightEncoderB, false, Encoder.EncodingType.k4X);
  Encoder driveLeftEncoder = new Encoder(RobotMap.driveLeftEncoderA, RobotMap.driveLeftEncoderB, false, Encoder.EncodingType.k4X);

  public Drive() {
    super(0,0,0);
    setAbsoluteTolerance(5);
    getPIDController().setInputRange(180.0,-180.0);
    getPIDController().setOutputRange(-1,1);
    getPIDController().setContinuous(true);
  }

  @Override
  public void initDefaultCommand() { }

  public double getHeading() {
    return mxp.getYaw();
  }
  public void reset() {
    mxp.reset();
  }

  @Override
  public double returnPIDInput() {
   return getHeading();
  }

  @Override
  public void usePIDOutput(double output) {
    this.outPID = output;
  }
}
