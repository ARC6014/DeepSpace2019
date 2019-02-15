/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;

/**
 * An example subsystem.  You can replace me with your own Subsystem.
 */
public class CargoIntakeWrist extends PIDSubsystem {
    double outPID = 0;
    VictorSPX cargoIntakeWristLeftMotor = new VictorSPX(RobotMap.cargoIntakeWristLeftMotor);
    VictorSPX cargoIntakeWristRightMotor = new VictorSPX(RobotMap.cargoIntakeWristRightMotor);
    Encoder cargoIntakeWristEncoder = new Encoder(RobotMap.cargoIntakeWristEncoderA, RobotMap.cargoIntakeWristEncoderB, false, Encoder.EncodingType.k4X);
    DigitalInput cargoIntakeWristBottomSwitch = new DigitalInput(RobotMap.cargoIntakeWristBottomSwitch);
    
    
    
    public CargoIntakeWrist() {
    super(0,0,0);
    setAbsoluteTolerance(1);
    getPIDController().setInputRange(0,maxWristAngle);
    getPIDController().setOutputRange(-1,1);
    
    public void initDefaultCommand() {
  }

    public void periodic() {
    SmartDashboard.putNumber("WristAngle", getWristAngle());
    
  }
 public void resetEncoder() {
    cargoIntakeWristEncoder.reset();
  }

  public double getEncoderRev() {
    return cargoIntakeWristEncoder.get() / (double)encoderCPR;
  }
  public double getWristAngle() {
  }
  public void PIDRotate() {
    setWristSpeed(outPID);
  }




  }

    

  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
