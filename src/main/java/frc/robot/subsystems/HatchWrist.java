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
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotMap;

/**
 * An example subsystem.  You can replace me with your own Subsystem.
 */
public class HatchWrist extends Subsystem {

  VictorSPX hatchWristMotor = new VictorSPX(RobotMap.hatchWristMotor);
  DigitalInput hatchWristBottomSwitch = new DigitalInput(RobotMap.hatchWristBottomSwitch);

  @Override
  public void initDefaultCommand() { }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("HatchIntakeDown", getSwitchStatus());
    SmartDashboard.putNumber("HatchWristMotor",hatchWristMotor.getMotorOutputPercent());
  }

  public boolean getSwitchStatus() {
    return hatchWristBottomSwitch.get();
  }

  //resetEncoder

  public void setHatchWristSpeed(double speed) {
    hatchWristMotor.set(ControlMode.PercentOutput, speed);
  }

}
