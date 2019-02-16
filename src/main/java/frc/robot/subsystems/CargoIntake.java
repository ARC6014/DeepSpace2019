/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;
import com.ctre.phoenix.motorcontrol.ControlMode;


public class CargoIntake extends Subsystem {
    

  
  VictorSPX cargoIntakeMotor = new VictorSPX(RobotMap.cargoIntakeMotor);

  @Override
  public void initDefaultCommand() {

  }
  public void setIntakeSpeed(double speed) {
    cargoIntakeMotor.set(ControlMode.PercentOutput, speed);

}
