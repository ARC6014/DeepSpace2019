/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;
import frc.robot.commands.teleop.PIDIntakeWrist;
import frc.robot.commands.teleop.TeleopCargoIntake;

/**
 * An example subsystem.  You can replace me with your own Subsystem.
 */
public class CargoIntake extends Subsystem {

  VictorSPX cargoIntakeMotor = new VictorSPX(RobotMap.cargoIntakeMotor);

  public enum CargoIntakeStateMachine {
    DISABLED,
    MANUAL
  }
  public CargoIntakeStateMachine cargoIntakeStateMachine = CargoIntakeStateMachine.MANUAL;

  @Override
  public void initDefaultCommand() {
    setDefaultCommand(new TeleopCargoIntake());

  }

  public double getMotorSet() {
    return cargoIntakeMotor.getMotorOutputPercent();
  }

  public void setMotorDirect(double percent) {
    cargoIntakeMotor.set(ControlMode.PercentOutput, percent);
  }

  public void setIntakeSpeed(double speed)  {
    cargoIntakeMotor.set(ControlMode.PercentOutput, speed);
  }
}
