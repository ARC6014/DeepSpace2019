/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.teleop;

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.CargoIntake;
import frc.robot.subsystems.CargoIntakeWrist;
import frc.robot.subsystems.Elevator;


/**
 * An example command.  You can replace me with your own command.
 */
public class PIDIntakeWrist extends Command {

    public PIDIntakeWrist() {
    }

    @Override
    protected void initialize() {

    }

    @Override
    protected void execute() {
        if (Robot.cargoIntakeWrist.cargoIntakeWristStateMachine == CargoIntakeWrist.CargoIntakeWristStateMachine.PID ){
            Robot.cargoIntakeWrist.enable();
        }
        else{
            Robot.cargoIntakeWrist.disable();
        }

        if (Robot.cargoIntakeWrist.cargoIntakeWristStateMachine == CargoIntakeWrist.CargoIntakeWristStateMachine.PID ){
            Robot.cargoIntakeWrist.PIDRotate();
        }
        else if (Robot.cargoIntakeWrist.cargoIntakeWristStateMachine== CargoIntakeWrist.CargoIntakeWristStateMachine.MANUAL){
            Robot.cargoIntakeWrist.setWristSpeed(Robot.manualControl.getCargoIntakeWrist());
    }
    }

    @Override
    protected boolean isFinished() {
        return false;
    }

    @Override
    protected void end() {Robot.cargoIntakeWrist.setWristSpeed(0); }

    @Override
    protected void interrupted() { end(); }
}
