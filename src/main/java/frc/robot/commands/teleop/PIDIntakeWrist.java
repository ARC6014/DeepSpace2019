/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.teleop;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.commands.RotateIntakeWrist;
import frc.robot.subsystems.CargoIntakeWrist;


/**
 * An example command.  You can replace me with your own command.
 */
public class PIDIntakeWrist extends Command {
    private boolean switcher = false;

    public PIDIntakeWrist() {
        requires(Robot.cargoIntakeWrist);
    }

    @Override
    protected void initialize() {

    }

    @Override
    protected void execute() {
        if (Robot.competitionController.switchModes() && !switcher) {
            switcher = true;
            if (Robot.cargoIntakeWrist.cargoIntakeWristStateMachine == CargoIntakeWrist.CargoIntakeWristStateMachine.PID) {
                Robot.cargoIntakeWrist.cargoIntakeWristStateMachine = CargoIntakeWrist.CargoIntakeWristStateMachine.MANUAL;
            } else if (Robot.cargoIntakeWrist.cargoIntakeWristStateMachine == CargoIntakeWrist.CargoIntakeWristStateMachine.MANUAL) {
                Robot.cargoIntakeWrist.cargoIntakeWristStateMachine = CargoIntakeWrist.CargoIntakeWristStateMachine.PID;
            }
        } else {
            switcher = false;
        }

        if (Robot.competitionController.getToSetAngle30()) {
            (new RotateIntakeWrist(30)).start();
        } else if (Robot.competitionController.getToSetAngle70()) {
            (new RotateIntakeWrist(70)).start();
        }

        if (Robot.cargoIntakeWrist.cargoIntakeWristStateMachine == CargoIntakeWrist.CargoIntakeWristStateMachine.PID ){
            Robot.cargoIntakeWrist.enable();
        }
        else {
            Robot.cargoIntakeWrist.disable();
        }

        if (Robot.cargoIntakeWrist.cargoIntakeWristStateMachine == CargoIntakeWrist.CargoIntakeWristStateMachine.PID ){
            Robot.cargoIntakeWrist.PIDRotate();
            //Robot.elevator.setElevatorSpeed (Robot.competitionController.getCargoIntakeWrist());
        }
        else if (Robot.cargoIntakeWrist.cargoIntakeWristStateMachine== CargoIntakeWrist.CargoIntakeWristStateMachine.MANUAL){
            Robot.cargoIntakeWrist.setWristSpeed(Robot.competitionController.getCargoIntakeWrist() * 0.75);
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
