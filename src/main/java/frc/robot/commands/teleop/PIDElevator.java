/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.teleop;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import frc.robot.Robot;
import frc.robot.subsystems.CargoIntakeWrist;
import frc.robot.subsystems.Elevator;
import frc.robot.commandgroups.*;


/**
 * An example command.  You can replace me with your own command.
 */
public class PIDElevator extends Command {
    private boolean switcher = false;
    private boolean working = false;

    public PIDElevator() {
        requires(Robot.elevator);
    }

    @Override
    protected void initialize() {

    }

    @Override
    protected void execute() {

        if (Robot.competitionController.switchModes() && !switcher) {
            switcher = true;
            if (Robot.elevator.elevatorStateMachine == Elevator.ElevatorStateMachine.PID ) {
                Robot.elevator.elevatorStateMachine = Elevator.ElevatorStateMachine.MANUAL;
            } else if (Robot.elevator.elevatorStateMachine == Elevator.ElevatorStateMachine.MANUAL) {
                Robot.elevator.elevatorStateMachine = Elevator.ElevatorStateMachine.PID;
            }
        } else {
            switcher = false;
        }

        if (Robot.elevator.elevatorStateMachine == Elevator.ElevatorStateMachine.PID ){
            Robot.elevator.enable();
        }
        else{
            Robot.elevator.disable();
        }

        if (Robot.elevator.elevatorStateMachine == Elevator.ElevatorStateMachine.PID ){

            if (Robot.elevator.getElevatorSwitchStatus()) {
                Robot.elevator.resetEncoder();
            }

            if (Robot.competitionController.getIntakeCargo1RocketLevel()) {
                working = true;
                (new RocketCargoL1()).start();
            } else if (Robot.competitionController.getIntakeCargo2RocketLevel()) {
                working = true;
                (new RocketCargoL2()).start();
            } else if (Robot.competitionController.getIntakeCargo3RocketLevel()) {
                working = true;
                (new RocketCargoL3()).start();
            } else if (Robot.competitionController.getIntakeHatch1RocketLevel()) {
                working = true;
                (new RocketHatchL1()).start();
            } else if (Robot.competitionController.getIntakeHatch2RocketLevel()) {
                working = true;
                (new RocketHatchL2()).start();
            } else if (Robot.competitionController.getIntakeHatch3RocketLevel()) {
                working = true;
                (new RocketHatchL3()).start();
            } else if (Robot.competitionController.getIntakeCargoShipLevel()) {
                working = true;
                (new CargoShipCargo()).start();
            } else if (Robot.competitionController.getIntakeLowestLevel()) {
                working = true;
                (new IntakeBaseLevel()).start();
            } else if (Robot.competitionController.getHatchIntake()) {
                working = true;
                (new GetHatchIntake()).start();
            } else if (Robot.competitionController.getHatchPlace()) {
                working = true;
                (new GetHatchPlace()).start();
            } else {
                working = false;
            }

            Robot.elevator.PIDLift();
        }
        else if (Robot.elevator.elevatorStateMachine== Elevator.ElevatorStateMachine.MANUAL) {
            Robot.elevator.setElevatorSpeed (Robot.competitionController.getCargoIntakeWrist());
        }

    }


    @Override
    protected boolean isFinished() {
        return false;
    }

    @Override
    protected void end() { Robot.elevator.setElevatorSpeed(0); }

    @Override
    protected void interrupted() { end(); }
}
