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
import frc.robot.commands.FreeElevator;
import frc.robot.subsystems.CargoIntakeWrist;
import frc.robot.subsystems.Elevator;
import frc.robot.commandgroups.*;


/**
 * An example command.  You can replace me with your own command.
 */
public class PIDElevator extends Command {
    private boolean switcher = false;
    private boolean resetStatus = false;

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

            if (Robot.competitionController.getIntakeCargo1RocketLevel()) {
                if(!Robot.elevatorActive) {
                    (new RocketCargoL1()).start();
                }
                Robot.elevatorActive = true;
            } else if (Robot.competitionController.getIntakeCargo2RocketLevel()) {
                if(!Robot.elevatorActive) {
                    (new RocketCargoL2()).start();
                }
                Robot.elevatorActive = true;
            } else if (Robot.competitionController.getIntakeCargo3RocketLevel()) {
                if(!Robot.elevatorActive) {
                    (new RocketCargoL3()).start();
                }
                Robot.elevatorActive = true;
            } else if (Robot.competitionController.getIntakeHatch1RocketLevel()) {
                if(!Robot.elevatorActive) {
                    (new RocketHatchL1()).start();
                }
                Robot.elevatorActive = true;
            } else if (Robot.competitionController.getIntakeHatch2RocketLevel()) {
                if(!Robot.elevatorActive) {
                    (new RocketHatchL2()).start();
                }
                Robot.elevatorActive = true;
            } else if (Robot.competitionController.getIntakeHatch3RocketLevel()) {
                if(!Robot.elevatorActive) {
                    (new RocketHatchL3()).start();
                }
                Robot.elevatorActive = true;
            } else if (Robot.competitionController.getIntakeCargoShipLevel()) {
                if(!Robot.elevatorActive) {
                    (new CargoShipCargo()).start();
                }
                Robot.elevatorActive = true;
            } else if (Robot.competitionController.getIntakeLowestLevel()) {
                if(!Robot.elevatorActive) {
                    (new IntakeBaseLevel()).start();
                }
                Robot.elevatorActive = true;
            } else if (Robot.competitionController.getHatchIntake()) {
                if(!Robot.elevatorActive) {
                    (new GetHatchIntake()).start();
                }
                Robot.elevatorActive = true;
            } else if (Robot.competitionController.getHatchPlace()) {
                if(!Robot.elevatorActive){
                    (new GetHatchPlace()).start();
                }
                Robot.elevatorActive = true;
            } else {
                Robot.elevatorActive = false;
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
