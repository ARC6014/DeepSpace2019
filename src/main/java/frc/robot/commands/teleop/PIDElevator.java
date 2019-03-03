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

            if (Robot.elevator.getElevatorSwitchStatus()) {
                if (resetStatus = false) {
                    Robot.elevator.resetEncoder();
                    Robot.elevator.setSetpoint(Robot.elevator.baseToIntakeHeight);
                    resetStatus = true;
                }
                if (resetStatus && !Robot.elevator.getElevatorSwitchStatus()) {
                    resetStatus = false;
                }
            }

            if (Robot.competitionController.getIntakeCargo1RocketLevel()) {
                Robot.elevatorActive = true;
                (new RocketCargoL1()).start();
            } else if (Robot.competitionController.getIntakeCargo2RocketLevel()) {
                Robot.elevatorActive = true;
                (new RocketCargoL2()).start();
            } else if (Robot.competitionController.getIntakeCargo3RocketLevel()) {
                Robot.elevatorActive = true;
                (new RocketCargoL3()).start();
            } else if (Robot.competitionController.getIntakeHatch1RocketLevel()) {
                Robot.elevatorActive = true;
                (new RocketHatchL1()).start();
            } else if (Robot.competitionController.getIntakeHatch2RocketLevel()) {
                Robot.elevatorActive = true;
                (new RocketHatchL2()).start();
            } else if (Robot.competitionController.getIntakeHatch3RocketLevel()) {
                Robot.elevatorActive = true;
                (new RocketHatchL3()).start();
            } else if (Robot.competitionController.getIntakeCargoShipLevel()) {
                Robot.elevatorActive = true;
                (new CargoShipCargo()).start();
            } else if (Robot.competitionController.getIntakeLowestLevel()) {
                Robot.elevatorActive = true;
                (new IntakeBaseLevel()).start();
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
