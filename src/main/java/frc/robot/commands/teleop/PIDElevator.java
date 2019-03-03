/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.teleop;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.CargoIntakeWrist;
import frc.robot.subsystems.Elevator;


/**
 * An example command.  You can replace me with your own command.
 */
public class PIDElevator extends Command {

    public PIDElevator() {
        requires(Robot.elevator);
    }

    @Override
    protected void initialize() {

    }

    @Override
    protected void execute() {

        if (Robot.elevator.elevatorStateMachine == Elevator.ElevatorStateMachine.PID ){
            Robot.elevator.enable();
        }
        else{
            Robot.elevator.disable();
        }

        if (Robot.elevator.elevatorStateMachine == Elevator.ElevatorStateMachine.PID ){
            Robot.elevator.PIDLift();
        }
        else if (Robot.elevator.elevatorStateMachine== Elevator.ElevatorStateMachine.MANUAL){
            Robot.elevator.setElevatorSpeed (Robot.manualControl.getElevator());

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
