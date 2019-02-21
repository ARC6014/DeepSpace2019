/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;


/**
 * An example command.  You can replace me with your own command.
 */
public class AutoLiftElevator extends Command {
    private double height;

    public AutoLiftElevator(double height) {
        this.height = height;
    }

    @Override
    protected void initialize() {
        Robot.elevator.enable();
        Robot.elevator.setSetpoint(height);
    }

    @Override
    protected void execute() {
        Robot.elevator.PIDLift();
    }

    @Override
    protected boolean isFinished() {
        if (Robot.elevator.onTarget()) {
            return true;
        } else {
            return false;
        }
    }

    @Override
    protected void end() {
        Robot.elevator.disable();
    }

    @Override
    protected void interrupted() {
        Robot.elevator.disable();
    }
}
