/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;


/**
 * An example command.  You can replace me with your own command.
 */
public class WaitLiftElevator extends Command {
    private double height,startTime;

    public WaitLiftElevator(double height) {
        requires(Robot.elevator);
        this.height = height;
    }

    @Override
    protected void initialize() {
            Robot.elevator.setSetpoint(height);
            startTime=0;
    }

    @Override
    protected void execute() {
        Robot.elevator.PIDLift();
    }

    @Override
    protected boolean isFinished() {
        if (Timer.getFPGATimestamp()-startTime >= 0.2) {
            return true;
        }
        if(!Robot.elevator.onTarget()) {
            startTime = Timer.getFPGATimestamp();
        }
        return false;
    }

    @Override
    protected void end() {
    }

    @Override
    protected void interrupted() {
        end();
    }
}
