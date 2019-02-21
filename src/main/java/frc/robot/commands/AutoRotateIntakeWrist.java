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
public class AutoRotateIntakeWrist extends Command {
    private double angle;

    public AutoRotateIntakeWrist(double angle) {
        this.angle = angle;
    }

    @Override
    protected void initialize() {
        Robot.cargoIntakeWrist.enable();
        Robot.cargoIntakeWrist.setWristAngle(angle);
    }

    @Override
    protected void execute() {
        Robot.cargoIntakeWrist.PIDRotate();
    }

    @Override
    protected boolean isFinished() {
        if (Robot.cargoIntakeWrist.onTarget()) {
            return true;
        } else {
            return false;
        }
    }

    @Override
    protected void end() {
        Robot.cargoIntakeWrist.disable();
    }

    @Override
    protected void interrupted() {
        Robot.cargoIntakeWrist.disable();
    }
}
