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
import frc.robot.subsystems.CargoIntakeWrist;


/**
 * An example command.  You can replace me with your own command.
 */
public class RotateIntakeWrist extends Command {
    private double angle;
    private double startTime;
    private double resetTime;

    public RotateIntakeWrist(double angle) {
        requires(Robot.cargoIntakeWrist);
        this.angle = angle;
    }

    @Override
    protected void initialize() {
        startTime=Timer.getFPGATimestamp();
        resetTime=Timer.getFPGATimestamp();
        Robot.cargoIntakeWrist.setWristAngle(angle);
        Robot.cargoIntakeWrist.cargoIntakeWristStateMachine = CargoIntakeWrist.CargoIntakeWristStateMachine.PID;
        Robot.cargoIntakeWrist.enable();
    }

    @Override
    protected void execute() {
        Robot.cargoIntakeWrist.PIDRotate();
    }

    @Override
    protected boolean isFinished() {
         if (Timer.getFPGATimestamp()-startTime >= 0.8 || Timer.getFPGATimestamp()-resetTime >=0.2) {
             return true;
         }
         if(!Robot.cargoIntakeWrist.onTarget()) {
             resetTime = Timer.getFPGATimestamp();
         }
         return false;
    }

    @Override
    protected void end() {
        Robot.cargoIntakeWrist.cargoIntakeWristStateMachine = CargoIntakeWrist.CargoIntakeWristStateMachine.MANUAL;
        Robot.cargoIntakeWrist.disable();
    }

    @Override
    protected void interrupted() {
        end();
    }
}
