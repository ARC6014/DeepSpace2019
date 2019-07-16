package frc.robot.commands.teleop;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import frc.robot.Robot;
import frc.robot.commandgroups.motionmagic.*;
import frc.robot.commands.motionmagic.CalibrateElevator;
import frc.robot.subsystems.MotionMagicElevator;


public class MotionMagicLift extends Command {

    public MotionMagicLift() {
        requires(Robot.motionMagicElevator);
    }

    @Override
    protected void initialize() {

    }

    @Override
    protected void execute() {
        if(Robot.motionMagicElevator.motionMagicStateMachine == MotionMagicElevator.MotionMagicStateMachine.MANUAL) {
            Robot.motionMagicElevator.manualControl(Robot.competitionController.getCargoIntakeWrist());
        }
        else if(Robot.motionMagicElevator.motionMagicStateMachine == MotionMagicElevator.MotionMagicStateMachine.MOTION_MAGIC) {
            if(Robot.motionMagicElevator.getCalibrated()) {

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

                } else if (Robot.competitionController.getHatch1Xbox()) {
                    if(!Robot.elevatorActive){
                        (new Hatch1Xbox()).start();
                    }
                    Robot.elevatorActive = true;
                }
                else {
                    Robot.elevatorActive = false;
                }

                Robot.motionMagicElevator.AutoElevator();
            } else {
                Scheduler.getInstance().add(new CalibrateElevator());
            }
        }
    }

    @Override
    protected boolean isFinished() {
        return false;
    }

    @Override
    protected void end() {

    }

    @Override
    protected void interrupted() { end(); }
}
