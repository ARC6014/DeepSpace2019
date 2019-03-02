package frc.robot.commands.teleop;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.CargoIntake;

public class TeleopCargoIntake  extends Command{
    private double power;

    public TeleopCargoIntake() {}

    @Override
    protected void initialize() {}

    @Override
    protected void execute() {
        if (Robot.cargoIntake.cargoIntakeStateMachine== CargoIntake.CargoIntakeStateMachine.MANUAL){
            if(Robot.manualControl.getCargoIntake()) {
                Robot.cargoIntake.setIntakeSpeed(1.0);
            } else if(Robot.manualControl.getLaunch()) {
                Robot.cargoIntake.setIntakeSpeed(-1.0);
            } else {
                Robot.cargoIntake.setIntakeSpeed(0);
            }

        }
    }

    @Override
    protected boolean isFinished() {
        return false;
    }

    @Override
    protected void end() {
        Robot.cargoIntake.setIntakeSpeed(0);
    }

    @Override
    protected void interrupted() { end(); }
}
