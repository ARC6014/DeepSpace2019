package frc.robot.commands.teleop;


import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;
import frc.robot.Robot;
import frc.robot.subsystems.CargoIntakeWrist;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Rotator;


public class TeleopDrive extends Command {
    public TeleopDrive() {
        requires(Robot.drive);
    }

    @Override
    protected void initialize() {
    }

    @Override
    protected void execute() {

        if (Robot.drive.driveStateMachine == Drive.DriveStateMachine.MANUAL){
            Robot.rotator.setAngle(Robot.competitionController.getDriveX() * 0.8);
            Robot.rotator.pidDrive(Robot.competitionController.getDriveY());
        }
    }

    @Override
    protected boolean isFinished (){
        return false;

    }

    @Override
    protected void end() {
        Robot.drive.arcadeDrive(0,0);
    }

    @Override
    protected void interrupted() {
        end();
    }

}
