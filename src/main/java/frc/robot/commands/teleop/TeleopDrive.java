package frc.robot.commands.teleop;


import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.commands.TurnToAngle;
import frc.robot.subsystems.Drive;


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
            Robot.drive.arcadeDrive(Robot.competitionController.getDriveY(), Robot.competitionController.getDriveX() * 0.6);
            if(Robot.competitionController.getTurnToAngle30()) {
                (new TurnToAngle(1.2, 0.8, 30)).start();
            }

            if(Robot.competitionController.getTurnToAngle90()) {
                (new TurnToAngle(1.2, 0.8, 90)).start();
            }

        } else if (Robot.drive.driveStateMachine == Drive.DriveStateMachine.PID) {
            //Nothing yet
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
