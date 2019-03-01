package frc.robot.commands.teleop;


import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;
import frc.robot.Robot;

public class TeleopDrive extends Command {
    private double y,x;
    public TeleopDrive() {

    }

    @Override
    protected void initialize() {
    }

    @Override
    protected void execute() {
        Robot.drive.arcadeDrive(y,x);
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
