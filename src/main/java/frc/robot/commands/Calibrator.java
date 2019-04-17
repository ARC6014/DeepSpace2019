package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

import java.io.BufferedWriter;
import java.io.FileWriter;
import java.io.IOException;

public class Calibrator extends Command {
    private double startTime;

    public Calibrator() {
    }

    @Override
    public void initialize() {
        startTime = Timer.getFPGATimestamp();
    }

    @Override
    public void execute() {
        Robot.motorValues.get(0).add(Timer.getFPGATimestamp()-startTime);
        Robot.motorValues.get(1).add(Robot.cargoIntakeWrist.getMotorSet());
        Robot.motorValues.get(2).add(Robot.drive.getLeftMotorSet());
        Robot.motorValues.get(3).add(Robot.drive.getRightMotorSet());
        Robot.motorValues.get(4).add(Robot.cargoIntake.getMotorSet());
        Robot.motorValues.get(5).add(Robot.elevator.getMotorSet());
    }

    @Override
    public void interrupted() {
        end();
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end() {
        try {
            BufferedWriter writer = new BufferedWriter(new FileWriter("/home/lvuser/deploy/testData.txt"));
            writer.write(Robot.motorValues.get(0).toString());
            writer.newLine();
            writer.write(Robot.motorValues.get(1).toString());
            writer.newLine();
            writer.write(Robot.motorValues.get(2).toString());
            writer.newLine();
            writer.write(Robot.motorValues.get(3).toString());
            writer.newLine();
            writer.write(Robot.motorValues.get(4).toString());
            writer.newLine();
            writer.write(Robot.motorValues.get(5).toString());
        } catch (IOException e) {
            //ERROR
        }
    }

}