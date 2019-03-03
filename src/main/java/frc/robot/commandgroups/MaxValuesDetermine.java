package frc.robot.commandgroups;

import frc.robot.Robot;
import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.commands.Testing;
import frc.robot.commands.Delay;
import java.io.*;
import java.util.ArrayList;
import java.util.concurrent.TimeUnit;


public class MaxValuesDetermine extends CommandGroup{
    private double testTime = 5;

    public MaxValuesDetermine () {
        double dt = Robot.dt;

        for(int i = 0; i < (testTime/dt); i++) {
            addSequential(new Testing());
            Robot.positions.get(0).add((Robot.drive.getLeftEncoderRev() * Robot.encoderTicksPerRev));
            Robot.positions.get(1).add((Robot.drive.getRightEncoderRev() * Robot.encoderTicksPerRev));
        }
        Robot.drive.arcadeDrive(0,0);


        ArrayList<Double> position = new ArrayList<Double>();
        for (int i = 0; i < Robot.positions.get(0).size(); i++) {
            position.add((Robot.positions.get(0).get(i) + Robot.positions.get(1).get(i)) / 2);
        }


        ArrayList<Double> velocity = new ArrayList<Double>();
        for (int i = 0; i < position.size() - 1; i++) {
            velocity.add((position.get(i) + position.get(i + 1)) / dt);
        }

        double maxVel = 0;
        for (int i = 0; i < velocity.size(); i++) {
            if (velocity.get(i) > maxVel) {
                maxVel = velocity.get(i);
            }
        }
        ArrayList<Double> acceleration = new ArrayList<Double>();
        for (int i = 0; i < velocity.size() - 1; i++) {
            acceleration.add((velocity.get(i) + velocity.get(i + 1)) / dt);
        }

        double maxAcc = 0;
        for (int i = 0; i < acceleration.size(); i++) {
            if (acceleration.get(i) > maxAcc) {
                maxAcc = acceleration.get(i);
            }
        }
        ArrayList<Double> jerk = new ArrayList<Double>();
        for (int i = 0; i < acceleration.size() - 1; i++) {
            jerk.add((acceleration.get(i) + acceleration.get(i + 1)) / dt);
        }

        double maxJerk = 0;
        for (int i = 0; i < jerk.size(); i++) {
            if (jerk.get(i) > maxJerk) {
                maxJerk = jerk.get(i);
            }
        }

        try
        {
            BufferedWriter writer = new BufferedWriter(new FileWriter("/home/lvuser/deploy/testData.txt"));
            writer.newLine();
            writer.write(Robot.positions.get(0).toString());
            writer.newLine();
            writer.write(Robot.positions.get(1).toString());
            writer.newLine();
            writer.write(position.toString());
            writer.newLine();
            writer.write(velocity.toString());
            writer.newLine();
            writer.write(acceleration.toString());
            writer.newLine();
            writer.write(jerk.toString());
            writer.newLine();
            writer.write(String.valueOf(maxVel));
            writer.newLine();
            writer.write(String.valueOf(maxAcc));
            writer.newLine();
            writer.write(String.valueOf(maxJerk));
            writer.newLine();
            writer.close();
        } catch (IOException e) {
            //ERROR
        }


    }
}
