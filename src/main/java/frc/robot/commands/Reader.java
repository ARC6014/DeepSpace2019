package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

import java.io.BufferedReader;
import java.io.FileReader;
import java.io.IOException;
import java.util.ArrayList;

public class Reader extends Command {
    private ArrayList<ArrayList<Double>> motorValues = new ArrayList<ArrayList<Double>>();
    private int it;

    public Reader() {
        requires(Robot.elevator);
        requires(Robot.drive);
        requires(Robot.cargoIntake);
        requires(Robot.cargoIntakeWrist);
    }

    @Override
    public void initialize() {
        it = 0;
        try {
            BufferedReader reader = new BufferedReader(new FileReader("/home/lvuser/deploy/testData.txt"));
            for(int i = 0; i < 6; i++) {
                String line = reader.readLine();
                int lastComma = -1;
                for(int j = 2; j < line.length(); j++) {
                    if (line.substring(j, j+1) == "," || line.substring(j, j+1) == "]") {
                        double number = Double.parseDouble(line.substring(lastComma + 2, j));
                        lastComma = j;
                        motorValues.get(i).add(number);
                    }
                }
            }


        } catch (IOException e) {

        }
    }

    @Override
    public void execute() {
        Robot.cargoIntakeWrist.setMotorDirect(motorValues.get(1).get(it));
        Robot.drive.setLeftMotorDirect(motorValues.get(2).get(it));
        Robot.drive.setRightMotorDirect(motorValues.get(3).get(it));
        Robot.cargoIntake.setMotorDirect(motorValues.get(4).get(it));
        Robot.elevator.setMotorDirect(motorValues.get(5).get(it));
        it++;
    }

    @Override
    public void interrupted() {
        end();
    }

    @Override
    public boolean isFinished() {
        return it == motorValues.get(1).size();
    }

    @Override
    public void end() {

    }
}
