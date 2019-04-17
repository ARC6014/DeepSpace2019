package frc.robot.commandgroups;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.Robot;

import java.io.BufferedReader;
import java.io.FileReader;
import java.io.IOException;
import java.util.ArrayList;
import java.lang.String;

public class AutonomousReader extends CommandGroup {
    public AutonomousReader() {
        ArrayList<ArrayList<Double>> motorValues = new ArrayList<ArrayList<Double>>();

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

        for (int i = 0; i < motorValues.get(1).size(); i++) {
            Robot.cargoIntakeWrist.setMotorDirect(motorValues.get(1).get(i));
            Robot.drive.setLeftMotorDirect(motorValues.get(2).get(i));
            Robot.drive.setRightMotorDirect(motorValues.get(3).get(i));
            Robot.cargoIntake.setMotorDirect(motorValues.get(4).get(i));
            Robot.elevator.setMotorDirect(motorValues.get(5).get(i));
        }
    }
}
