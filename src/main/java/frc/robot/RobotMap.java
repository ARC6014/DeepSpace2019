/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.SPI;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap {
    //CAN
    public static final int driveFrontLeftMotor = 0;
    public static final int driveRearLeftMotor = 0;
    public static final int driveFrontRightMotor = 0;
    public static final int driveRearRightMotor = 0;

    public static final int elevatorMotor = 0;

    public static final int cargoIntakeMotor = 0;

    public static final int cargoIntakeWristRightMotor = 0;
    public static final int cargoIntakeWristLeftMotor = 0;

    public static final int hatchWristMotor = 0;
    //DIO
    public static final int driveLeftEncoderA = 0;
    public static final int driveLeftEncoderB = 0;
    public static final int driveRightEncoderA = 0;
    public static final int driveRightEncoderB = 0;

    public static final int elevatorEncoderA = 0;
    public static final int elevatorEncoderB = 0;
    public static final int elevatorBottomSwitch = 0;

    public static final int cargoIntakeWristEncoderA = 0;
    public static final int cargoIntakeWristEncoderB = 0;
    public static final int cargoIntakeWristBottomSwitch = 0;

    public static final int hatchWristBottomSwitch = 0;
    //Hatch Wrist Encoder
    //NavX
    public static final SPI.Port navx = SPI.Port.kMXP;
}
