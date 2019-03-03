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
    public static final int driveFrontLeftMotor = 1;
    public static final int driveRearLeftMotor = 2;
    public static final int driveFrontRightMotor = 8;
    public static final int driveRearRightMotor = 5;

    public static final int elevatorMotor = 6;

    public static final int cargoIntakeMotor = 7;

    public static final int cargoIntakeWristRightMotor = 4;
    public static final int cargoIntakeWristLeftMotor = 3;
    //DIO
    public static final int driveLeftEncoderA = 0;
    public static final int driveLeftEncoderB = 1;
    public static final int driveRightEncoderA = 2;
    public static final int driveRightEncoderB = 3;

    public static final int elevatorEncoderA = 4;
    public static final int elevatorEncoderB = 5;
    public static final int elevatorBottomSwitch = 9;

    public static final int cargoIntakeWristEncoderA = 6;
    public static final int cargoIntakeWristEncoderB = 7;
    public static final int cargoIntakeWristBottomSwitch = 8;

    //NavX
    public static final SPI.Port navx = SPI.Port.kMXP;
}
