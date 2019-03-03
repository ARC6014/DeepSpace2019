package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;

public class ManualController {

    XboxController xboxDrive = new XboxController(0);
    XboxController xboxElevator = new XboxController(1);
    GenericHID.Hand rightStick = GenericHID.Hand.kRight;
    GenericHID.Hand leftStick = GenericHID.Hand.kLeft;


    public double getDriveX() { return xboxDrive.getX(rightStick); }
    public double getDriveY() { return -xboxDrive.getY(leftStick); }

    public boolean getCargoIntake() { return xboxDrive.getBumper(rightStick); }
    public boolean getLaunch() { return xboxDrive.getBumper(leftStick); }

    public double getElevator() { return -xboxElevator.getY(rightStick); }

    public double getCargoIntakeWrist() { return xboxElevator.getX(rightStick); }



}
