package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;

public class CompetitionController {
    XboxController xbox = new XboxController(0);
    GenericHID.Hand rightStick = GenericHID.Hand.kRight;
    GenericHID.Hand leftStick = GenericHID.Hand.kLeft;
    Joystick joystick  = new Joystick(1);


    public double getDriveX() { return xbox.getX(leftStick); }
    public double getDriveY() { return -xbox.getY(leftStick); }

    public boolean getIntakeCargoShipLevel() {return xbox.getAButton();}
    public boolean getIntakeLowestLevel() {return xbox.getAButton();}
    public boolean getIntakeCargo1RocketLevel(){return joystick.getRawButton(11);}
    public boolean getIntakeHatch1RocketLevel(){return joystick.getRawButton(12);}
    public boolean getIntakeCargo2RocketLevel(){return joystick.getRawButton(9);}
    public boolean getIntakeHatch2RocketLevel(){return joystick.getRawButton(10);}
    public boolean getIntakeCargo3RocketLevel(){return joystick.getRawButton(7);}
    public boolean getIntakeHatch3RocketLevel(){return joystick.getRawButton(8);}

    public double getWrist() {return joystick.getY();}


    public boolean getCargoPlace() {return xbox.getBumper(leftStick);}
    public boolean getHatchPlace() {return xbox.getBumper(rightStick);}
    public boolean getCargoIntake() {return xbox.getTriggerAxis(leftStick) >= 0.5;}
    public boolean getHatchIntake() {return xbox.getTriggerAxis(rightStick) >= 0.5;}

    public boolean getLaunch() { return xbox.getBumper(rightStick); }

    public double getElevator() { return -xbox.getY(rightStick); }

    public double getCargoIntakeWrist() { return xbox.getX(rightStick); }



}
