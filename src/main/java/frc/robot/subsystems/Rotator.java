/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.command.PIDSubsystem;
import com.kauailabs.navx.frc.AHRS;
import frc.robot.Robot;

public class Rotator extends PIDSubsystem {

    AHRS mxp = new AHRS(SPI.Port.kMXP);
    private double outPID = 0;
    private boolean reset = true;
    public Rotator() {
        super(0,0,0);
        setAbsoluteTolerance(1.0);
        getPIDController().setInputRange(-180.0,180.0);
        getPIDController().setOutputRange(-1.0,1.0);
        getPIDController().setContinuous(true);
    }

    public void setAngle(double angle) {
        this.setSetpoint(angle);
    }

    @Override
    public void initDefaultCommand() {
    }

    public double getHeading() {
        return mxp.getYaw();
    }
    public void reset() {
        mxp.reset();
    }
    @Override
    protected double returnPIDInput() {
        return getHeading();
    }
    @Override
    protected void usePIDOutput(double output) {
        this.outPID = output;
    }
    public void pidDrive(double speed) {
        Robot.drive.arcadeDrive(speed, outPID);
    }
}
