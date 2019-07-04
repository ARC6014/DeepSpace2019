package frc.robot.subsystems.baseclasses;

import edu.wpi.first.wpilibj.Encoder;

//
//Base Class for Encoders
//ARC6014
//

public class EncoderBase extends Encoder{

    double ENCODER_CPR = 2048;

    public EncoderBase(int inputA, int inputB) {
        super(inputA, inputB, false, Encoder.EncodingType.k4X);
    }

    //
    //Common Methods
    //

    public void resetEncoder() {
        this.reset();
    }

    public double getEncoderRev() {
        return this.get() / ENCODER_CPR;
    }

    //
    //Elevator
    //

    public double getElevatorHeight(double BASE_TO_INTAKE_HEIGHT, double OUTPUT_RATIO, double SPROCKET_TEETH, double CHAIN_PITCH) {
        return BASE_TO_INTAKE_HEIGHT + (getEncoderRev() / OUTPUT_RATIO) * SPROCKET_TEETH * CHAIN_PITCH * 2;
    }

    //
    //Drive
    //

    public double getEncoderDistance(double WHEEL_DIAMETER) {
        return getEncoderRev() * WHEEL_DIAMETER * Math.PI;
    }

    //
    //Rotatory Wrist
    //

    public double getWristAngle(double MAX_WRIST_ANGLE, double OUTPUT_RATIO, boolean switchPressed) {
        if(switchPressed) {
            return getEncoderRev() / OUTPUT_RATIO * 360;
        }
        //only if the wrist starts from the maximum wrist angle
        return MAX_WRIST_ANGLE + (getEncoderRev() / OUTPUT_RATIO * 360);
    }

}
