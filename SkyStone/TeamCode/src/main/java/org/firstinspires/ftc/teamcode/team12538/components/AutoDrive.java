package org.firstinspires.ftc.teamcode.team12538.components;

import static org.firstinspires.ftc.teamcode.team12538.components.MecanumDrive.StrafingDirection;

import org.firstinspires.ftc.teamcode.team12538.dogecv.VisionDetector;

public interface AutoDrive {
    void resetEncoderValues();
    void stop();

    void encoderDrive(double speed, double distanceInInches, double timeout);
    void encoderDrive(double speed, double distanceInInches, double timeout, VisionDetector detector);
    void encoderStrafe(StrafingDirection direction, double speed, double distanceInInches, double timeout);
    void encoderStrafe(StrafingDirection direction, double speed, double distanceInInches, double timeout, VisionDetector detector);
}
