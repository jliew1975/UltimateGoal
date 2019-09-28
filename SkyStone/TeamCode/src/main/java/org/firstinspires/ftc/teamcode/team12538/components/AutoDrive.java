package org.firstinspires.ftc.teamcode.team12538.components;

import org.firstinspires.ftc.teamcode.team12538.dogecv.VisionDetector;

public interface AutoDrive {
    void resetEncoderValues();
    void stop();

    void encoderStrafe(MecanumDrive.StrafingDirection direction, double speed, double distance, double timeout);
    void encoderStrafe(MecanumDrive.StrafingDirection direction, double speed, double distance, double timeout, VisionDetector detector);
}
