package org.firstinspires.ftc.teamcode.team12538.drive;

import org.firstinspires.ftc.teamcode.team12538.ext.AutoGamepad;

public interface AutoDrive {
    void init();
    void init_imu();

    void printTelemetry();
    void resetEncoderValues();

    void stop();
    void autoNavigateWithGamepad(AutoGamepad gamepad);
    void flipLastAngleForErrorCorrection(MecanumDrive.LastAngleMode lastAngleMode);
}
