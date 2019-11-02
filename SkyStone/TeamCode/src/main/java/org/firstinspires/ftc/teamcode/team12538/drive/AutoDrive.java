package org.firstinspires.ftc.teamcode.team12538.drive;

import org.firstinspires.ftc.teamcode.team12538.components.AutoGamepad;

public interface AutoDrive {
    void init();
    void printTelemetry();

    void resetEncoderValues();

    void stop();
    void autoNavigateWithGamepad(AutoGamepad gamepad);
}
