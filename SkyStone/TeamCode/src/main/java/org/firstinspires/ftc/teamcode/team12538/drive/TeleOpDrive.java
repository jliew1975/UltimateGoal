package org.firstinspires.ftc.teamcode.team12538.drive;

import com.qualcomm.robotcore.hardware.Gamepad;

public interface TeleOpDrive {
    void init();
    void init_imu();
    void printTelemetry();
    void navigateWithGamepad(Gamepad gamepad);
    void resetEncoderValues();

    default void adjustAngle() {}
}
