package org.firstinspires.ftc.teamcode.team12538.drive;

import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.teamcode.team12538.ext.AutoGamepad;

public interface AutoDrive {
    void init();
    void init_imu();

    Position getPosition();

    void printTelemetry();
    void resetEncoderValues();

    double getAngle();

    void stop();
    boolean autoNavigateWithGamepad(AutoGamepad gamepad);
}
