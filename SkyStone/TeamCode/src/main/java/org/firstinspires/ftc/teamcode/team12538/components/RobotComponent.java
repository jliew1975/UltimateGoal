package org.firstinspires.ftc.teamcode.team12538.components;

import com.qualcomm.robotcore.hardware.Gamepad;

import java.util.concurrent.TimeUnit;

public interface RobotComponent {
    enum ClawMode { Open, Close }
    void init();
}
