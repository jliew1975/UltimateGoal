package org.firstinspires.ftc.teamcode.team12538;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.team12538.utils.AutoColor;

@Autonomous(name="Auto Blue (Loading Zone)", group="Linear Opmode")
public class AutoLoadingBlueApp extends AutoLoadingZoneApp {
    public AutoLoadingBlueApp() {
        super();
        super.autoColor = AutoColor.Blue;
    }
}
