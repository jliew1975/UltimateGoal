package org.firstinspires.ftc.teamcode.team12538;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Auto Red (Loading Zone)", group="Linear Opmode")
public class AutoLoadingRedApp extends AutoLoadingZoneApp {
    public AutoLoadingRedApp() {
        super();
        super.autoColor = AutoColor.Red;
    }
}
