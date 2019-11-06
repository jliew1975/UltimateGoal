package org.firstinspires.ftc.teamcode.team12538;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Auto Blue (Building Zone)", group="Linear Opmode")
public class AutoBuildingBlueApp extends AutoBuildingZoneApp {
    public AutoBuildingBlueApp() {
        super();
        super.autoColor = AutoColor.Blue;
    }
}
