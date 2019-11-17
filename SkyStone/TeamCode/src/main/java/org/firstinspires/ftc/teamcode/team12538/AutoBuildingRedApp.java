package org.firstinspires.ftc.teamcode.team12538;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.team12538.utils.AutoColor;

@Autonomous(name="Auto Red (Building Zone)", group="Linear Opmode")
public class AutoBuildingRedApp extends AutoBuildingZoneApp {
    public AutoBuildingRedApp() {
        super();
        super.autoColor = AutoColor.Red;
    }
}
