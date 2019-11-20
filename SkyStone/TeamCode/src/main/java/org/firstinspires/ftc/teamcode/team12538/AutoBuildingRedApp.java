package org.firstinspires.ftc.teamcode.team12538;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.team12538.utils.AutonomousColor;
import org.firstinspires.ftc.teamcode.team12538.utils.AutonomousMode;

@Autonomous(name="Red Building", group="Linear Opmode")
public class AutoBuildingRedApp extends AutoBuildingZoneApp {
    public AutoBuildingRedApp() {
        super();
        super.autoMode = AutonomousMode.RedBuilding;
        super.autoColor = AutonomousColor.Red;
    }
}
