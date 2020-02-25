package org.firstinspires.ftc.teamcode.team12538.opModes.eBorg;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.team12538.utils.AutonomousColor;
import org.firstinspires.ftc.teamcode.team12538.utils.AutonomousMode;

@Autonomous(name="Red (Foundation)", group="Group 4")
public class AutoBuildingRedApp extends AutoBuildingZoneApp {
    public AutoBuildingRedApp() {
        super();
        super.autoMode = AutonomousMode.RedBuilding;
        super.autoColor = AutonomousColor.Red;
    }
}
