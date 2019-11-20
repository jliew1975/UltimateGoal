package org.firstinspires.ftc.teamcode.team12538;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.team12538.utils.AutonomousColor;
import org.firstinspires.ftc.teamcode.team12538.utils.AutonomousMode;

@Autonomous(name="Blue Building (Skystone)", group="Linear Opmode")
public class AutoBuildingBlueWithSkystoneApp extends AutoBuildingZoneApp {
    public AutoBuildingBlueWithSkystoneApp() {
        super();
        super.autoMode = AutonomousMode.BlueBuilding;
        super.autoColor = AutonomousColor.Blue;
        super.pickupSkystone = true;
    }
}
