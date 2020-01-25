package org.firstinspires.ftc.teamcode.team12538.opModes.eBorg;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.team12538.utils.AutonomousColor;
import org.firstinspires.ftc.teamcode.team12538.utils.AutonomousMode;

@Autonomous(name="Building Blue", group="Linear Opmode")
public class AutoBuildingBlueApp extends AutoBuildingZoneApp {
    public AutoBuildingBlueApp() {
        super();
        super.autoMode = AutonomousMode.BlueBuilding;
        super.autoColor = AutonomousColor.Blue;
    }
}
