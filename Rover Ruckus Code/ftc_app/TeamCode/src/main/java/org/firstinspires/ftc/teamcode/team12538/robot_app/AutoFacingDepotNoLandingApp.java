package org.firstinspires.ftc.teamcode.team12538.robot_app;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Autonomous(name="Auto (Facing Depot - No Landing)", group="Linear Opmode")
public class AutoFacingDepotNoLandingApp extends RoverRuckusAutoApp {
    @Override
    public void runOpMode() throws InterruptedException {
        super.enableLanding = false;
        super.runOpMode();
    }
}
