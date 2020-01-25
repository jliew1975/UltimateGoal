package org.firstinspires.ftc.teamcode.team12538.opModes.eBorg;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.team12538.drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.team12538.ext.AutoGamepad;
import org.firstinspires.ftc.teamcode.team12538.robot.SkyStoneAutoRobot;
import org.firstinspires.ftc.teamcode.team12538.utils.AutoGamepadUtils;
import org.firstinspires.ftc.teamcode.team12538.utils.AutonomousColor;
import org.firstinspires.ftc.teamcode.team12538.utils.AutonomousMode;

@Autonomous(name="Parking Only", group="Linear Opmode")
public class AutoParkingOnlyApp extends RobotApp {
    public AutoParkingOnlyApp() {
        super();
        super.autoMode = AutonomousMode.BlueBuilding;
        super.autoColor = AutonomousColor.Blue;
    }

    @Override
    public void performRobotOperation() throws InterruptedException {
        SkyStoneAutoRobot robot = new SkyStoneAutoRobot();
        robot.init();

        AutoGamepad gamepad = new AutoGamepad();

        waitForStart();

        // Navigate halfway to foundation
        AutoGamepadUtils.move(gamepad, MecanumDrive.AutoDirection.Forward, 0.5,10);
        robot.mecanumDrive.autoNavigateWithGamepad(gamepad);
    }
}
