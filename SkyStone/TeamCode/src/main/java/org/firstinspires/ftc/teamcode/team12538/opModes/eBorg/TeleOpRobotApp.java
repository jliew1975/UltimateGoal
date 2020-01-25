package org.firstinspires.ftc.teamcode.team12538.opModes.eBorg;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.team12538.components.RobotStoneClaw;
import org.firstinspires.ftc.teamcode.team12538.robot.SkyStoneTeleOpRobot;
import org.firstinspires.ftc.teamcode.team12538.utils.OpModeStore;
import org.firstinspires.ftc.teamcode.team12538.utils.OpModeUtils;
import org.firstinspires.ftc.teamcode.team12538.utils.states.Button;

@TeleOp(name="Robot Tele", group="Linear Opmode")
public class TeleOpRobotApp extends RobotApp {
    private int stoneLevel = 1;

    Button dPadUp = new Button();
    Button dPadDown = new Button();
    Button btnX = new Button();

    @Override
    public void performRobotOperation() throws InterruptedException {
        try {
            // Tell global store that runMode is in TeleOp mode
            OpModeUtils.getGlobalStore().runMode = OpModeStore.RunMode.TeleOp;

            SkyStoneTeleOpRobot robot = new SkyStoneTeleOpRobot();
            robot.init();

            RobotStoneClaw.ARM_DEPLOYMENT_POSITION = 0.7;

            waitForStart();

            if(isStopRequested()) {
                return;
            }

            while (opModeIsActive()) {
                // mecanum drive controls
                robot.mecanumDrive.navigateWithGamepad(gamepad1);

                robot.intake.control(gamepad1);
                robot.outtake.control(gamepad1);
                robot.foundationClaw.control(gamepad2);
                robot.outtake.aligner.control(gamepad1);

                dPadUp.input(gamepad2.dpad_up);
                dPadDown.input(gamepad2.dpad_down);
                btnX.input(gamepad2.x);

                if(dPadUp.onPress()) {
                    robot.outtake.stoneHeight += 1;
                } else if(dPadDown.onPress()) {
                    robot.outtake.stoneHeight -= 1;
                } else if(btnX.onPress()) {
                    robot.outtake.stoneHeight = 1;
                }

                robot.capstone.control(gamepad2);


                // telemetry printing
                // robot.mecanumDrive.printTelemetry();
                // robot.outtakeSlides.printTelemetry();
                // robot.intake.printTelemetry();

                telemetry.addData("Stone Level", robot.outtake.stoneHeight);
                telemetry.update();
            }
        } finally {
            OpModeUtils.stop();
        }
    }
}
