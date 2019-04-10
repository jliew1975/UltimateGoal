package org.firstinspires.ftc.teamcode.team12538.robot_app;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.team12538.components.MineralMechanism;
import org.firstinspires.ftc.teamcode.team12538.robotV1.AutoRobotV1;
import org.firstinspires.ftc.teamcode.team12538.utils.OpModeUtils;

@TeleOp(name="Robot Tele (Test)", group="Linear Opmode")
public class RoverRuckusTeleOpTestApp extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        OpModeUtils.init(this);
        OpModeUtils.getGlobalStore().setCloseDepoArm(true);
        OpModeUtils.getGlobalStore().setDisableInitPos(false);
        OpModeUtils.getGlobalStore().setResetArmExtensionEncoderValue(true);

        try {
            AutoRobotV1 robot = new AutoRobotV1();
            robot.init();
            robot.init_imu();

            waitForStart();

            if(isStopRequested()) {
                return;
            }

            // tilt the phone for teleOp mode
            /*
            robot.getParkingRod().setPosition(1d);
            robot.getRobotLatch().powerLiftRunToPosition(1.0, 3900);
            sleep(1000);
            robot.getRobotLatch().powerLiftRunToPosition(1.0, 0);
            */
            MineralMechanism collector = robot.getCollector();
            while(opModeIsActive()) {
                if (gamepad1.x) {
                    robot.strafeRight(0.5, 10.0);
                } else if (gamepad1.b) {
                    robot.strafeLeft(0.5, 10.0);
                }

                telemetry.addData("Intake Pos", robot.getCollector().getIntake().getCurrentPosition());
                telemetry.update();
            }

        } finally {
            OpModeUtils.stop();
        }
    }
}
