package org.firstinspires.ftc.teamcode.team12538.robot_app;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.team12538.robotV1.AutoRobotTest;
import org.firstinspires.ftc.teamcode.team12538.utils.MotorUtils;
import org.firstinspires.ftc.teamcode.team12538.utils.OpModeUtils;
import org.firstinspires.ftc.teamcode.team12538.utils.ThreadUtils;

@TeleOp(name="Robot Tele (Test)", group="Linear Opmode")
@Disabled
public class RoverRuckusTeleOpTestApp extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();

    private Object lock = new Object();
    private volatile boolean busy = false;

    Servo hangLeg = null;
    DcMotor scissorLift = null;

    Servo hook = null;

    @Override
    public void runOpMode() throws InterruptedException {
        try {
            OpModeUtils.init(this);
            OpModeUtils.getGlobalStore().setDisableInitPos(true);

            AutoRobotTest testRobot = new AutoRobotTest();
            testRobot.init();

            hangLeg = hardwareMap.get(Servo.class, "hang_leg");

            scissorLift = hardwareMap.get(DcMotor.class, "lift");
            MotorUtils.setZeroPowerMode(DcMotor.ZeroPowerBehavior.BRAKE, scissorLift);
            MotorUtils.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER, scissorLift);
            MotorUtils.setMode(DcMotor.RunMode.RUN_USING_ENCODER, scissorLift);

            hook = hardwareMap.get(Servo.class, "latch");
            hook.setPosition(0.1);


            waitForStart();

            while (opModeIsActive()) {
                if(gamepad1.dpad_down) {
                    scissorLift.setPower(-1.0);
                    if(scissorLift.getCurrentPosition() < 5000) {
                        hangLeg.setPosition(0.1);
                    }
                } else if(gamepad1.dpad_up) {
                    scissorLift.setPower(1.0);
                    if(scissorLift.getCurrentPosition() > 5000) {
                        hangLeg.setPosition(1.0);
                    }
                } else {
                    scissorLift.setPower(0d);
                }

                if(gamepad1.x) {
                    hook.setPosition(0.5);
                } else if(gamepad1.a) {
                    hook.setPosition(0.1);
                }

                if(gamepad2.x) {
                    testRobot.strafeLeft(0.3, 10d);
                } else if(gamepad2.b) {
                    testRobot.strafeRight(0.3, 10d);
                } else {
                    testRobot.stop();
                }

                telemetry.addData("lift", scissorLift.getCurrentPosition());
                telemetry.update();
            }
        } finally {

        }
    }
}
