package org.firstinspires.ftc.teamcode.team12538.test;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.team12538.robotV1.AutoRobotTest;
import org.firstinspires.ftc.teamcode.team12538.utils.MotorUtils;
import org.firstinspires.ftc.teamcode.team12538.utils.OpModeUtils;

@TeleOp(name="Test App", group="Linear Opmode")
public class TestApp extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();

    private Object lock = new Object();
    private volatile boolean busy = false;

    DcMotor depoLift = null;

    Servo hook = null;

    @Override
    public void runOpMode() throws InterruptedException {
        try {
            OpModeUtils.init(this);
            OpModeUtils.getGlobalStore().setDisableInitPos(true);

            depoLift = hardwareMap.get(DcMotor.class, "depo_lift");
            MotorUtils.setZeroPowerMode(DcMotor.ZeroPowerBehavior.BRAKE, depoLift);
            MotorUtils.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER, depoLift);
            MotorUtils.setMode(DcMotor.RunMode.RUN_USING_ENCODER, depoLift);

            waitForStart();

            while (opModeIsActive()) {
                depoLift.setPower(-gamepad1.left_stick_y);

                telemetry.addData("depo_lift", depoLift.getCurrentPosition());
                telemetry.update();
            }
        } finally {

        }
    }
}
