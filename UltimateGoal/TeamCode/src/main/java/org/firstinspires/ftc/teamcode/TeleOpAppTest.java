package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.components.Intake;
import org.firstinspires.ftc.teamcode.components.Shooter;
import org.firstinspires.ftc.teamcode.util.OpModeUtils;

@TeleOp(name="Test", group = "test")
public class TeleOpAppTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        OpModeUtils.init(this);

        Shooter shooter = new Shooter();
        shooter.init();

        Intake intake = new Intake(shooter);
        intake.init();

        waitForStart();

        if(isStopRequested()) {
            return;
        }

        while(opModeIsActive() && !isStopRequested()) {
            shooter.control(gamepad1);
            intake.control(gamepad2);

            telemetry.update();
        }
    }
}
