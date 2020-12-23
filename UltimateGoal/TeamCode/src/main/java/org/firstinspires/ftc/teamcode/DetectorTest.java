package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.detectors.Detector;
import org.firstinspires.ftc.teamcode.detectors.FtcLibRingsDetector;
import org.firstinspires.ftc.teamcode.detectors.OpenCVStarterRingsDetector;
import org.firstinspires.ftc.teamcode.util.OpModeUtils;

@Autonomous(name="DetectorTest", group = "test")
public class DetectorTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        OpModeUtils.init(this);

        Detector detector = new OpenCVStarterRingsDetector();
        detector.init();
        detector.activate();

        waitForStart();

        detector.deactivate();

        while(opModeIsActive()) {
            telemetry.addData("RingCount", detector.getRingCount());
            telemetry.update();
        }
    }
}
