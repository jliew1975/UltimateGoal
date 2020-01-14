package org.firstinspires.ftc.teamcode.team12538.test;

import com.disnodeteam.dogecv.detectors.skystone.SkystoneDetector;
import com.disnodeteam.dogecv.detectors.skystone.StoneDetector;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.team12538.detectors.opencv.OpenCvDetector;
import org.firstinspires.ftc.teamcode.team12538.utils.OpModeUtils;
import org.opencv.core.Point;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;


@TeleOp(name = "OpenCV WebCam Test", group = "Test")
@Disabled
public class OpenCVWebCamTest extends LinearOpMode {
    enum DetectMode { Stone, SkyStone }

    @Override
    public void runOpMode() throws InterruptedException {
        OpModeUtils.init(this);

        OpenCvDetector detector = new OpenCvDetector();
        detector.init();
        detector.activate();

        /*
         * Wait for the user to press start on the Driver Station
         */
        waitForStart();

        DetectMode mode = DetectMode.SkyStone;

        while (opModeIsActive()) {
            telemetry.addData("Position", detector.getPosition());
            telemetry.update();
        }

        detector.deactivate();
    }
}
