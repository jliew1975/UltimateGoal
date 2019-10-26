package org.firstinspires.ftc.teamcode.team12538.test;

import com.disnodeteam.dogecv.detectors.skystone.SkystoneDetector;
import com.disnodeteam.dogecv.detectors.skystone.StoneDetector;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Point;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvWebcam;

@TeleOp(name = "DogeCV WebCam Test", group = "Test")
public class DogeCVWebCamTest extends LinearOpMode {
    enum DetectMode { Stone, SkyStone }

    @Override
    public void runOpMode() throws InterruptedException {
        /*
         * Instantiate an OpenCvCamera object for the camera we'll be using.
         * In this sample, we're using the phone's internal camera. We pass it a
         * CameraDirection enum indicating whether to use the front or back facing
         * camera, as well as the view that we wish to use for camera monitor (on
         * the RC phone). If no camera monitor is desired, use the alternate
         * single-parameter constructor instead (commented out below)
         */
        int webCameraMonitorViewId =
            hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        OpenCvCamera webcam =
                new OpenCvWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), webCameraMonitorViewId);

        /*
         * Open the connection to the camera device
         */
        webcam.openCameraDevice();

        /*
         * Specify the image processing pipeline we wish to invoke upon receipt
         * of a frame from the camera. Note that switching pipelines on-the-fly
         * (while a streaming session is in flight) *IS* supported.
         */
        StoneDetector stoneDetector = new StoneDetector();
        SkystoneDetector skystoneDetector = new SkystoneDetector();

        webcam.setPipeline(skystoneDetector);

        /*
         * Tell the camera to start streaming images to us! Note that you must make sure
         * the resolution you specify is supported by the camera. If it is not, an exception
         * will be thrown.
         *
         * Also, we specify the rotation that the camera is used in. This is so that the image
         * from the camera sensor can be rotated such that it is always displayed with the image upright.
         * For a front facing camera, rotation is defined assuming the user is looking at the screen.
         * For a rear facing camera or a webcam, rotation is defined assuming the camera is facing
         * away from the user.
         */
        webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);

        /*
         * Wait for the user to press start on the Driver Station
         */
        waitForStart();

        DetectMode mode = DetectMode.SkyStone;

        while (opModeIsActive()) {
            if(gamepad1.a && mode != DetectMode.Stone) {
                mode = DetectMode.Stone;
                webcam.setPipeline(stoneDetector);
            } else if(gamepad1.b && mode != DetectMode.SkyStone) {
                mode = DetectMode.SkyStone;
                webcam.setPipeline(skystoneDetector);
            }

            /*
             * Send some stats to the telemetry
             */
            telemetry.addData("Frame Count", webcam.getFrameCount());
            telemetry.addData("FPS", String.format("%.2f", webcam.getFps()));
            telemetry.addData("Total frame time ms", webcam.getTotalFrameTimeMs());
            telemetry.addData("Pipeline time ms", webcam.getPipelineTimeMs());
            telemetry.addData("Overhead time ms", webcam.getOverheadTimeMs());
            telemetry.addData("Theoretical max FPS", webcam.getCurrentPipelineMaxFps());

            if(mode == DetectMode.Stone && stoneDetector.foundScreenPositions().size() != 0d) {
                for(Point point : stoneDetector.foundScreenPositions()) {
                    printDetectorTelemetry(mode, point);
                }
            } else if(mode == DetectMode.SkyStone && skystoneDetector.foundRectangle().area() != 0d) {
                printDetectorTelemetry(mode, skystoneDetector.getScreenPosition());
            }

            telemetry.update();


            sleep(100);
        }

        webcam.closeCameraDevice();
    }

    private void printDetectorTelemetry(DetectMode mode, Point point) {
        telemetry.addData(String.format("%s found", mode), true);
        telemetry.addData("Skystone Point", point);
    }
}
