package org.firstinspires.ftc.teamcode.team12538.detectors.dogecv;

import com.disnodeteam.dogecv.detectors.skystone.SkystoneDetector;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.team12538.detectors.VisionDetector;
import org.firstinspires.ftc.teamcode.team12538.utils.OpModeUtils;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

public class DogeCvDetector implements VisionDetector {
    enum DetectMode { Stone, SkyStone }

    private boolean targetVisible = false;
    private boolean targetAligned = false;

    private double targetPosition = 0d;

    public boolean isTargetVisible() {
        return targetVisible;
    }

    public double getTargetPosition() {
        return targetPosition;
    }

    private OpenCvCamera webcam = null;
    SkystoneDetector skystoneDetector = new SkystoneDetector();

    public void init() {
        HardwareMap hardwareMap = OpModeUtils.getHardwareMap();
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

        webcam = new OpenCvWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), webCameraMonitorViewId);

        /*
         * Open the connection to the camera device
         */
        webcam.openCameraDevice();
        webcam.setPipeline(skystoneDetector);
    }

    public void activate() {
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

        while(!OpModeUtils.getOpMode().isStopRequested()) {
            if(skystoneDetector.foundRectangle().area() != 0d) {
                targetVisible = true;
                targetAligned = skystoneDetector.getScreenPosition().x >= 20;
                targetPosition = skystoneDetector.getScreenPosition().x;
                OpModeUtils.getTelemetry().addData("Skystone", "Found");
                OpModeUtils.getTelemetry().addData("Skystone Position", targetPosition);
            } else {
                targetVisible = false;
                targetAligned = false;
                OpModeUtils.getTelemetry().addData("Target", "Not Found");
            }

            OpModeUtils.getTelemetry().update();
        }
    }

    public void deactivate() {
        webcam.closeCameraDevice();
    }

    @Override
    public boolean isAligned() {
        return targetAligned;
    }
}
