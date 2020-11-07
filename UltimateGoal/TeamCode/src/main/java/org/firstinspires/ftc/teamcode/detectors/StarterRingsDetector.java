package org.firstinspires.ftc.teamcode.detectors;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.OpModeUtils;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;

public class StarterRingsDetector {
    /*
     * An enum to define the skystone position
     */
    public enum RingCount
    {
        FOUR,
        ONE,
        NONE
    }

    private OpenCvCamera robotCam;
    private RingsDeterminationPipeline pipeline;

    private int avg1;

    // Volatile since accessed by OpMode thread w/o synchronization
    private volatile RingCount ringCount = RingCount.FOUR;

    private Telemetry telemetry;

    public void init() throws InterruptedException {
        HardwareMap hardwareMap = OpModeUtils.getHardwareMap();
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        // Get the reference to the telemetry object
        telemetry = OpModeUtils.getTelemetry();

        // initialized the pipeline object
        pipeline = new RingsDeterminationPipeline();

        // Uncomment below for web cam
        // robotCam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        // Uncomment below for phone cam
        robotCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);

        robotCam.setPipeline(pipeline); // different stages
    }

    public void activate() {
        robotCam.openCameraDeviceAsync(() -> {
            robotCam.startStreaming(320, 240, OpenCvCameraRotation.SIDEWAYS_LEFT);
        });
    }

    public void deactivate() {
        robotCam.closeCameraDeviceAsync(() -> {
            // purposely left blank
        });
    }

    public int getAnalysis() {
        return avg1;
    }

    public RingCount getRingCount() {
        return ringCount;
    }

    class RingsDeterminationPipeline extends OpenCvPipeline
    {
        /*
         * Some color constants
         */
        final Scalar RECT = new Scalar(0, 255, 0);

        /*
         * The core values which define the location and size of the sample regions
         */
        final Point REGION1_TOPLEFT_ANCHOR_POINT = new Point(100,98);

        static final int REGION_WIDTH = 35;
        static final int REGION_HEIGHT = 25;

        final int FOUR_RING_THRESHOLD = 150;
        final int ONE_RING_THRESHOLD = 136;

        Point region1_pointA = new Point(
                REGION1_TOPLEFT_ANCHOR_POINT.x,
                REGION1_TOPLEFT_ANCHOR_POINT.y);
        Point region1_pointB = new Point(
                REGION1_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
                REGION1_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);

        /*
         * Working variables
         */
        Mat region1_Cb;
        Mat YCrCb = new Mat();
        Mat Cb = new Mat();

        /*
         * This function takes the RGB frame, converts to YCrCb,
         * and extracts the Cb channel to the 'Cb' variable
         */
        void inputToCb(Mat input)
        {
            Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
            Core.extractChannel(YCrCb, Cb, 1);
        }

        @Override
        public void init(Mat firstFrame)
        {
            inputToCb(firstFrame);

            region1_Cb = Cb.submat(new Rect(region1_pointA, region1_pointB));
        }

        @Override
        public Mat processFrame(Mat input)
        {
            inputToCb(input);

            avg1 = (int) Core.mean(region1_Cb).val[0];

            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region1_pointA, // First point which defines the rectangle
                    region1_pointB, // Second point which defines the rectangle
                    RECT, // The color the rectangle is drawn in
                    2); // Thickness of the rectangle lines

            ringCount = RingCount.FOUR; // Record our analysis
            if(avg1 > FOUR_RING_THRESHOLD) {
                ringCount = RingCount.FOUR;
            } else if (avg1 > ONE_RING_THRESHOLD) {
                ringCount = RingCount.ONE;
            } else {
                ringCount = RingCount.NONE;
            }

            telemetry.addData("Analysis", avg1);
            telemetry.addData("Ring Count", ringCount);
            telemetry.update();

            return input;
        }
    }
}
