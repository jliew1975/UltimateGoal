package org.firstinspires.ftc.teamcode.detectors;

import com.arcrobotics.ftclib.vision.UGRectDetector;
import com.arcrobotics.ftclib.vision.UGRectRingPipeline;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.detectors.enums.RingCount;
import org.firstinspires.ftc.teamcode.util.OpModeUtils;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

import lombok.NoArgsConstructor;

@NoArgsConstructor
public class FtcLibRingsDetector implements Detector, Runnable {
    private OpenCvCamera camera;
    private boolean isUsingWebcam = false;
    private HardwareMap hardwareMap;
    private UGRectRingPipeline ftclibPipeline;

    private RingCount ringCount = RingCount.ZERO;

    private volatile boolean isActivated = false;
    private ExecutorService executorService = Executors.newSingleThreadExecutor();

    public FtcLibRingsDetector(boolean isUsingWebcam) {
        this.isUsingWebcam = isUsingWebcam;
    }

    @Override
    public void init() {
       this.hardwareMap = OpModeUtils.getHardwareMap();
        if (isUsingWebcam) {
            int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
            camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        } else {
            int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
            camera = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        }

        // Set the pipeline the camera should use and start streaming
        camera.setPipeline(ftclibPipeline = new UGRectRingPipeline());
    }

    @Override
    public void activate() {
        isActivated = true;
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }
        });

        executorService.submit(this);
    }

    @Override
    public void deactivate() {
        this.isActivated = false;
        camera.closeCameraDeviceAsync(() -> {

        });

        executorService.shutdownNow();
    }

    @Override
    public RingCount getRingCount() {
        return ringCount;
    }

    public void run() {
        while(OpModeUtils.opModeIsActive() && isActivated) {
            if (Math.abs(ftclibPipeline.getTopAverage() - ftclibPipeline.getBottomAverage()) < ftclibPipeline.getThreshold() && (ftclibPipeline.getTopAverage() <= 100 && ftclibPipeline.getBottomAverage() <= 100)) {
                this.ringCount = RingCount.FOUR;
            } else if (Math.abs(ftclibPipeline.getTopAverage() - ftclibPipeline.getBottomAverage()) < ftclibPipeline.getThreshold() && (ftclibPipeline.getTopAverage() >= 100 && ftclibPipeline.getBottomAverage() >= 100)) {
                this.ringCount = RingCount.ZERO;
            } else {
                this.ringCount = RingCount.ONE;
            }

            Telemetry telemetry = OpModeUtils.getTelemetry();
            telemetry.addData("topAverage", this::getTopAverage);
            telemetry.addData("bottomAverage", this::getBottomAverage);
            telemetry.addData("RingCount", this::getRingCount);
            telemetry.update();
        }
    }

    public void setTopRectangle(double topRectHeightPercentage, double topRectWidthPercentage) {
        ftclibPipeline.setTopRectHeightPercentage(topRectHeightPercentage);
        ftclibPipeline.setTopRectWidthPercentage(topRectWidthPercentage);
    }

    public void setBottomRectangle(double bottomRectHeightPercentage, double bottomRectWidthPercentage) {
        ftclibPipeline.setBottomRectHeightPercentage(bottomRectHeightPercentage);
        ftclibPipeline.setBottomRectWidthPercentage(bottomRectWidthPercentage);
    }

    public void setRectangleSize(int rectangleWidth, int rectangleHeight){
        ftclibPipeline.setRectangleHeight(rectangleHeight);
        ftclibPipeline.setRectangleWidth(rectangleWidth);
    }

    public void setThreshold(int threshold) {
        ftclibPipeline.setThreshold(threshold);
    }

    public double getTopAverage() {
        return ftclibPipeline.getTopAverage();
    }

    public double getBottomAverage() {
        return ftclibPipeline.getBottomAverage();
    }
}
