package org.firstinspires.ftc.teamcode.team12538.components;

import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.team12538.detectors.RobotDetector;
import org.firstinspires.ftc.teamcode.team12538.utils.OpModeUtils;
import org.firstinspires.ftc.teamcode.team12538.utils.ThreadUtils;

public class RobotColorProximitySensor implements RobotDetector {
    public DistanceSensor sensorDistance;

    private boolean isDetected = false;
    private volatile boolean isStopRequested = false;

    public void init() {
        HardwareMap hardwareMap = OpModeUtils.getHardwareMap();
        sensorDistance = hardwareMap.get(DistanceSensor.class, "sensor_color_distance");
    }

    public void start() {
        ThreadUtils.getExecutorService().submit(() -> {
            while(!isStopRequested) {
                if(sensorDistance.getDistance(DistanceUnit.CM) != Double.NaN) {
                    if(sensorDistance.getDistance(DistanceUnit.CM) < 7d) {
                        ThreadUtils.sleep(500);
                        if(sensorDistance.getDistance(DistanceUnit.CM) < 7d) {
                            isDetected = true;
                        }
                    } else {
                        isDetected = false;
                    }
                }
            }
        });
    }

    public void stop() {
        isStopRequested = true;
    }

    public boolean isDetected() {
        return isDetected;
    }
}
