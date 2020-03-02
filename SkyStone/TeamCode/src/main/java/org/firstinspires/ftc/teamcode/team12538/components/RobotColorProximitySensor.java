package org.firstinspires.ftc.teamcode.team12538.components;

import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.team12538.detectors.RobotDetector;
import org.firstinspires.ftc.teamcode.team12538.utils.OpModeUtils;
import org.firstinspires.ftc.teamcode.team12538.utils.ThreadUtils;

public class RobotColorProximitySensor implements RobotDetector, RobotComponent {
    public DistanceSensor sensorDistance;

    private boolean isDetected = false;
    private volatile boolean isStopRequested = false;

    private DistanceUnit unit = DistanceUnit.INCH;

    public void init() {
        HardwareMap hardwareMap = OpModeUtils.getHardwareMap();
        sensorDistance = hardwareMap.get(DistanceSensor.class, "sensor_color_distance");
    }

    public void start() {
        ThreadUtils.getExecutorService().submit(() -> {
            while(!isStopRequested) {
                if(sensorDistance.getDistance(unit) != Double.NaN) {
                    if(sensorDistance.getDistance(unit) < 2.2d) {
                        ThreadUtils.sleep(500);
                        if(sensorDistance.getDistance(unit) < 2.2d) {
                            isDetected = true;
                        }
                    } else {
                        isDetected = false;
                    }
                }
            }
        });
    }

    public double getSensorValue() {
        if(sensorDistance.getDistance(unit) == Double.NaN)
            return  -1d;

        return sensorDistance.getDistance(unit);
    }

    public boolean hasStoneButNotCompletelyIn() {
        if(sensorDistance.getDistance(unit) != Double.NaN) {
            return getSensorValue() > 2.2d;
        }

        return false;
    }

    public void stop() {
        isStopRequested = true;
    }

    public boolean isDetected() {
        return isDetected;
    }
}
