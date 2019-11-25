package org.firstinspires.ftc.teamcode.team12538.components;

import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.team12538.detectors.RobotDetector;
import org.firstinspires.ftc.teamcode.team12538.utils.OpModeUtils;

public class RobotColorProximitySensor implements RobotDetector {
    public DistanceSensor sensorDistance;

    public void init() {
        HardwareMap hardwareMap = OpModeUtils.getHardwareMap();
        sensorDistance = hardwareMap.get(DistanceSensor.class, "sensor_color_distance");
    }

    public boolean isDetected() {
        Double distance = sensorDistance.getDistance(DistanceUnit.CM);
        return distance != Double.NaN && distance < 6.55d;
    }

}
