package org.firstinspires.ftc.teamcode.team12538.components;

import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.team12538.detectors.RobotDetectorLimit;
import org.firstinspires.ftc.teamcode.team12538.utils.OpModeUtils;

public class RobotDistanceSensor implements RobotComponent, RobotDetectorLimit {
    private String prefixName;

    private double sideView;
    private double forntView;

    public double limit = 0.5d;
    public Servo distanceSensorServo;
    public DistanceSensor distanceSensor;


    public RobotDistanceSensor(String prefixName, double sideView, double frontView) {
        this.prefixName = prefixName;
        this.sideView = sideView;
        this.forntView = frontView;
    }

    @Override
    public void init() {
        HardwareMap hardwareMap = OpModeUtils.getHardwareMap();
        distanceSensor = hardwareMap.get(DistanceSensor.class, String.format("%s_distance_sensor", prefixName));
        distanceSensorServo = hardwareMap.get(Servo.class, String.format("%s_distance_sensor_servo", prefixName));
        distanceSensorServo.setPosition(forntView);
    }

    @Override
    public boolean isDetected() {
        return distanceSensor.getDistance(DistanceUnit.INCH) <= limit;
    }

    @Override
    public double getLimit() {
        return limit;
    }

    @Override
    public void setLimit(double limit) {
        this.limit = limit;
    }

    @Override
    public double getCurrentDistance() {
        return distanceSensor.getDistance(DistanceUnit.INCH);
    }

    public void frontView() {
        distanceSensorServo.setPosition(forntView);
    }

    public void sideView() {
        distanceSensorServo.setPosition(sideView);
    }
}
