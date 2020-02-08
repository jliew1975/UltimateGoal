package org.firstinspires.ftc.teamcode.team12538.components;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.team12538.utils.OpModeUtils;

public class RobotParkingServo implements RobotComponent, ControlAware, TelemetryAware {
    public static final double INIT_POSITION = 0d;
    public static final double PARKING_POSITION = 1d;


    private Servo parkingServo;
    private volatile ClawMode servoClawMode;

    private Object lock = new Object();
    private volatile boolean busy = false;

    @Override
    public void init() {
        HardwareMap hardwareMap = OpModeUtils.getHardwareMap();
        parkingServo = hardwareMap.get(Servo.class, "parkingServo");
        parkingServo.setPosition(INIT_POSITION);
    }

    @Override
    public void control(Gamepad gamepad) {
        if(gamepad.right_bumper) {
            parkingMode();
        } else if(gamepad.left_bumper) {
            teleOpMode();
        }
    }

    @Override
    public void printTelemetry() {
        Telemetry telemetry = OpModeUtils.getTelemetry();
        telemetry.addData(this.getClass().getSimpleName(), parkingServo.getPosition());
    }

    public void parkingMode() {
        parkingServo.setPosition(PARKING_POSITION);
    }

    public void teleOpMode() {
        parkingServo.setPosition(INIT_POSITION);
    }
}
