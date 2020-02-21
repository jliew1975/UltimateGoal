package org.firstinspires.ftc.teamcode.team12538.components;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.team12538.utils.OpModeUtils;
import org.firstinspires.ftc.teamcode.team12538.utils.states.ToggleBoolean;

public class RobotStoneAligner implements RobotComponent, ControlAware {
    public static final double INTAKE = 0;
    public static final double ALIGN = 1d;

    private Servo stoneAligner;

    private boolean isLiftSlides = true;
    private ToggleBoolean servoToggle = new ToggleBoolean(false);

    @Override
    public void init() {
        HardwareMap hardwareMap = OpModeUtils.getHardwareMap();

        stoneAligner =  hardwareMap.get(Servo.class, "intakeAligner");
        stoneAligner.setPosition(INTAKE);
    }

    public double getPosition() {
        return stoneAligner.getPosition();
    }

    public void setPosition(double position) {
        stoneAligner.setPosition(position);
    }

    @Override
    public void control(Gamepad gamepad) {
        if(gamepad.right_stick_button) {
            setPosition(ALIGN);
        } else {
            setPosition(INTAKE);
        }
    }
}
