package org.firstinspires.ftc.teamcode.team12538.components;

import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.team12538.utils.OpModeUtils;
import org.firstinspires.ftc.teamcode.team12538.utils.ThreadUtils;
import org.firstinspires.ftc.teamcode.team12538.utils.states.Button;

public class RobotOuttakeSlidesSensor implements RobotComponent, TelemetryAware {
    private DigitalChannel magneticSensor;

    @Override
    public void init() {
        magneticSensor = OpModeUtils.getHardwareMap().get(DigitalChannel.class, "slideSensor");
    }

    @Override
    public void printTelemetry() {
        Telemetry telemetry = OpModeUtils.getTelemetry();
        telemetry.addData("Slide Down", !magneticSensor.getState());
    }

    public boolean getState() {
        return magneticSensor.getState();
    }
}
