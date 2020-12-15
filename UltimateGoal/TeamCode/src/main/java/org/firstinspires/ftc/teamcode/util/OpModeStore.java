package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.components.RobotComponent;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import java.util.HashMap;
import java.util.Map;

import lombok.Data;

@Data
public class OpModeStore {
    public enum RunMode { Autonomous, TeleOp }

    private LinearOpMode opMode = null;
    public RunMode runMode = RunMode.TeleOp;

    private volatile boolean resetEncoder = false;

    private Map<String, RobotComponent> componentMap = new HashMap<>();

    private volatile boolean liftOuttake = false;

    private SampleMecanumDrive drive;

    public void init() {
        resetEncoder = false;
    }

    public void destroy() {
        this.opMode = null;
    }

    @SuppressWarnings("unchecked")
    public <T extends RobotComponent> T getComponent(String name) {
        return (T) componentMap.get(name);
    }

    public <T extends RobotComponent> void addComponent(String name, T component) {
        componentMap.put(name, component);
    }
}
