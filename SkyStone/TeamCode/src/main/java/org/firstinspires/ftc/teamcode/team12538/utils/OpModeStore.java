package org.firstinspires.ftc.teamcode.team12538.utils;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.team12538.components.RobotComponent;

import java.util.HashMap;
import java.util.Map;

import lombok.Data;

@Data
public class OpModeStore {
    public enum RunMode { Autonomous, TeleOp }

    private LinearOpMode opMode = null;
    public RunMode runMode = RunMode.TeleOp;
    public AutonomousColor autoColor = AutonomousColor.Unknown;

    private volatile boolean resetEncoder = false;
    private volatile boolean foundationClawDown = false;
    private volatile boolean depositMode = false;

    private Map<String, RobotComponent> componentMap = new HashMap<>();

    private volatile boolean liftOuttake = false;

    public void init() {
        resetEncoder = false;
        foundationClawDown = false;
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
