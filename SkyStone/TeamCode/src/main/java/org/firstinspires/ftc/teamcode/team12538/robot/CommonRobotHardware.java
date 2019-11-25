package org.firstinspires.ftc.teamcode.team12538.robot;

import android.graphics.Path;

import org.firstinspires.ftc.teamcode.team12538.components.RobotColorProximitySensor;
import org.firstinspires.ftc.teamcode.team12538.components.RobotDistanceSensor;
import org.firstinspires.ftc.teamcode.team12538.components.RobotFoundationClaw;
import org.firstinspires.ftc.teamcode.team12538.components.RobotOuttake;
import org.firstinspires.ftc.teamcode.team12538.components.RobotOuttakeSlides;
import org.firstinspires.ftc.teamcode.team12538.components.RobotIntake;
import org.firstinspires.ftc.teamcode.team12538.components.RobotStoneArm;
import org.firstinspires.ftc.teamcode.team12538.utils.OpModeUtils;

/**
 * Define all robot components this class for easy maintenance
 */
public abstract class CommonRobotHardware {
    public RobotIntake intake = new RobotIntake();
    public RobotOuttakeSlides outtakeSlides = new RobotOuttakeSlides();
    public RobotFoundationClaw foundationClaw = new RobotFoundationClaw();
    public RobotOuttake outtake = new RobotOuttake();
    public RobotStoneArm stoneArm = new RobotStoneArm();

    public RobotColorProximitySensor intakeSensor = new RobotColorProximitySensor();
    public RobotDistanceSensor leftDistSensor = new RobotDistanceSensor("left",0.040, 0.52);
    public RobotDistanceSensor rightDistSensor = new RobotDistanceSensor("right",0.821, 0.321);

    public void init() {
        intake.init();
        outtakeSlides.init();
        foundationClaw.init();
        outtake.init();
        stoneArm.init();

        leftDistSensor.init();
        rightDistSensor.init();
        intakeSensor.init();

        OpModeUtils.getGlobalStore().addComponent("foundationClaw", foundationClaw);
    }
}
