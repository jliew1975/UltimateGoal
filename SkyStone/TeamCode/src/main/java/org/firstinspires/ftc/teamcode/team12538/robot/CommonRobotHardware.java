package org.firstinspires.ftc.teamcode.team12538.robot;

import android.graphics.Path;

import org.firstinspires.ftc.teamcode.team12538.components.RobotCapstone;
import org.firstinspires.ftc.teamcode.team12538.components.RobotColorProximitySensor;
import org.firstinspires.ftc.teamcode.team12538.components.RobotDistanceSensor;
import org.firstinspires.ftc.teamcode.team12538.components.RobotFoundationClaw;
import org.firstinspires.ftc.teamcode.team12538.components.RobotOuttake;
import org.firstinspires.ftc.teamcode.team12538.components.RobotOuttakeSlides;
import org.firstinspires.ftc.teamcode.team12538.components.RobotIntake;
import org.firstinspires.ftc.teamcode.team12538.components.RobotOuttakeSlidesSensor;
import org.firstinspires.ftc.teamcode.team12538.components.RobotParkingServo;
import org.firstinspires.ftc.teamcode.team12538.components.RobotStoneAligner;
import org.firstinspires.ftc.teamcode.team12538.components.RobotStoneArm;
import org.firstinspires.ftc.teamcode.team12538.utils.OpModeUtils;

/**
 * Define all robot components this class for easy maintenance
 */
public abstract class CommonRobotHardware {
    public RobotIntake intake = new RobotIntake();
    public RobotFoundationClaw foundationClaw = new RobotFoundationClaw();
    public RobotOuttake outtake = new RobotOuttake();
    public RobotCapstone capstone = new RobotCapstone();
    public RobotParkingServo parkingServo = new RobotParkingServo();

    public RobotStoneAligner stoneAligner = new RobotStoneAligner();
    public RobotColorProximitySensor intakeSensor = new RobotColorProximitySensor();
    public RobotDistanceSensor leftDistSensor = new RobotDistanceSensor("left",0.040, 0.52);
    public RobotDistanceSensor rightDistSensor = new RobotDistanceSensor("right",0.821, 0.321);

    public void init() {
        intake.init();
        foundationClaw.init();
        outtake.init();
        leftDistSensor.init();
        rightDistSensor.init();
        intakeSensor.init();
        capstone.init();
        stoneAligner.init();
        parkingServo.init();

        OpModeUtils.getGlobalStore().addComponent("outtake", outtake);
    }
}
