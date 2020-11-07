package org.firstinspires.ftc.teamcode.robot;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.components.CommonComponents;
import org.firstinspires.ftc.teamcode.components.Shooter;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.teleop.MecanumDrive;
import org.firstinspires.ftc.teamcode.util.OpModeUtils;

public class TeleOpRobot extends CommonComponents {
    // private MecanumDrive drive = new MecanumDrive();

    /**
     * using road runner drive in TeleOp
     */
    SampleMecanumDrive drive;

    public void init() {
        super.init();
        // drive.init();

        drive = new SampleMecanumDrive(OpModeUtils.getHardwareMap());

        // Retrieve out pose from the PoseStorage.currentPose static field
        // drive.setPoseEstimate(PostStorage.currentPose);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    /**
     * Control for player 1
     * @param gamepad
     */
    public void controlA(Gamepad gamepad) {
        // drive.navigateWithGamepad(gamepad);

        drive.setWeightedDrivePower(
                new Pose2d(
                        -gamepad.left_stick_y,
                        -gamepad.left_stick_x,
                        -gamepad.right_stick_x
                )
        );

        // Update everything. Odometry. Etc.
        drive.update();

        // Read pose
        Pose2d poseEstimate = drive.getPoseEstimate();

        // Print pose to telemetry
        Telemetry telemetry = OpModeUtils.getTelemetry();
        telemetry.addData("x", poseEstimate.getX());
        telemetry.addData("y", poseEstimate.getY());
        telemetry.addData("heading", poseEstimate.getHeading());

        // robot components controls
        super.get(Shooter.class).control(gamepad);
    }

    /**
     * Control for player 2
     * @param gamepad
     */
    public void controlB(Gamepad gamepad) {

    }
}
