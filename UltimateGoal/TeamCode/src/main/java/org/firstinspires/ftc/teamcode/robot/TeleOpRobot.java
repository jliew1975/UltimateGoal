package org.firstinspires.ftc.teamcode.robot;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.buttons.Button;
import org.firstinspires.ftc.teamcode.components.CommonComponents;
import org.firstinspires.ftc.teamcode.components.Shooter;
import org.firstinspires.ftc.teamcode.components.WobbleArm;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.teleop.MecanumDrive;
import org.firstinspires.ftc.teamcode.util.DashboardUtil;
import org.firstinspires.ftc.teamcode.util.OpModeUtils;
import org.firstinspires.ftc.teamcode.util.PoseStorage;

public class TeleOpRobot extends CommonComponents {
    public static double DRAWING_TARGET_RADIUS = 2;

    // Define 2 states, driver control or alignment control
    enum Mode {
        NORMAL_CONTROL,
        ALIGN_TO_POINT
    }

    /**
     * Old MecanumDrive
     */
     // private MecanumDrive drive = new MecanumDrive();

    /**
     * Road runner drive in TeleOp
     */
    private SampleMecanumDrive drive;

    private Mode currentMode = Mode.NORMAL_CONTROL;

    // Declare a target vector you'd like your bot to align with
    // Can be any x/y coordinate of your choosing
    private Vector2d targetPosition = new Vector2d(0, 0);

    // Declare a PIDF Controller to regulate heading
    // Use the same gains as SampleMecanumDrive's heading controller
    private PIDFController headingController = new PIDFController(SampleMecanumDrive.HEADING_PID);

    private Telemetry telemetry;

    private Button leftBumper = new Button();

    public void init() {
        super.init();
        // drive.init();

        drive = new SampleMecanumDrive(OpModeUtils.getHardwareMap());

        // Retrieve out pose from the PoseStorage.currentPose static field
        // drive.setPoseEstimate(PostStorage.currentPose);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Retrieve our pose from the PoseStorage.currentPose static field
        // See AutoTransferPose.java for further details
        drive.getLocalizer().setPoseEstimate(PoseStorage.currentPose);

        // Set input bounds for the heading controller
        // Automatically handles overflow
        headingController.setInputBounds(-Math.PI, Math.PI);

        // Get a reference to the telemetry object
        telemetry = OpModeUtils.getTelemetry();
    }

    /**
     * Control for player 1
     * @param gamepad
     */
    public void controlA(Gamepad gamepad) {
        // drive.navigateWithGamepad(gamepad);

        // Read pose
        Pose2d poseEstimate = drive.getLocalizer().getPoseEstimate();

        // Declare a drive direction
        // Pose representing desired x, y, and angular velocity
        Pose2d driveDirection = new Pose2d();

        telemetry.addData("mode", currentMode);

        // Declare telemetry packet for dashboard field drawing
        TelemetryPacket packet = new TelemetryPacket();
        Canvas fieldOverlay = packet.fieldOverlay();

        leftBumper.input(gamepad.left_bumper);

        switch (currentMode) {
            case NORMAL_CONTROL:
                // Switch into alignment mode if `a` is pressed
                if (leftBumper.onPress()) {
                    currentMode = Mode.ALIGN_TO_POINT;
                }

                // Standard teleop control
                // Convert gamepad input into desired pose velocity
                driveDirection = new Pose2d(
                        -gamepad.left_stick_y,
                        -gamepad.left_stick_x,
                        -gamepad.right_stick_x
                );
                break;
            case ALIGN_TO_POINT:
                // Switch back into normal driver control mode if `b` is pressed
                if (leftBumper.onPress()) {
                    currentMode = Mode.NORMAL_CONTROL;
                }

                // Create a vector from the gamepad x/y inputs which is the field relative movement
                // Then, rotate that vector by the inverse of that heading for field centric control
                Vector2d fieldFrameInput = new Vector2d(
                        -gamepad.left_stick_y,
                        -gamepad.left_stick_x
                );
                Vector2d robotFrameInput = fieldFrameInput.rotated(-poseEstimate.getHeading());

                // Difference between the target vector and the bot's position
                Vector2d difference = targetPosition.minus(poseEstimate.vec());
                // Obtain the target angle for feedback and derivative for feedforward
                double theta = difference.angle();

                // Not technically omega because its power. This is the derivative of atan2
                double thetaFF = -fieldFrameInput.rotated(-Math.PI / 2).dot(difference) / (difference.norm() * difference.norm());

                // Set the target heading for the heading controller to our desired angle
                headingController.setTargetPosition(theta);

                // Set desired angular velocity to the heading controller output + angular
                // velocity feedforward
                double headingInput = (headingController.update(poseEstimate.getHeading())
                        * DriveConstants.kV + thetaFF)
                        * DriveConstants.TRACK_WIDTH;

                // Combine the field centric x/y velocity with our derived angular velocity
                driveDirection = new Pose2d(
                        robotFrameInput,
                        headingInput
                );

                // Draw the target on the field
                fieldOverlay.setStroke("#dd2c00");
                fieldOverlay.strokeCircle(targetPosition.getX(), targetPosition.getY(), DRAWING_TARGET_RADIUS);

                // Draw lines to target
                fieldOverlay.setStroke("#b89eff");
                fieldOverlay.strokeLine(targetPosition.getX(), targetPosition.getY(), poseEstimate.getX(), poseEstimate.getY());
                fieldOverlay.setStroke("#ffce7a");
                fieldOverlay.strokeLine(targetPosition.getX(), targetPosition.getY(), targetPosition.getX(), poseEstimate.getY());
                fieldOverlay.strokeLine(targetPosition.getX(), poseEstimate.getY(), poseEstimate.getX(), poseEstimate.getY());
                break;
        }

        // Draw bot on canvas
        fieldOverlay.setStroke("#3F51B5");
        DashboardUtil.drawRobot(fieldOverlay, poseEstimate);

        drive.setWeightedDrivePower(driveDirection);

        // Update the heading controller with our current heading
        headingController.update(poseEstimate.getHeading());

        // Update he localizer
        drive.getLocalizer().update();

        // Send telemetry packet off to dashboard
        FtcDashboard.getInstance().sendTelemetryPacket(packet);

        // Print pose to telemetry
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
        super.get(WobbleArm.class).control(gamepad);
    }
}
