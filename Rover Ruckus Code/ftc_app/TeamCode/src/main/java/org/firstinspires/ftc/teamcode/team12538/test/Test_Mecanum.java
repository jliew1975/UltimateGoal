package org.firstinspires.ftc.teamcode.team12538.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="Test_Mecanum", group="Linear Opmode")
public class Test_Mecanum extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor fleftDrive = null;
    private DcMotor frightDrive = null;
    private DcMotor bleftDrive = null;
    private DcMotor brightDrive = null;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        fleftDrive  = hardwareMap.get(DcMotor.class, "fleft_drive");
        frightDrive = hardwareMap.get(DcMotor.class, "fright_drive");
        bleftDrive  = hardwareMap.get(DcMotor.class, "bleft_drive");
        brightDrive = hardwareMap.get(DcMotor.class, "bright_drive");

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        fleftDrive.setDirection(DcMotor.Direction.FORWARD);
        frightDrive.setDirection(DcMotor.Direction.REVERSE);
        bleftDrive.setDirection(DcMotor.Direction.FORWARD);
        brightDrive.setDirection(DcMotor.Direction.REVERSE);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            double r = Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y);
            double robotAngle = Math.atan2(gamepad1.left_stick_y,gamepad1.left_stick_x)-Math.PI /4;
            double rightX = gamepad1.right_stick_x;
            final double v1 = r * Math.sin(robotAngle) - rightX;
            final double v2 = r * Math.cos(robotAngle) + rightX;
            final double v3 = r * Math.cos(robotAngle) - rightX;
            final double v4 = r * Math.sin(robotAngle) + rightX;

            fleftDrive.setPower(0.9 * v1);
            frightDrive.setPower(0.9 * v2);
            bleftDrive.setPower(0.9 * v3);
            brightDrive.setPower(0.9 * v4);
        }
    }
}