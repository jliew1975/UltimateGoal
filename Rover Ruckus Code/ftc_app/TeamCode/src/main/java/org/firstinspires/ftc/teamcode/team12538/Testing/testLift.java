/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.team12538.Testing;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.team12538.utils.MotorUtils;


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

@TeleOp(name="testBot", group="Linear Opmode")

public class testLift extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor scissorJack = null;
    private DcMotor verticalLift = null;
    private DcMotor intake = null;
    private AnalogInput deadwheel = null;

    private CRServo hook = null;


    private Servo deposit = null;

    private DcMotor frontLeftDrive = null;
    private DcMotor frontRightDrive = null;
    private DcMotor rearLeftDrive = null;
    private DcMotor rearRightDrive = null;
    private DcMotor intakeFlip = null;
    private DcMotor linearSlides = null;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery

        /*scissorJack = hardwareMap.get(DcMotor.class, "jack");
        MotorUtils.setZeroPowerMode(DcMotor.ZeroPowerBehavior.BRAKE, scissorJack);
        MotorUtils.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER, scissorJack);
        MotorUtils.setMode(DcMotor.RunMode.RUN_USING_ENCODER, scissorJack);

        intake = hardwareMap.get(DcMotor.class, "intake");
        deposit = hardwareMap.get(Servo.class, "depo");
        */
        deadwheel = hardwareMap.get(AnalogInput.class, "dead_wheel");
/*
        hook = hardwareMap.get(CRServo.class, "latch");



        frontLeftDrive = hardwareMap.get(DcMotor.class, "left_forward_drive");
        frontRightDrive = hardwareMap.get(DcMotor.class, "right_forward_drive");
        rearLeftDrive = hardwareMap.get(DcMotor.class, "left_back_drive");
        rearRightDrive = hardwareMap.get(DcMotor.class, "right_back_drive");

        intakeFlip = hardwareMap.get(DcMotor.class, "intake_flip");
        MotorUtils.setZeroPowerMode(DcMotor.ZeroPowerBehavior.BRAKE, intakeFlip);
        MotorUtils.setMode(DcMotor.RunMode.RUN_USING_ENCODER, intakeFlip);
*/

      /*  verticalLift = hardwareMap.get(DcMotor.class, "vertical_slides");
        MotorUtils.setZeroPowerMode(DcMotor.ZeroPowerBehavior.BRAKE, verticalLift);
        deposit.setPosition(0.9);*/
/*
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        rearLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        rearRightDrive.setDirection(DcMotor.Direction.FORWARD);


        linearSlides = hardwareMap.get(DcMotor.class, "linear_slides");
*/


        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        //deadwheel.resetDeviceConfigurationForOpMode();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            telemetry.addData("Dead Wheel Encoder Value", deadwheel.getVoltage());
            telemetry.addData("Encoder Max Voltage", deadwheel.getMaxVoltage());
            telemetry.addData("DeadWheel Rotations", loopBack.getRotation(deadwheel.getVoltage()));
            telemetry.update();
            /*
           //scissor lift code
           if(gamepad1.dpad_up){
               scissorJack.setPower(-1);
           }
           else if(gamepad1.dpad_down){
               scissorJack.setPower(1);
           }
           else{
               scissorJack.setPower(0);
           }
/*
           //depositing code
            if(gamepad1.a){
               deposit.setPosition(0.68);
            }
            else if(gamepad1.x){
               deposit.setPosition(0.22);
            }


           //intake flip code
            if(gamepad2.dpad_up){
                intakeFlip.setPower(-0.9);
            }
            else if(gamepad2.dpad_down){
                intakeFlip.setPower(0.9);
            }
            else{
                intakeFlip.setPower(0);
            }
*/
            /*
           //intake code
           if(gamepad2.right_bumper){
               intake.setPower(1);
           }
           else if(gamepad2.left_bumper){
               intake.setPower(-1);
           }
           else{
               intake.setPower(0);
           }

           //linear slide horizontal extension
            if(gamepad2.dpad_left){
                linearSlides.setPower(-1);
            }
            else if(gamepad2.dpad_right){
                linearSlides.setPower(1);
            }
            else{
                linearSlides.setPower(0);
            }*/

/*
           //hook code
           if(gamepad1.dpad_right){
                hook.setPower(1);
            }
            else if(gamepad1.dpad_left){
                hook.setPower(-1);
            }
            else{
                hook.setPower(0);
            }
*/
            //vertical lift code
            //verticalLift.setPower(gamepad2.left_stick_y);

            /*
            //wheel code
            double r = Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y);
            double robotAngle = Math.atan2(gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI / 4;
            double rightX = gamepad1.right_stick_x;

            final double v1 = r * Math.sin(robotAngle) - rightX;
            final double v2 = r * Math.cos(robotAngle) + rightX;
            final double v3 = r * Math.cos(robotAngle) - rightX;
            final double v4 = r * Math.sin(robotAngle) + rightX;

            double power = 1.0;
            if(gamepad1.left_trigger > 0 || gamepad1.right_trigger > 0) {
                power = 0.3; // slowdown robot on left or right trigger
            }

            frontLeftDrive.setPower(power * Math.signum(v1));
            frontRightDrive.setPower(power * Math.signum(v2));
            rearLeftDrive.setPower(power * Math.signum(v3));
            rearRightDrive.setPower(power * Math.signum(v4));
           */
        }

    }
}
