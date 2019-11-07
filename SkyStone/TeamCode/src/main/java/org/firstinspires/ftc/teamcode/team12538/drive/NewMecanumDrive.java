package org.firstinspires.ftc.teamcode.team12538.drive;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.team12538.components.AutoGamepad;
import org.firstinspires.ftc.teamcode.team12538.utils.OpModeUtils;

public class NewMecanumDrive extends MecanumDrive implements AutoDrive {
    // Declare information about robot
    private static final double TRACKLENGTH = 6;
    private static final double TRACKBASE = 7.645;
    private static final double WHEELRADIUS = 1.969;

    private double rearWheelFactor = 0.70;

    @Override
    public void navigateWithGamepad(Gamepad gamepad) {

        // Set desired velocities
        double xVelocity = gamepad.left_stick_y;
        double yVelocity = gamepad.left_stick_x;
        double angularVelocity = gamepad.right_stick_x;

        // Convert desired velocities into wheel velocities
        double leftFrontPower = (xVelocity - yVelocity - (TRACKBASE + TRACKLENGTH) * (angularVelocity)) / WHEELRADIUS;
        double leftRearPower = (xVelocity + yVelocity - (TRACKBASE + TRACKLENGTH) * (angularVelocity)) / WHEELRADIUS;
        double rightRearPower = (xVelocity - yVelocity + (TRACKBASE + TRACKLENGTH) * (angularVelocity)) / WHEELRADIUS;
        double rightFrontPower = (xVelocity + yVelocity + (TRACKBASE + TRACKLENGTH) * (angularVelocity)) / WHEELRADIUS;

        // Calculate wheel with max power
        double maxLeftPower = Math.max(Math.abs(leftFrontPower), Math.abs(leftRearPower));
        double maxRightPower = Math.max(Math.abs(rightFrontPower), Math.abs(rightRearPower));
        double maxPower = Math.max(maxLeftPower, maxRightPower);

        // Calculate factor to scale all wheel powers to make less than 1
        double scaleFactor = 1 / maxPower;

        if(OpModeUtils.getGlobalStore().isFoundationClawDown()) {
            scaleFactor *= 0.3 ;
        }

        if(gamepad.left_trigger > 0) {
            scaleFactor *= 0.5;
        }

        // Set motors to correct powers
        /*
        leftFront.setPower(leftFrontPower * scaleFactor);
        leftRear.setPower(leftRearPower * scaleFactor);
        rightRear.setPower(rightRearPower * scaleFactor);
        rightFront.setPower(rightFrontPower * scaleFactor);
        */

        // Due to uneven weight distribution on the robot,
        // power for the rear wheel motors are multiplied by the rearWheelFactor to slow it down
        // so it can strafe correctly.
        leftFront.setPower(Math.abs(leftFrontPower * scaleFactor) <= 0.2 ? 0d : leftFrontPower * scaleFactor);
        leftRear.setPower(Math.abs(leftRearPower * scaleFactor * rearWheelFactor) <= 0.2 ? 0d : leftRearPower * scaleFactor * rearWheelFactor);
        rightRear.setPower(Math.abs(rightRearPower * scaleFactor * rearWheelFactor) <= 0.2 ? 0d : rightRearPower * scaleFactor * rearWheelFactor);
        rightFront.setPower(Math.abs(rightFrontPower * scaleFactor) <= 0.2 ? 0d : rightFrontPower * scaleFactor);
    }


    @Override
    public void stop() {
        for(DcMotor motor : driveMotorList) {
            motor.setPower(0d);
        }
    }

    @Override
    public void autoNavigateWithGamepad(AutoGamepad gamepad) {
        // Set desired velocities
        double xVelocity = gamepad.left_stick_y;
        double yVelocity = gamepad.left_stick_x;
        double angularVelocity = gamepad.right_stick_x;

        // Convert desired velocities into wheel velocities
        double leftFrontPower = (xVelocity - yVelocity - (TRACKBASE + TRACKLENGTH) * (angularVelocity)) / WHEELRADIUS;
        double leftRearPower = (xVelocity + yVelocity - (TRACKBASE + TRACKLENGTH) * (angularVelocity)) / WHEELRADIUS;
        double rightRearPower = (xVelocity - yVelocity + (TRACKBASE + TRACKLENGTH) * (angularVelocity)) / WHEELRADIUS;
        double rightFrontPower = (xVelocity + yVelocity + (TRACKBASE + TRACKLENGTH) * (angularVelocity)) / WHEELRADIUS;

        // Calculate wheel with max power
        double maxLeftPower = Math.max(Math.abs(leftFrontPower), Math.abs(leftRearPower));
        double maxRightPower = Math.max(Math.abs(rightFrontPower), Math.abs(rightRearPower));
        double maxPower = Math.max(maxLeftPower, maxRightPower);

        // Calculate factor to scale all wheel powers to make less than 1
        double scaleFactor = gamepad.power;

        // Calculate the destination encoder tick values for the given distance
        int[] targetPositions = new int[gamepad.isStrafing() ? strafeEncoderMotors.size() : directionalEncoderMotors.size()];
        if(gamepad.isStrafing()) {
            if(gamepad.left_stick_x < 0) {
                targetPositions[0] = strafeEncoderMotors.get(0).getCurrentPosition() + (int) (gamepad.distanceInInches * DEAD_WHEEL_ENCODER_TICKS_PER_INCH);
            } else {
                targetPositions[0] = strafeEncoderMotors.get(0).getCurrentPosition() - (int) (gamepad.distanceInInches * DEAD_WHEEL_ENCODER_TICKS_PER_INCH);
            }
        } else {
            if(gamepad.left_stick_y < 0) {
                targetPositions[0] = directionalEncoderMotors.get(0).getCurrentPosition() - (int) (gamepad.distanceInInches * DEAD_WHEEL_ENCODER_TICKS_PER_INCH);
                targetPositions[1] = directionalEncoderMotors.get(1).getCurrentPosition() - (int) (gamepad.distanceInInches * DEAD_WHEEL_ENCODER_TICKS_PER_INCH);
            } else {
                targetPositions[0] = directionalEncoderMotors.get(0).getCurrentPosition() + (int) (gamepad.distanceInInches * DEAD_WHEEL_ENCODER_TICKS_PER_INCH);
                targetPositions[1] = directionalEncoderMotors.get(1).getCurrentPosition() + (int) (gamepad.distanceInInches * DEAD_WHEEL_ENCODER_TICKS_PER_INCH);
            }
        }

        leftFront.setPower(leftFrontPower * scaleFactor);
        leftRear.setPower(leftRearPower * scaleFactor);
        rightRear.setPower(rightRearPower * scaleFactor);
        rightFront.setPower(rightFrontPower * scaleFactor);

        runtime.reset();
        while(OpModeUtils.opModeIsActive()) { //&& runtime.seconds() < gamepad.timeout) {

            if(gamepad.isStrafing()) {
                if((Math.signum(gamepad.left_stick_x) >= 0 && strafeEncoderMotors.get(0).getCurrentPosition() <= targetPositions[0]) ||
                        (Math.signum(gamepad.left_stick_x) < 0 && strafeEncoderMotors.get(0).getCurrentPosition() >= targetPositions[0]))
                {
                    break;
                }
            } else {
                if((Math.signum(gamepad.left_stick_y) >= 0 &&
                    (directionalEncoderMotors.get(0).getCurrentPosition() >= targetPositions[0] ||
                            directionalEncoderMotors.get(1).getCurrentPosition() >= targetPositions[1])) ||
                        (Math.signum(gamepad.left_stick_y) < 0 &&
                                (directionalEncoderMotors.get(0).getCurrentPosition() <= targetPositions[0] ||
                                        directionalEncoderMotors.get(1).getCurrentPosition() <= targetPositions[1]))) {
                    break;
                }
            }


            // report target and current positions to driver station
            if(gamepad.isStrafing()) {
                // report target and current positions to driver station
                telemetry.addData("Target", "Running to %7d",
                        targetPositions[0]);

                telemetry.addData("Current", "Running at %7d",
                        strafeEncoderMotors.get(0).getCurrentPosition());

                telemetry.addData( "Power", "Running at %.1f",
                        strafeEncoderMotors.get(0).getPower());
            } else {
                telemetry.addData("Target", "Running to %7d : %7d",
                        targetPositions[0], targetPositions[1]);

                telemetry.addData("Current", "Running at %7d : %7d",
                        directionalEncoderMotors.get(0).getCurrentPosition(),
                        directionalEncoderMotors.get(1).getCurrentPosition());

                telemetry.addData("Power", "Running at %.1f : %.1f",
                        directionalEncoderMotors.get(0).getPower(),
                        directionalEncoderMotors.get(1).getPower());
            }

            telemetry.update();
        }

        stop();
        gamepad.reset();
    }

    @Override
    public void printTelemetry() {
        telemetry.addData("leftRear", leftRear.getCurrentPosition());
    }
}
