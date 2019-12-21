package org.firstinspires.ftc.teamcode.team12538.drive;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.team12538.ext.AutoGamepad;
import org.firstinspires.ftc.teamcode.team12538.detectors.RobotDetectorLimit;
import org.firstinspires.ftc.teamcode.team12538.ext.PIDControllerV1;
import org.firstinspires.ftc.teamcode.team12538.utils.OpModeUtils;

public class NewMecanumDrive extends MecanumDrive implements AutoDrive {
    MotorPower motorPower = new MotorPower();

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

        if(gamepad.left_trigger > 0) {
            if(gamepad.left_stick_x != 0d) {
                scaleFactor *= 0.8;
            } else {
                scaleFactor *= 0.3;
            }
        }

        // Due to uneven weight distribution on the robot,
        // power for the rear wheel motors are multiplied by the rearWheelFactor to slow it down
        // so it can strafe correctly.
        leftFront.setPower(Math.abs(leftFrontPower * scaleFactor) <= 0.2 ? 0d : leftFrontPower * scaleFactor);
        leftRear.setPower(Math.abs(leftRearPower * scaleFactor) <= 0.2 ? 0d : leftRearPower * scaleFactor);
        rightRear.setPower(Math.abs(rightRearPower * scaleFactor) <= 0.2 ? 0d : rightRearPower * scaleFactor);
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
        double scaleFactor = (1/ maxPower) * gamepad.power;

        // Calculate the destination encoder tick values for the given distance
        int[] targetPositions = new int[gamepad.isStrafing() ? strafeEncoderMotors.size() : directionalEncoderMotors.size()];
        if(gamepad.isStrafing()) {
            if (gamepad.left_stick_x < 0) {
                targetPositions[0] = strafeEncoderMotors.get(0).getCurrentPosition() + (int) (gamepad.distanceInInches * DEAD_WHEEL_ENCODER_TICKS_PER_INCH);
            } else {
                targetPositions[0] = strafeEncoderMotors.get(0).getCurrentPosition() - (int) (gamepad.distanceInInches * DEAD_WHEEL_ENCODER_TICKS_PER_INCH);
            }
        } else if(gamepad.isCurving()) {
            if(gamepad.isCurvingRight()) {
                targetPositions[1] = directionalEncoderMotors.get(0).getCurrentPosition() - (int) (gamepad.distanceInInches * DEAD_WHEEL_ENCODER_TICKS_PER_INCH);
            } else {
                targetPositions[0] = directionalEncoderMotors.get(1).getCurrentPosition() - (int) (gamepad.distanceInInches * DEAD_WHEEL_ENCODER_TICKS_PER_INCH);
            }
        } else if(gamepad.isTurning()) {
            /*
            if(gamepad.right_stick_x < 0) {
                targetPositions[0] = directionalEncoderMotors.get(0).getCurrentPosition() + (int) (((gamepad.turnDegree / 360) * Math.PI) * (2 * TRACKBASE) * DEAD_WHEEL_ENCODER_TICKS_PER_INCH);
                targetPositions[1] = directionalEncoderMotors.get(1).getCurrentPosition() - (int) (((gamepad.turnDegree / 360) * Math.PI) * (2 * TRACKBASE) * DEAD_WHEEL_ENCODER_TICKS_PER_INCH);
            } else {
                targetPositions[0] = directionalEncoderMotors.get(0).getCurrentPosition() + (int) (((gamepad.turnDegree / 360) * Math.PI) * (2 * TRACKBASE) * DEAD_WHEEL_ENCODER_TICKS_PER_INCH);
                targetPositions[1] = directionalEncoderMotors.get(1).getCurrentPosition() - (int) (((gamepad.turnDegree / 360) * Math.PI) * (2 * TRACKBASE) * DEAD_WHEEL_ENCODER_TICKS_PER_INCH);
            }
            */
        } else {
            if(gamepad.left_stick_y < 0) {
                targetPositions[0] = directionalEncoderMotors.get(0).getCurrentPosition() - (int) (gamepad.distanceInInches * DEAD_WHEEL_ENCODER_TICKS_PER_INCH);
                targetPositions[1] = directionalEncoderMotors.get(1).getCurrentPosition() - (int) (gamepad.distanceInInches * DEAD_WHEEL_ENCODER_TICKS_PER_INCH);
            } else {
                targetPositions[0] = directionalEncoderMotors.get(0).getCurrentPosition() + (int) (gamepad.distanceInInches * DEAD_WHEEL_ENCODER_TICKS_PER_INCH);
                targetPositions[1] = directionalEncoderMotors.get(1).getCurrentPosition() + (int) (gamepad.distanceInInches * DEAD_WHEEL_ENCODER_TICKS_PER_INCH);
            }
        }

        if(gamepad.isCurving()) {
            if (gamepad.isCurvingLeft()) {
                rightRear.setPower(rightRearPower * scaleFactor);
                rightFront.setPower(rightFrontPower * scaleFactor);
            } else {
                leftFront.setPower(leftFrontPower * scaleFactor);
                leftRear.setPower(leftRearPower * scaleFactor);
            }
        } else {
            leftFront.setPower(Math.signum(leftFrontPower) * scaleFactor);
            leftRear.setPower(Math.signum(leftRearPower) * scaleFactor);
            rightRear.setPower(Math.signum(rightRearPower) * scaleFactor);
            rightFront.setPower(Math.signum(rightFrontPower)  * scaleFactor);
        }

        runtime.reset();

        // int rightOldEncoderVal = rightFront.getCurrentPosition();
        // int leftOldEncoderVal = leftFront.getCurrentPosition();

        RobotDetectorLimit detectorLimit = null;
        if(gamepad.detector != null && gamepad.detector instanceof  RobotDetectorLimit) {
            detectorLimit = (RobotDetectorLimit) gamepad.detector;
        }

        while(OpModeUtils.opModeIsActive() && runtime.seconds() < gamepad.timeout) {
            if(gamepad.isStrafing()) {
                if((Math.signum(gamepad.left_stick_x) >= 0 && strafeEncoderMotors.get(0).getCurrentPosition() <= targetPositions[0]) ||
                        (Math.signum(gamepad.left_stick_x) < 0 && strafeEncoderMotors.get(0).getCurrentPosition() >= targetPositions[0]))
                {
                    break;
                }

                gamepad.right_stick_x = getAngle() * (Math.PI / 180) / 5;
                angularAdjustment(gamepad);
            } else if(gamepad.isCurving()) {
                if((gamepad.isCurvingRight() && directionalEncoderMotors.get(1).getCurrentPosition() <= targetPositions[1]) ||
                        (gamepad.isCurvingLeft() && directionalEncoderMotors.get(0).getCurrentPosition() <= targetPositions[0])) {
                    break;
                }
            } else if(gamepad.isTurning()) {
                if((gamepad.right_stick_x < 0 && (directionalEncoderMotors.get(0).getCurrentPosition() <= targetPositions[0] ||
                        directionalEncoderMotors.get(1).getCurrentPosition() >= targetPositions[1])) ||
                        (gamepad.right_stick_x > 0 && (directionalEncoderMotors.get(0).getCurrentPosition() >= targetPositions[0] ||
                                directionalEncoderMotors.get(1).getCurrentPosition() <= targetPositions[1])))
                {
                    break;
                }
            } else {
                if(gamepad.left_stick_y >= 0 &&
                    (directionalEncoderMotors.get(0).getCurrentPosition() >= targetPositions[0] ||
                            directionalEncoderMotors.get(1).getCurrentPosition() >= targetPositions[1]) ||
                        (gamepad.left_stick_y < 0 &&
                                (directionalEncoderMotors.get(0).getCurrentPosition() <= targetPositions[0] ||
                                        directionalEncoderMotors.get(1).getCurrentPosition() <= targetPositions[1]))) {
                    break;
                }

                gamepad.right_stick_x = getAngle() * (Math.PI / 180) / 5;
                angularAdjustment(gamepad);
            }

            if(detectorLimit != null) {
                telemetry.addData("Distance", detectorLimit.getCurrentDistance());
                telemetry.update();
            }

            if(gamepad.detector != null && gamepad.detector.isDetected()) {
                break;
            }

            printTelemetry(gamepad, targetPositions);
        }

        stop();

        if(gamepad.resetAngle) {
            globalAngle = 0;
        }

        gamepad.reset();
    }

    @Override
    public void printTelemetry() {
        telemetry.addData("strafe", leftRear.getCurrentPosition());
        telemetry.addData("leftFront", leftFront.getCurrentPosition());
        telemetry.addData("rightFront", rightFront.getCurrentPosition());
    }

    private void printTelemetry(AutoGamepad gamepad, int[] targetPositions) {
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

    private double toAngleinDegrees(double leftEncoder, double rightEncoder) {
        if (Math.signum(leftEncoder * rightEncoder) == -1)
            return ((rightEncoder - leftEncoder) / (4 * TRACKBASE * Math.PI)) * 360;
        else
            return (rightEncoder + leftEncoder) / (2 * TRACKBASE) * 180 / Math.PI;
    }

    private double toAngleinRadians(double leftEncoder, double rightEncoder) {
        if (Math.signum(leftEncoder * rightEncoder) == -1)
            return ((rightEncoder - leftEncoder) / (4 * TRACKBASE * Math.PI)) * 2 * Math.PI;
        else
            return (rightEncoder + leftEncoder) / (2 * TRACKBASE);
    }

    private void angularAdjustment(AutoGamepad gamepad) {
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
        double scaleFactor = (1 / maxPower) * gamepad.power;

        leftFront.setPower(leftFrontPower * scaleFactor);
        leftRear.setPower(leftRearPower * scaleFactor);
        rightRear.setPower(rightRearPower * scaleFactor);
        rightFront.setPower(rightFrontPower * scaleFactor);
    }
}
