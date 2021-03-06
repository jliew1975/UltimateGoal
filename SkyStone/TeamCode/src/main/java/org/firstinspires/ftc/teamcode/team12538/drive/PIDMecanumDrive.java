package org.firstinspires.ftc.teamcode.team12538.drive;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.team12538.ext.AutoGamepad;
import org.firstinspires.ftc.teamcode.team12538.ext.PIDControllerV1;
import org.firstinspires.ftc.teamcode.team12538.ext.PIDControllerV2;
import org.firstinspires.ftc.teamcode.team12538.utils.OpModeUtils;

public class PIDMecanumDrive extends MecanumDrive implements AutoDrive {
    MotorPower motorPower = new MotorPower();

    // Declare information about robot
    private static final double TRACKLENGTH = 6;
    private static final double TRACKBASE = 7.645;
    private static final double WHEELRADIUS = 1.969;

    private double rearWheelFactor = 0.70;

    public PIDControllerV2 leftController = new PIDControllerV2(5, 0, 0);
    public PIDControllerV2 rightController = new PIDControllerV2(5, 0, 0);

    public PIDControllerV2 rotateController = new PIDControllerV2(12d, 10d, 15d, 0.0001, 0.0001);
    public PIDControllerV2 strafeController = new PIDControllerV2(0.001, 0, 0.5);

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
            leftFront.setPower(Math.signum(leftFrontPower) * 0.5);
            leftRear.setPower(Math.signum(leftRearPower) * 0.5);
            rightRear.setPower(Math.signum(rightRearPower) * 0.5);
            rightFront.setPower(Math.signum(rightFrontPower) * 0.5);
        } else {
            leftFront.setPower(leftFrontPower * scaleFactor);
            leftRear.setPower(leftRearPower * scaleFactor);
            rightRear.setPower(rightRearPower * scaleFactor);
            rightFront.setPower(rightFrontPower * scaleFactor);
        }
    }

    @Override
    public boolean autoNavigateWithGamepad(AutoGamepad gamepad) {
        boolean navigated = true;

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
        double scaleFactor = (1/maxPower) * gamepad.power;

        // Calculate the destination encoder tick values for the given distance
        double[] targetPositions = calculateTarget(gamepad);

        if(gamepad.isCurving()) {
            if (gamepad.isCurvingLeft()) {
                rightRear.setPower(rightRearPower * scaleFactor);
                rightFront.setPower(rightFrontPower * scaleFactor);
            } else {
                leftFront.setPower(leftFrontPower * scaleFactor);
                leftRear.setPower(leftRearPower * scaleFactor);
            }
        } else {
            leftFront.setPower(leftFrontPower * scaleFactor);
            leftRear.setPower(leftRearPower * scaleFactor);
            rightRear.setPower(rightRearPower * scaleFactor);
            rightFront.setPower(rightFrontPower  * scaleFactor);
        }

        runtime.reset();

        do {
            if(gamepad.isStrafing()) {
                double strafePower = strafeController.performPID(strafeEncoderMotors.get(0).getCurrentPosition())/DEAD_WHEEL_ENCODER_TICKS_PER_INCH;
                gamepad.left_stick_x = Range.clip(strafePower, -1d, 1d);
            } else if(gamepad.isCurvingRight()) {
                double curvingPower = rightController.performPID(strafeEncoderMotors.get(0).getCurrentPosition())/DEAD_WHEEL_ENCODER_TICKS_PER_INCH;
                gamepad.left_stick_y = Range.clip(curvingPower, -1d, 1d);
            } else if(gamepad.isCurvingLeft()) {
                double curvingPower = leftController.performPID(strafeEncoderMotors.get(1).getCurrentPosition())/DEAD_WHEEL_ENCODER_TICKS_PER_INCH;
                gamepad.left_stick_y = Range.clip(curvingPower, -1d, 1d);
            } else if(gamepad.isTurning()) {
                gamepad.right_stick_x = -rotateController.performPID(getAngle());
            } else {
                /*
                double leftPower = leftController.performPID(directionalEncoderMotors.get(0).getCurrentPosition());
                double rightPower = rightController.performPID(directionalEncoderMotors.get(1).getCurrentPosition());
                double normalizedPower = Math.max(leftPower, rightPower)/DEAD_WHEEL_ENCODER_TICKS_PER_INCH;
                gamepad.left_stick_y = Range.clip(normalizedPower, -1d, 1d);
                */
                if(gamepad.left_stick_y >= 0 &&
                        (directionalEncoderMotors.get(0).getCurrentPosition() >= targetPositions[0] ||
                                directionalEncoderMotors.get(1).getCurrentPosition() >= targetPositions[1]) ||
                        (gamepad.left_stick_y < 0 &&
                                (directionalEncoderMotors.get(0).getCurrentPosition() <= targetPositions[0] ||
                                        directionalEncoderMotors.get(1).getCurrentPosition() <= targetPositions[1]))) {
                    break;
                }

                gamepad.right_stick_x = getAngle() * (Math.PI / 180) / 5;
            }

            if(!gamepad.isCurving() && !gamepad.isTurning()) {
                gamepad.right_stick_x = getAngle() / 20;
            }

            telemetry.addData("target angle", rotateController.getTarget());
            telemetry.addData("current angle", getAngle());
            telemetry.addData("gamepad.right_stick_x", gamepad.right_stick_x);
            telemetry.update();

            angularAdjustment(gamepad);

            if(gamepad.detector != null && gamepad.detector.isDetected()) {
                break;
            }

            // printTelemetry(gamepad, targetPositions);
        } while(OpModeUtils.opModeIsActive() && runtime.seconds() < gamepad.timeout && !isOnTarget(gamepad));

        stop();

        /*
        if(gamepad.resetAngle) {
            worldAngle += globalAngle;
            globalAngle = 0;
        }
        */

        leftController.reset();
        rightController.reset();
        gamepad.reset();

        return navigated;
    }

    public double[] calculateTarget(AutoGamepad gamepad) {
        double[] targetPositions = new double[(gamepad.isStrafing() || gamepad.isCurving()) ? 1 : 2];
        if(gamepad.isStrafing()) {
            if (gamepad.left_stick_x < 0) {
                targetPositions[0] = strafeEncoderMotors.get(0).getCurrentPosition() + (int) (gamepad.distanceInInches * DEAD_WHEEL_ENCODER_TICKS_PER_INCH);
            } else {
                targetPositions[0] = strafeEncoderMotors.get(0).getCurrentPosition() - (int) (gamepad.distanceInInches * DEAD_WHEEL_ENCODER_TICKS_PER_INCH);
            }
            strafeController.setTarget(targetPositions[0]);
        } else if(gamepad.isCurving()) {
            if(gamepad.isCurvingRight()) {
                targetPositions[0] = directionalEncoderMotors.get(0).getCurrentPosition() - (int) (gamepad.distanceInInches * DEAD_WHEEL_ENCODER_TICKS_PER_INCH);
                rightController.setTarget(targetPositions[0]);
            } else {
                targetPositions[0] = directionalEncoderMotors.get(1).getCurrentPosition() - (int) (gamepad.distanceInInches * DEAD_WHEEL_ENCODER_TICKS_PER_INCH);
                leftController.setTarget(targetPositions[0]);
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

            leftController.setTarget(targetPositions[0]);
            rightController.setTarget(targetPositions[1]);
            */
            rotateController.setTarget(getAngle() + gamepad.turnRadian);
        } else {
            if(gamepad.left_stick_y < 0) {
                targetPositions[0] = directionalEncoderMotors.get(0).getCurrentPosition() - (int) (gamepad.distanceInInches * DEAD_WHEEL_ENCODER_TICKS_PER_INCH);
                targetPositions[1] = directionalEncoderMotors.get(1).getCurrentPosition() - (int) (gamepad.distanceInInches * DEAD_WHEEL_ENCODER_TICKS_PER_INCH);
            } else {
                targetPositions[0] = directionalEncoderMotors.get(0).getCurrentPosition() + (int) (gamepad.distanceInInches * DEAD_WHEEL_ENCODER_TICKS_PER_INCH);
                targetPositions[1] = directionalEncoderMotors.get(1).getCurrentPosition() + (int) (gamepad.distanceInInches * DEAD_WHEEL_ENCODER_TICKS_PER_INCH);
            }

            leftController.setTarget(targetPositions[0]);
            rightController.setTarget(targetPositions[1]);
        }

        return targetPositions;
    }

    public void printTelemetry(AutoGamepad gamepad, double[] targetPositions) {
        // report target and current positions to driver station
        if(gamepad.isStrafing()) {
            // report target and current positions to driver station
            telemetry.addData("Target", "Running to %.1f",
                    targetPositions[0]);

            telemetry.addData("Current", "Running at %.1f",
                    strafeEncoderMotors.get(0).getCurrentPosition());

            telemetry.addData( "Power", "Running at %.1f",
                    strafeEncoderMotors.get(0).getPower());
        } else {
            telemetry.addData("Target", "Running to %.1f : %.1f",
                    targetPositions[0], targetPositions[1]);

            telemetry.addData("Current", "Running at %.1f : %.1f",
                    directionalEncoderMotors.get(0).getCurrentPosition(),
                    directionalEncoderMotors.get(1).getCurrentPosition());

            telemetry.addData("Power", "Running at %.1f : %.1f",
                    directionalEncoderMotors.get(0).getPower(),
                    directionalEncoderMotors.get(1).getPower());

            telemetry.addData("leftController.onTarget()", leftController.onTarget());
            telemetry.addData("rightController.onTarget()", rightController.onTarget());
            telemetry.addData("leftError", leftController.getError());
            telemetry.addData("rightError", rightController.getError());

            telemetry.addData("kP", leftController.getKP());
            telemetry.addData("kI", leftController.getKI());
            telemetry.addData("kD", leftController.getKD());
        }

        telemetry.update();
    }

    private boolean isOnTarget(AutoGamepad gamepad) {
        if(gamepad.isStrafing()) {
            return strafeController.onTarget();
        } else if(gamepad.isCurvingLeft()) {
            return leftController.onTarget();
        } else if(gamepad.isCurvingRight()) {
            return rightController.onTarget();
        } else if(gamepad.isTurning()) {
            return rotateController.onTarget();
        } else {
            return (leftController.onTarget() || rightController.onTarget());
        }
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

        if (gamepad.isCurvingLeft()) {
            rightRear.setPower(rightRearPower * scaleFactor);
            rightFront.setPower(rightFrontPower * scaleFactor);
        } else if(gamepad.isCurvingRight()) {
            leftFront.setPower(leftFrontPower * scaleFactor);
            leftRear.setPower(leftRearPower * scaleFactor);
        } else {
            leftFront.setPower(leftFrontPower * scaleFactor);
            leftRear.setPower(leftRearPower * scaleFactor);
            rightRear.setPower(rightRearPower * scaleFactor);
            rightFront.setPower(rightFrontPower * scaleFactor);
        }
    }
}

