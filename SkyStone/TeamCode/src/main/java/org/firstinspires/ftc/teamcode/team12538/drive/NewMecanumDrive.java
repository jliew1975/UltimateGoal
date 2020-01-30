package org.firstinspires.ftc.teamcode.team12538.drive;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.team12538.components.TelemetryAware;
import org.firstinspires.ftc.teamcode.team12538.ext.AutoGamepad;
import org.firstinspires.ftc.teamcode.team12538.detectors.RobotDetectorLimit;
import org.firstinspires.ftc.teamcode.team12538.ext.PIDControllerV1;
import org.firstinspires.ftc.teamcode.team12538.ext.PIDControllerV2;
import org.firstinspires.ftc.teamcode.team12538.utils.MotorUtils;
import org.firstinspires.ftc.teamcode.team12538.utils.OpModeUtils;

import java.util.ArrayList;
import java.util.List;

public class NewMecanumDrive extends MecanumDrive implements AutoDrive, TelemetryAware {
    MotorPower motorPower = new MotorPower();

    // Declare information about robot
    private static final double TRACKLENGTH = 6;
    private static final double TRACKBASE = 7.645;
    private static final double WHEELRADIUS = 1.969;

    private double rearWheelFactor = 0.70;

    public PIDControllerV2 rotateController = new PIDControllerV2(12d, 10d, 15d, 0.0001, 0.0001);

    @Override
    public void stop() {
        for(DcMotor motor : driveMotorList) {
            motor.setPower(0d);
        }
    }

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
            scaleFactor *= 0.5;
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
    public boolean autoNavigateWithGamepad(AutoGamepad gamepad) {
        boolean navigated = false;

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
        List<Integer> targetPositions = calculateTargetPositions(gamepad);

        if(OpModeUtils.opModeIsActive()) {
            if (gamepad.isCurving()) {
                if (gamepad.isCurvingLeft()) {
                    leftFront.setPower(0d);
                    leftRear.setPower(0d);
                    rightRear.setPower(rightRearPower * scaleFactor);
                    rightFront.setPower(rightFrontPower * scaleFactor);
                } else {
                    leftFront.setPower(leftFrontPower * scaleFactor);
                    leftRear.setPower(leftRearPower * scaleFactor);
                    rightFront.setPower(0d);
                    rightRear.setPower(0d);
                }
            } else if (gamepad.isDiagonal()) {
                if (gamepad.isDiagonalLeft()) {
                    leftFront.setPower(0d);
                    leftRear.setPower(leftFrontPower * scaleFactor);
                    rightFront.setPower(rightRearPower * scaleFactor);
                    rightRear.setPower(0d);
                } else {
                    leftFront.setPower(rightRearPower * scaleFactor);
                    leftRear.setPower(0d);
                    rightFront.setPower(0d);
                    rightRear.setPower(leftFrontPower * scaleFactor);
                }
            } else {
                leftFront.setPower(leftFrontPower * scaleFactor);
                leftRear.setPower(leftRearPower * scaleFactor);
                rightRear.setPower(rightRearPower * scaleFactor);
                rightFront.setPower(rightFrontPower * scaleFactor);
            }
        } else {
            stop();
        }

        runtime.reset();

        RobotDetectorLimit detectorLimit = null;
        if(gamepad.detector != null && gamepad.detector instanceof  RobotDetectorLimit) {
            detectorLimit = (RobotDetectorLimit) gamepad.detector;
        }

        if(gamepad.isTurning()) {
            do {
                gamepad.right_stick_x = -rotateController.performPID(getAngle());
                angularAdjustment(gamepad);
            } while(OpModeUtils.opModeIsActive() && !isOntarget(gamepad) && runtime.seconds() < gamepad.timeout);

            if(isOntarget(gamepad)) {
                navigated = true;
            }

            telemetry.addData("Target", rotateController.getTarget());
            telemetry.addData("Current", getAngle());
            telemetry.update();
        } else {
            while(OpModeUtils.opModeIsActive() && runtime.seconds() < gamepad.timeout) {
                if(gamepad.isStrafing()) {
                    if((Math.signum(gamepad.left_stick_x) >= 0 && strafeEncoderMotors.get(0).getCurrentPosition() <= targetPositions.get(0) ||
                            (Math.signum(gamepad.left_stick_x) < 0 && strafeEncoderMotors.get(0).getCurrentPosition() >= targetPositions.get(0))))
                    {
                        navigated = true;
                        break;
                    }
                } else if(gamepad.isCurving()) {
                    if (gamepad.backCurving) {
                        if ((gamepad.isCurvingRight() && directionalEncoderMotors.get(1).getCurrentPosition() >= targetPositions.get(0)) ||
                                (gamepad.isCurvingLeft() && directionalEncoderMotors.get(0).getCurrentPosition() >= targetPositions.get(0))) {
                            navigated = true;
                            break;
                        }
                    } else {
                        if ((gamepad.isCurvingRight() && directionalEncoderMotors.get(1).getCurrentPosition() <= targetPositions.get(0)) ||
                                (gamepad.isCurvingLeft() && directionalEncoderMotors.get(0).getCurrentPosition() <= targetPositions.get(0))) {
                            navigated = true;
                            break;
                        }
                    }
                } else if(gamepad.isDiagonal()) {
                   if(directionalEncoderMotors.get(0).getCurrentPosition() >= targetPositions.get(0) ||
                           directionalEncoderMotors.get(1).getCurrentPosition() >= targetPositions.get(1) ||
                           (gamepad.isDiagonalLeft() &&
                                   strafeEncoderMotors.get(0).getCurrentPosition() >= targetPositions.get(2)) ||
                           (gamepad.isDiagonalRight() &&
                                   strafeEncoderMotors.get(0).getCurrentPosition() <= targetPositions.get(2)))
                   {
                       break;
                   }
                } else {
                    if(gamepad.left_stick_y >= 0 &&
                            (directionalEncoderMotors.get(0).getCurrentPosition() >= targetPositions.get(0) ||
                                    directionalEncoderMotors.get(1).getCurrentPosition() >= targetPositions.get(1)) ||
                            (gamepad.left_stick_y < 0 &&
                                    (directionalEncoderMotors.get(0).getCurrentPosition() <= targetPositions.get(0) ||
                                            directionalEncoderMotors.get(1).getCurrentPosition() <= targetPositions.get(1))))
                    {
                        navigated = true;
                        break;
                    }
                }

                if(!gamepad.isCurving() && !gamepad.isTurning()) {
                    gamepad.right_stick_x = getAngle() / 15;
                    angularAdjustment(gamepad);
                }

                if(detectorLimit != null) {
                    telemetry.addData("Distance", detectorLimit.getCurrentDistance());
                    telemetry.update();
                }

                if(gamepad.detector != null && gamepad.detector.isDetected()) {
                    navigated = true;
                    break;
                }

                printTelemetry(gamepad, targetPositions);
            }
        }

        if(gamepad.stopMotor) {
            stop();
        }

        if(gamepad.resetAngle) {
            resetAngle();
        }

        gamepad.reset();
        MotorUtils.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER, driveMotorList);
        MotorUtils.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER, driveMotorList);

        return navigated;
    }

    @Override
    public void flipLastAngleForErrorCorrection(LastAngleMode lastAngleMode) {
        switch(lastAngleMode) {
            case AudienceDirectionBlueAlliance:
                this.lastAngles = angleFacingAudienceBlueAlliance;
                break;
            case AudienceDirectionRedAlliance:
                this.lastAngles = angleFacingAudienceRedAlliance;
                break;
            default:
                this.lastAngles = angleFacingStone;
                break;
        }

        // reset delta to 0;
        globalAngle = 0;
    }

    @Override
    public void printTelemetry() {
        telemetry.addData("leftFront", leftFront.getCurrentPosition());
        telemetry.addData("rightFront", rightFront.getCurrentPosition());
        telemetry.addData("strafe", leftRear.getCurrentPosition());
    }

    private void printTelemetry(AutoGamepad gamepad, List<Integer> targetPositions) {
        // report target and current positions to driver station
        if(gamepad.isStrafing()) {
            // report target and current positions to driver station
            telemetry.addData("Target", "Running to %7d",
                    targetPositions.get(0));

            telemetry.addData("Current", "Running at %7d",
                    strafeEncoderMotors.get(0).getCurrentPosition());

            telemetry.addData( "Power", "Running at %.1f",
                    strafeEncoderMotors.get(0).getPower());
        } else if(gamepad.isCurving()) {
            telemetry.addData("Target", "Running to %7d", targetPositions.get(0));

            if(gamepad.isCurvingLeft()) {
                telemetry.addData("Current", "Running at %7d",
                        directionalEncoderMotors.get(0).getCurrentPosition());
            } else {
                telemetry.addData("Current", "Running at %7d",
                        directionalEncoderMotors.get(1).getCurrentPosition());
            }
        } else {
            telemetry.addData("Target", "Running to %7d : %7d",
                    targetPositions.get(0), targetPositions.get(1));

            telemetry.addData("Current", "Running at %7d : %7d",
                    directionalEncoderMotors.get(0).getCurrentPosition(),
                    directionalEncoderMotors.get(1).getCurrentPosition());

            telemetry.addData("Power", "Running at %.1f : %.1f",
                    directionalEncoderMotors.get(0).getPower(),
                    directionalEncoderMotors.get(1).getPower());
        }

        telemetry.update();
    }

    private List<Integer> calculateTargetPositions(AutoGamepad gamepad) {
        List<Integer> targetPositions = new ArrayList<>(); //int[gamepad.isStrafing() ? strafeEncoderMotors.size() : directionalEncoderMotors.size()];
        if (gamepad.isStrafing()) {
            if (gamepad.left_stick_x < 0) {
                targetPositions.add(strafeEncoderMotors.get(0).getCurrentPosition() + (int) (gamepad.distanceInInches * DEAD_WHEEL_ENCODER_TICKS_PER_INCH));
            } else {
                targetPositions.add(strafeEncoderMotors.get(0).getCurrentPosition() - (int) (gamepad.distanceInInches * DEAD_WHEEL_ENCODER_TICKS_PER_INCH));
            }
        } else if (gamepad.isCurving()) {
            if(gamepad.isCurvingLeft()) {
                if (gamepad.backCurving) {
                    targetPositions.add(directionalEncoderMotors.get(0).getCurrentPosition() + (int) (gamepad.distanceInInches * DEAD_WHEEL_ENCODER_TICKS_PER_INCH));
                } else {
                    targetPositions.add(directionalEncoderMotors.get(0).getCurrentPosition() - (int) (gamepad.distanceInInches * DEAD_WHEEL_ENCODER_TICKS_PER_INCH));
                }
            } else {
                if (gamepad.backCurving) {
                    targetPositions.add(directionalEncoderMotors.get(1).getCurrentPosition() + (int) (gamepad.distanceInInches * DEAD_WHEEL_ENCODER_TICKS_PER_INCH));
                } else {
                    targetPositions.add(directionalEncoderMotors.get(1).getCurrentPosition() - (int) (gamepad.distanceInInches * DEAD_WHEEL_ENCODER_TICKS_PER_INCH));
                }
            }
        } else if (gamepad.isTurning()) {
            rotateController.setTarget(getAngle() + gamepad.turnRadian);
        } else if (gamepad.isDiagonal()) {
            targetPositions.add(directionalEncoderMotors.get(0).getCurrentPosition() - (int) (gamepad.distanceInInches * DEAD_WHEEL_ENCODER_TICKS_PER_INCH));
            targetPositions.add(directionalEncoderMotors.get(1).getCurrentPosition() - (int) (gamepad.distanceInInches * DEAD_WHEEL_ENCODER_TICKS_PER_INCH));

            if(gamepad.isDiagonalLeft()) {
                targetPositions.add(strafeEncoderMotors.get(0).getCurrentPosition() + (int) (gamepad.distanceInInches * DEAD_WHEEL_ENCODER_TICKS_PER_INCH));
            } else {
                targetPositions.add(strafeEncoderMotors.get(0).getCurrentPosition() - (int) (gamepad.distanceInInches * DEAD_WHEEL_ENCODER_TICKS_PER_INCH));
            }
        } else {
            if(gamepad.left_stick_y < 0) {
                targetPositions.add(directionalEncoderMotors.get(0).getCurrentPosition() - (int) (gamepad.distanceInInches * DEAD_WHEEL_ENCODER_TICKS_PER_INCH));
                targetPositions.add(directionalEncoderMotors.get(1).getCurrentPosition() - (int) (gamepad.distanceInInches * DEAD_WHEEL_ENCODER_TICKS_PER_INCH));
            } else {
                targetPositions.add(directionalEncoderMotors.get(0).getCurrentPosition() + (int) (gamepad.distanceInInches * DEAD_WHEEL_ENCODER_TICKS_PER_INCH));
                targetPositions.add(directionalEncoderMotors.get(1).getCurrentPosition() + (int) (gamepad.distanceInInches * DEAD_WHEEL_ENCODER_TICKS_PER_INCH));
            }
        }
        return targetPositions;
    }

    private boolean isOntarget(AutoGamepad gamepad) {
        return gamepad.right_stick_x == 0;
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

        if(gamepad.isCurving()) {
            if (gamepad.isCurvingLeft()) {
                rightRear.setPower(rightRearPower * scaleFactor);
                rightFront.setPower(rightFrontPower * scaleFactor);
            } else {
                leftFront.setPower(leftFrontPower * scaleFactor);
                leftRear.setPower(leftRearPower * scaleFactor);
            }
        } else if(gamepad.isDiagonal()) {
            if(gamepad.isDiagonalLeft()) {
                leftFront.setPower(0d);
                leftRear.setPower(leftFrontPower * scaleFactor);
                rightFront.setPower(rightRearPower * scaleFactor);
                rightRear.setPower(0d);
            } else {
                leftFront.setPower(rightRearPower * scaleFactor);
                leftRear.setPower(0d);
                rightFront.setPower(0d);
                rightRear.setPower(leftFrontPower * scaleFactor);
            }
        } else {
            leftFront.setPower(leftFrontPower * scaleFactor);
            leftRear.setPower(leftRearPower * scaleFactor);
            rightRear.setPower(rightRearPower * scaleFactor);
            rightFront.setPower(rightFrontPower * scaleFactor);
        }
    }
}
