package org.firstinspires.ftc.teamcode.team12538.robotV1;


import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.BNO055IMUImpl;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.team12538.controller.PIDController;
import org.firstinspires.ftc.teamcode.team12538.detectors.MineralDetector;
import org.firstinspires.ftc.teamcode.team12538.utils.MotorUtils;
import org.firstinspires.ftc.teamcode.team12538.utils.OpModeUtils;
import org.firstinspires.ftc.teamcode.team12538.utils.ThreadUtils;

import java.util.List;
import java.util.Locale;
import java.util.concurrent.TimeUnit;

public class AutoRobotV1 extends RobotBase {
    private final double COUNTS_PER_MOTOR_REV = 560; // andymark 20:1 gearbox
    private final double DRIVE_GEAR_REDUCTION = 1.0;  // drive train geared down 1:2
    private final double WHEEL_DIAMETER_INCHES = 4.0;
    private final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);

    private BNO055IMUImpl imu = null;

    private double globalAngle;
    private Orientation lastAngles = new Orientation();

    private PIDController pidRotate;

    private Telemetry telemetry = null;
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void init() {
        super.init();
        MotorUtils.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER, motors);
        MotorUtils.setMode(DcMotor.RunMode.RUN_USING_ENCODER, motors);

        pidRotate = new PIDController(.05, 0, 0);

        telemetry = OpModeUtils.getGlobalStore().getTelemetry();
    }

    public void init_imu() {
        imu = OpModeUtils.getGlobalStore().getHardwareMap().get(BNO055IMUImpl.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.loggingEnabled = false;   //For debugging
        imu.initialize(parameters);

        while (OpModeUtils.opModeIsActive() && !imu.isGyroCalibrated()) {
            ThreadUtils.idle();
        }

        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        Telemetry telemetry = OpModeUtils.getGlobalStore().getTelemetry();
        telemetry.addData("imu", "finish imu calabration");
        telemetry.addData("imu calib status", imu.getCalibrationStatus().toString());
        telemetry.update();
    }

    public void moveForward(double power, double distance) {
        moveForward(power, distance, true);
    }

    public void moveForward(double power, double distance, boolean isStop) {
        // limit power to 1
        power = limitPower(power);
        encoderDrive(power, -distance, 5, isStop);
    }

    public void moveBackward(double power, double distance) {
        moveBackward(power, distance, true);
    }

    public void moveBackward(double power, double distance, boolean isStop) {
        // limit power to 1
        power = limitPower(power);
        encoderDrive(power, distance, 5, isStop);
    }

    public void strafeLeft(double power, double distance) {
        strafeLeft(power, distance, null);
    }

    public void strafeLeft(double power, double distance, MineralDetector detector) {
        // limit power to 1
        power = limitPower(power);
        encoderStrafe(StrafingDirection.Left, power, distance, 5, detector);
    }

    public void strafeRight(double power, double distance) {
        strafeRight(power, distance, null);
    }

    public void strafeRight(double power, double distance, MineralDetector detector) {
        // limit power to 1
        power = limitPower(power);
        encoderStrafe(StrafingDirection.Right, power, distance, 5, detector);

    }

    public double rotate(double degrees, double power, double timeout) throws InterruptedException {
        return rotate(degrees, power, timeout, null);
    }

    /**
     * Rotate left or right the number of degrees. Does not support turning more than 180 degrees.
     * @param degrees Degrees to turn, + is left - is right
     */
    public double rotate(double degrees, double power, double timeout, MineralDetector detector) throws InterruptedException {
        // restart imu movement tracking.
        resetAngle();

        runtime.reset();

        if(detector == null || (detector != null && !detector.isAligned())) {
            // getAngle() returns + when rotating counter clockwise (left) and - when rotating
            // clockwise (right).
            if (degrees < 0) {
                // On right turn we have to get off zero first.
                while (OpModeUtils.opModeIsActive() && getAngle() == 0) {
                    telemetry.addData("angle", getAngle());
                    telemetry.addData("degrees", 0);
                    telemetry.update();

                    turnRight(power);
                }

                turnRight(power);

                do {
                    telemetry.addData("angle", getAngle());
                    telemetry.addData("degrees", degrees);
                    if(detector != null) {
                        telemetry.addData("IsAlign", detector.isAligned());
                    }
                    telemetry.update();

                    if (detector != null && detector.isAligned()) {
                        stop();
                        break;
                    }

                    if (timeout != -1 && runtime.seconds() > timeout) {
                        stop();
                        break;
                    }
                } while (OpModeUtils.opModeIsActive() && getAngle() > degrees);
            } else {  // left turn.
                turnLeft(power);

                do {
                    telemetry.addData("angle", getAngle());
                    telemetry.addData("degrees", degrees);
                    if(detector != null) {
                        telemetry.addData("IsAlign", detector.isAligned());
                    }
                    telemetry.update();

                    if (detector != null && detector.isAligned()) {
                        stop();
                        break;
                    }

                    if (timeout != -1 && runtime.seconds() > timeout) {
                        stop();
                        break;
                    }
                } while (OpModeUtils.opModeIsActive() && getAngle() < degrees);
            }
        }

        stop();
        TimeUnit.MILLISECONDS.sleep(200);

        double rotateAngle = getAngle();

        // reset angle tracking on new heading.
        resetAngle();

        return rotateAngle;
    }

    public void corneringRight(double power, double distance) throws InterruptedException {
        corneringRight(power, distance, true);
    }

    public void corneringRight(double power, double distance, boolean isStop) throws InterruptedException {
        encoderCorneringRight(power, distance, isStop);
    }

    public void corneringLeft(double power, double distance) throws InterruptedException {
        corneringLeft(power, distance, true);
    }

    public void corneringLeft(double power, double distance, boolean isStop) throws InterruptedException {
        encoderCorneringLeft(power, distance, isStop);
    }

    /**
     * Get current cumulative angle rotation from last reset.
     * @return Angle in degrees. + = left, - = right.
     */
    public double getAngle()
    {
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;
        if (deltaAngle < -180) {
            deltaAngle += 360;
        } else if (deltaAngle > 180) {
            deltaAngle -= 360;
        }

        globalAngle += deltaAngle;
        lastAngles = angles;

        return globalAngle;
    }

    public void printImuAngleTelemtry() {
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        telemetry.addData("Status", imu.getSystemStatus().toString());
        telemetry.addData("Calib", imu.getCalibrationStatus().toString());
        telemetry.addData("Heading", formatAngle(angles.angleUnit, angles.firstAngle));
        telemetry.addData("Roll", formatAngle(angles.angleUnit, angles.secondAngle));
        telemetry.addData("Pitch", formatAngle(angles.angleUnit, angles.thirdAngle));
        telemetry.addData("Angle", getAngle());
    }

    public void printDriveEncoderTelemtry() {
        telemetry.addData("frontLeftDrive", frontLeftDrive.getCurrentPosition());
        telemetry.addData("frontRightDrive", frontRightDrive.getCurrentPosition());
        telemetry.addData("rearLeftDrive", rearLeftDrive.getCurrentPosition());
        telemetry.addData("rearRightDrive", rearRightDrive.getCurrentPosition());
    }

    private void encoderDrive(double speed, double distanceInInches, double timeout) {
        encoderDrive(speed, distanceInInches, timeout, true);
    }

    private void encoderDrive(double speed, double distanceInInches, double timeout, boolean isStop)
    {
        // reset encoders
        MotorUtils.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER, motors);
        MotorUtils.setMode(DcMotor.RunMode.RUN_USING_ENCODER, motors);

        int[] targetPositions = new int[motors.size()];

        //ensure that the opmode is still active
        if(OpModeUtils.getOpMode().opModeIsActive()) {
            //determine target positions
            int index = 0;
            for(DcMotor motor : motors) {
                targetPositions[index] = motor.getCurrentPosition() + (int) (distanceInInches * COUNTS_PER_INCH);
                motor.setTargetPosition(targetPositions[index]);
                index++;
            }

            MotorUtils.setMode(DcMotor.RunMode.RUN_TO_POSITION, motors);
            runtime.reset();

            for(DcMotor motor : motors) {
                motor.setPower(Math.abs(speed));
            }

            // keep looping until at least one of the motors finished its movement
            while(OpModeUtils.opModeIsActive() &&
                    //(runtime.seconds() < timeout) &&
                    (motors.get(0).isBusy() && motors.get(1).isBusy() && motors.get(2).isBusy() && motors.get(3).isBusy()))
            {

                //report target and current positions to driver station
                /*
                telemetry.addData("Path1", "Running to %7d : %7d : %7d : %7d",
                        targetPositions[0], targetPositions[1], targetPositions[2], targetPositions[3]);

                telemetry.addData("Path2", "Running at %7d : %7d : %7d : %7d",
                        motors.get(0).getCurrentPosition(),
                        motors.get(1).getCurrentPosition(),
                        motors.get(2).getCurrentPosition(),
                        motors.get(3).getCurrentPosition());
                telemetry.update();
                */
            }

            if(isStop) {
                stop();
            }

            MotorUtils.setMode(DcMotor.RunMode.RUN_USING_ENCODER, motors);
        }
    }

    private void encoderStrafe(StrafingDirection direction, double speed, double distance, double timeout, MineralDetector detector) {
        // reset encoders
        // MotorUtils.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER, motors);
        MotorUtils.setMode(DcMotor.RunMode.RUN_USING_ENCODER, motors);

        int[] targetPositions = new int[motors.size()];

        double newDistance = (direction == StrafingDirection.Right) ? distance : -1 * distance;

        //ensure that the opmode is still active
        if(OpModeUtils.getOpMode().opModeIsActive()) {
            int index = 0;
            for(DcMotor motor : motors) {
                // bleftDrive or frightDrive
                if(index == 1 || index == 3) {
                    targetPositions[index] = motor.getCurrentPosition() + (int) (-newDistance * Math.sqrt(2) * COUNTS_PER_INCH);
                } else {
                    targetPositions[index] = motor.getCurrentPosition() + (int) (newDistance * Math.sqrt(2) * COUNTS_PER_INCH);
                }
                motor.setTargetPosition(targetPositions[index]);
                index++;
            }

            MotorUtils.setMode(DcMotor.RunMode.RUN_TO_POSITION, motors);

            runtime.reset();

            for(DcMotor motor : motors) {
                motor.setPower(Math.abs(speed));
            }

            //keep looping until one of the motors finished its movement
            while (OpModeUtils.getOpMode().opModeIsActive() &&
                    (runtime.seconds() < timeout) &&
                    (motorsIsBusy(motors)))
            {
                if(detector != null) {
                    if(detector.isAligned()) {
                        break;
                    }
                }

                /*
                //report current and target positions to driver station
                telemetry.addData("Path1", "Running to %7d :%7d",
                        targetPositions[0], targetPositions[1], targetPositions[2], targetPositions[3]);

                telemetry.addData("Path2", "Running at %7d :%7d",
                        motors.get(0).getCurrentPosition(),
                        motors.get(1).getCurrentPosition(),
                        motors.get(2).getCurrentPosition(),
                        motors.get(3).getCurrentPosition());
                telemetry.update();
                */
            }

            stop();
        }
    }

    private void encoderCorneringRight(double speed, double distanceInInches) {
        encoderCorneringRight(speed, distanceInInches, true);
    }

    private void encoderCorneringRight(double speed, double distanceInInches, boolean isStop)
    {
        // reset encoders
        MotorUtils.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER, leftMotors);
        MotorUtils.setMode(DcMotor.RunMode.RUN_USING_ENCODER, leftMotors);

        int[] targetPositions = new int[leftMotors.size()];

        //ensure that the opmode is still active
        if(OpModeUtils.getOpMode().opModeIsActive()) {
            //determine target positions
            int index = 0;
            for(DcMotor motor : leftMotors) {
                targetPositions[index] = motor.getCurrentPosition() + (int) (distanceInInches * COUNTS_PER_INCH);
                motor.setTargetPosition(targetPositions[index]);
                index++;
            }

            MotorUtils.setMode(DcMotor.RunMode.RUN_TO_POSITION, leftMotors);
            runtime.reset();

            for(DcMotor motor : rightMotors) {
                motor.setPower(0d);
            }

            for(DcMotor motor : leftMotors) {
                motor.setPower(Math.abs(speed));
            }

            // keep looping until at least one of the motors finished its movement
            while(OpModeUtils.opModeIsActive() &&
                    //(runtime.seconds() < timeout) &&
                    (leftMotors.get(0).isBusy() && leftMotors.get(1).isBusy()))
            {

                //report target and current positions to driver station
                /*
                telemetry.addData("Path1", "Running to %7d : %7d : %7d : %7d",
                        targetPositions[0], targetPositions[1], targetPositions[2], targetPositions[3]);

                telemetry.addData("Path2", "Running at %7d : %7d : %7d : %7d",
                        motors.get(0).getCurrentPosition(),
                        motors.get(1).getCurrentPosition(),
                        motors.get(2).getCurrentPosition(),
                        motors.get(3).getCurrentPosition());
                telemetry.update();
                */
            }

            if(isStop) {
                stop();
            }

            MotorUtils.setMode(DcMotor.RunMode.RUN_USING_ENCODER, leftMotors);
        }
    }

    private void encoderCorneringLeft(double speed, double distanceInInches) {
        encoderCorneringLeft(speed, distanceInInches, true);
    }

    private void encoderCorneringLeft(double speed, double distanceInInches, boolean isStop)
    {
        // reset encoders
        MotorUtils.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER, rightMotors);
        MotorUtils.setMode(DcMotor.RunMode.RUN_USING_ENCODER, rightMotors);

        int[] targetPositions = new int[leftMotors.size()];

        //ensure that the opmode is still active
        if(OpModeUtils.getOpMode().opModeIsActive()) {
            //determine target positions
            int index = 0;
            for(DcMotor motor : rightMotors) {
                targetPositions[index] = motor.getCurrentPosition() + (int) (distanceInInches * COUNTS_PER_INCH);
                motor.setTargetPosition(targetPositions[index]);
                index++;
            }

            MotorUtils.setMode(DcMotor.RunMode.RUN_TO_POSITION, rightMotors);
            runtime.reset();

            for(DcMotor motor : rightMotors) {
                motor.setPower(Math.abs(speed));
            }

            // keep looping until at least one of the motors finished its movement
            while(OpModeUtils.opModeIsActive() &&
                    //(runtime.seconds() < timeout) &&
                    (rightMotors.get(0).isBusy() && rightMotors.get(1).isBusy()))
            {

                //report target and current positions to driver station
                /*
                telemetry.addData("Path1", "Running to %7d : %7d : %7d : %7d",
                        targetPositions[0], targetPositions[1], targetPositions[2], targetPositions[3]);

                telemetry.addData("Path2", "Running at %7d : %7d : %7d : %7d",
                        motors.get(0).getCurrentPosition(),
                        motors.get(1).getCurrentPosition(),
                        motors.get(2).getCurrentPosition(),
                        motors.get(3).getCurrentPosition());
                telemetry.update();
                */
            }

            if(isStop) {
                stop();
            }

            MotorUtils.setMode(DcMotor.RunMode.RUN_USING_ENCODER, rightMotors);
        }
    }

    private boolean motorsIsBusy(List<DcMotor> motors) {
        return motors.get(0).isBusy() && motors.get(1).isBusy() && motors.get(2).isBusy() && motors.get(3).isBusy();
    }

    /**
     * Resets the cumulative angle tracking to zero.
     */
    private void resetAngle()
    {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        globalAngle = 0;
    }

    private String formatAngle(AngleUnit angleUnit, double angle)
    {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    private String formatDegrees(double degrees)
    {
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }
}
