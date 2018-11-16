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
import org.firstinspires.ftc.teamcode.team12538.detectors.GoldAlignDetectorExt;
import org.firstinspires.ftc.teamcode.team12538.utils.MotorUtils;
import org.firstinspires.ftc.teamcode.team12538.utils.OpModeUtils;

import java.util.List;
import java.util.Locale;
import java.util.concurrent.TimeUnit;

public class AutoRobotV1 extends RobotBase {
    private final double COUNTS_PER_MOTOR_REV = 560; // andymark 20:1 gearbox
    private final double DRIVE_GEAR_REDUCTION = 1.0;  // drive train geared down 1:2
    private final double WHEEL_DIAMETER_INCHES = 4.0;
    private final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);

    protected BNO055IMUImpl imu = null;

    protected double globalAngle;
    protected Orientation lastAngles = new Orientation();

    PIDController pidRotate;

    private Telemetry telemetry = null;
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void init() {
        super.init();
        MotorUtils.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER, motors);
        MotorUtils.setMode(DcMotor.RunMode.RUN_USING_ENCODER, motors);

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

        while (!imu.isGyroCalibrated()) {

        };

        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        // Set PID proportional value to start reducing power at about 50 degrees of rotation.
        pidRotate = new PIDController(.005, 0, 0);

        telemetry.addData("imu", "finish imu calabration");
        telemetry.addData("imu calib status", imu.getCalibrationStatus().toString());
        telemetry.update();
    }

    public void unlatchFromLander() {
        robotLatch.unlatch();
    }

    public void expandMechanism() {
        collector.flipCollectorBox(0.6);
        collector.postionArm(255);
        collector.flipCollectorBox(0d);
        // collector.swingArmToPosition(80, 0.2);
    }

    public void moveForward(double power, double distance) {
        // limit power to 1
        power = limitPower(power);
        encoderDrive(power, distance, 5);
    }

    public void moveBackward(double power, double distance) {
        // limit power to 1
        power = limitPower(power);
        encoderDrive(power, -distance, 5);
    }

    public void strafeLeft(double power, double distance) {
        strafeLeft(power, distance, null);
    }

    public void strafeLeft(double power, double distance, GoldAlignDetectorExt detector) {
        // limit power to 1
        power = limitPower(power);
        encoderStrafe(StrafingDirection.Left, power, distance, 5, detector);
    }

    public void strafeRight(double power, double distance) {
        strafeRight(power, distance, null);
    }

    public void strafeRight(double power, double distance, GoldAlignDetectorExt detector) {
        // limit power to 1
        power = limitPower(power);
        encoderStrafe(StrafingDirection.Right, power, distance, 5, detector);

    }

    public void rotate(int degrees, double power) throws InterruptedException {
        rotate(degrees, power, null);
    }

    /**
     * Rotate left or right the number of degrees. Does not support turning more than 180 degrees.
     * @param degrees Degrees to turn, + is left - is right
     */
    public void rotate(int degrees, double power, GoldAlignDetectorExt detector) throws InterruptedException {
        // restart imu movement tracking.
        resetAngle();

        // getAngle() returns + when rotating counter clockwise (left) and - when rotating
        // clockwise (right).
        if (degrees < 0)
        {
            // On right turn we have to get off zero first.
            while (OpModeUtils.opModeIsActive() && getAngle() == 0) {
                turnRight(power);
                TimeUnit.MILLISECONDS.sleep(100);
            }

            while (OpModeUtils.opModeIsActive() && getAngle() > degrees) {
               if(detector != null && detector.isFound()) {
                   if(detector.isAligned()) {
                       break;
                   }
               }
            }
        } else {  // left turn.
            turnLeft(power);
            while (OpModeUtils.opModeIsActive() && getAngle() < degrees) {
                if(detector != null && detector.isFound()) {
                    if(detector.isAligned()) {
                        break;
                    }
                }
            }
        }

        stop();
        TimeUnit.MILLISECONDS.sleep(200);

        // reset angle tracking on new heading.
        resetAngle();
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

    protected void encoderDrive(double speed, double distanceInInches, double timeout)
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

            //keep looping until one of the motors finished its movement
            while(OpModeUtils.opModeIsActive() &&
                    //(runtime.seconds() < timeout) &&
                    (motors.get(0).isBusy() && motors.get(1).isBusy() && motors.get(2).isBusy() && motors.get(3).isBusy()))
            {

                //report target and current positions to driver station
                telemetry.addData("Path1", "Running to %7d : %7d : %7d : %7d",
                        targetPositions[0], targetPositions[1], targetPositions[2], targetPositions[3]);

                telemetry.addData("Path2", "Running at %7d : %7d : %7d : %7d",
                        motors.get(0).getCurrentPosition(),
                        motors.get(1).getCurrentPosition(),
                        motors.get(2).getCurrentPosition(),
                        motors.get(3).getCurrentPosition());
                telemetry.update();
            }

            stop();
            MotorUtils.setMode(DcMotor.RunMode.RUN_USING_ENCODER, motors);
        }
    }

    private void encoderStrafe(StrafingDirection direction, double speed, double distance, double timeout, GoldAlignDetectorExt detector) {
        // reset encoders
        // MotorUtils.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER, motors);
        MotorUtils.setMode(DcMotor.RunMode.RUN_USING_ENCODER, motors);

        int[] targetPositions = new int[motors.size()];

        double newDistance = (direction == StrafingDirection.Left) ? -1 * distance : distance;

        //ensure that the opmode is still active
        if(OpModeUtils.getOpMode().opModeIsActive()) {
            int index = 0;
            for(DcMotor motor : motors) {
                // bleftDrive or frightDrive
                if(index == 2 || index == 3) {
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

                //report current and target positions to driver station
                telemetry.addData("Path1", "Running to %7d :%7d",
                        targetPositions[0], targetPositions[1], targetPositions[2], targetPositions[3]);

                telemetry.addData("Path2", "Running at %7d :%7d",
                        motors.get(0).getCurrentPosition(),
                        motors.get(1).getCurrentPosition(),
                        motors.get(2).getCurrentPosition(),
                        motors.get(3).getCurrentPosition());
                telemetry.update();
            }

            stop();
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
