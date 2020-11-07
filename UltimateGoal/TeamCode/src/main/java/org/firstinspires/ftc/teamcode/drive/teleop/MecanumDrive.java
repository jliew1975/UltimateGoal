package org.firstinspires.ftc.teamcode.drive.teleop;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.BNO055IMUImpl;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.util.HardwareUtils;
import org.firstinspires.ftc.teamcode.util.MotorUtils;
import org.firstinspires.ftc.teamcode.util.OpModeUtils;
import org.firstinspires.ftc.teamcode.util.ThreadUtils;

import java.io.File;
import java.util.Arrays;
import java.util.HashSet;
import java.util.List;

import lombok.Data;

import static org.firstinspires.ftc.teamcode.util.OpModeUtils.opModeIsActive;


@Data
public class MecanumDrive {
    public DcMotor leftFront = null;
    public DcMotor rightFront = null;
    public DcMotor leftRear = null;
    public DcMotor rightRear = null;

    protected List<DcMotor> driveMotorList;

    protected BNO055IMUImpl imu = null;

    protected double worldAngle = 0d; // accumulated angle init angle
    protected double globalAngle = 0d;

    protected Orientation lastAngles = new Orientation();
    protected Orientation angleFacingAudience;

    protected Telemetry telemetry;
    protected ElapsedTime runtime = new ElapsedTime();

    protected double rotation;

    public void init() {
        // Do all the initial stuff
        HardwareMap hardwareMap = OpModeUtils.getHardwareMap();
        leftFront = HardwareUtils.get("leftFront", hardwareMap);
        rightFront = HardwareUtils.get("rightFront", hardwareMap);
        leftRear = HardwareUtils.get("leftRear", hardwareMap);
        rightRear = HardwareUtils.get("rightRear", hardwareMap);

        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftRear.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront.setDirection(DcMotorSimple.Direction.FORWARD);
        rightRear.setDirection(DcMotorSimple.Direction.FORWARD);

        driveMotorList = Arrays.asList(leftFront, rightFront, leftRear, rightRear);

        if(OpModeUtils.isResetEncoder()) {
            MotorUtils.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER, driveMotorList);
        }

        MotorUtils.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER, driveMotorList);
        MotorUtils.setZeroPowerMode(DcMotor.ZeroPowerBehavior.BRAKE, driveMotorList);

        telemetry = OpModeUtils.getOpMode().telemetry;
    }

    public void init_imu() {
        imu = OpModeUtils.getHardwareMap().get(BNO055IMUImpl.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.loggingEnabled = false;   //For debugging
        imu.initialize(parameters);

        while (opModeIsActive() && !imu.isGyroCalibrated()) {
            ThreadUtils.idle();
        }

        imu.startAccelerationIntegration(new Position(), new Velocity(), 500);

        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
        angleFacingAudience = new Orientation();

        File initAngleFile = AppUtil.getInstance().getSettingsFile("InitAngle.txt");
        angleFacingAudience.firstAngle = Float.parseFloat(ReadWriteFile.readFile(initAngleFile));

        telemetry.addData("imu", "finish imu calabration");
        telemetry.addData("imu calib status", imu.getCalibrationStatus().toString());
        telemetry.update();
    }

    public Position getPosition() {
        return imu.getPosition();
    }

    // TeleOp Drive APIs
    public void navigateWithGamepad(Gamepad gamepad) {
        double r = Math.hypot(gamepad.left_stick_x, gamepad.left_stick_y);
        double robotAngle = Math.atan2(gamepad.left_stick_x, gamepad.left_stick_y) - Math.PI / 4;
        double rightX = gamepad.right_stick_x;

        final double v1 = r * Math.cos(robotAngle) + rightX;
        final double v2 = r * Math.sin(robotAngle) - rightX;
        final double v3 = r * Math.sin(robotAngle) + rightX;
        final double v4 = r * Math.cos(robotAngle) - rightX;

        double powerMultiplier = 1.0;
        if(gamepad.right_trigger > 0) {
            powerMultiplier = 0.3;
        }

        double powerV1 = v1 * powerMultiplier;
        double powerV2 = v2 * powerMultiplier;
        double powerV3 = v3 * powerMultiplier;
        double powerV4 = v4 * powerMultiplier;

        if(gamepad.right_bumper) {
            powerV2 = 0d;
            powerV4 = 0d;
        } else if(gamepad.left_bumper) {
            powerV1 = 0d;
            powerV3 = 0d;
        }

        leftFront.setPower(powerV1);
        rightFront.setPower(powerV2);
        leftRear.setPower(powerV3);
        rightRear.setPower(powerV4);
    }

    public void stop() {
        driveMotorList.parallelStream().forEach(motor -> {
            motor.setPower(0);
        });
    }

    public void resetAngle() {
        resetAngle(AngleUnit.RADIANS);
    }

    public void resetAngle(AngleUnit angleUnit) {
        globalAngle = 0;
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, angleUnit);
    }

    public double getAngle() {
        return getAngle(AngleUnit.RADIANS);
    }

    public double getAngle(AngleUnit angleUnit)
    {
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, angleUnit);

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;
        if (deltaAngle < -Math.PI) {
            deltaAngle += (2 * Math.PI);
        } else if (deltaAngle > Math.PI) {
            deltaAngle -= (2 * Math.PI);
        }

        globalAngle += deltaAngle;
        lastAngles = angles;

        return globalAngle;
    }
}