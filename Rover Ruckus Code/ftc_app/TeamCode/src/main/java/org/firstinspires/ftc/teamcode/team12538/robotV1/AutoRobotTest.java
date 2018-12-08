package org.firstinspires.ftc.teamcode.team12538.robotV1;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.team12538.components.MineralMechanism;
import org.firstinspires.ftc.teamcode.team12538.detectors.GoldAlignDetectorExt;
import org.firstinspires.ftc.teamcode.team12538.drive.MecanumDriveBase;
import org.firstinspires.ftc.teamcode.team12538.utils.MotorUtils;
import org.firstinspires.ftc.teamcode.team12538.utils.OpModeUtils;

import java.util.List;

public class AutoRobotTest extends MecanumDriveBase {
    private final double COUNTS_PER_MOTOR_REV = 560; // andymark 20:1 gearbox
    private final double DRIVE_GEAR_REDUCTION = 1.0;  // drive train geared down 1:2
    private final double WHEEL_DIAMETER_INCHES = 4.0;
    private final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);


    Telemetry telemetry = null;
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void init() {
        super.init();
        telemetry = OpModeUtils.getGlobalStore().getTelemetry();
    }

    public void printMotorTelemetry() {
        if(telemetry == null) {
            return;
        }

        telemetry.addData("Motor Position", "Motor at %7d :%7d",
                motors.get(0).getCurrentPosition(),
                motors.get(1).getCurrentPosition(),
                motors.get(2).getCurrentPosition(),
                motors.get(3).getCurrentPosition());
        telemetry.update();
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

    private void encoderStrafe(StrafingDirection direction, double speed, double distance, double timeout, GoldAlignDetectorExt detector) {
        // reset encoders
        // MotorUtils.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER, motors);
        MotorUtils.setMode(DcMotor.RunMode.RUN_USING_ENCODER, motors);

        int[] targetPositions = new int[motors.size()];

        double newDistance = (direction == StrafingDirection.Right) ? -1 * distance : distance;

        //ensure that the opmode is still active
        if(OpModeUtils.getOpMode().opModeIsActive()) {
            int index = 0;
            for(DcMotor motor : motors) {
                // bleftDrive or frightDrive
                if(index == 1 || index == 2) {
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
}
