package org.firstinspires.ftc.teamcode.team12538.components;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.team12538.utils.MotorUtils;
import org.firstinspires.ftc.teamcode.team12538.utils.OpModeUtils;
import org.firstinspires.ftc.teamcode.team12538.utils.ThreadUtils;

import lombok.Data;

@Data
public class RobotLatch implements RobotMechanic {
    private DcMotor liftMotor;
    private volatile boolean liftBusy = false;

    private ElapsedTime runtime = new ElapsedTime();

    private int latchPosition = 6900;
    private int unlatchPosition = 0;

    public void init() {
        HardwareMap hardwareMap = OpModeUtils.getGlobalStore().getHardwareMap();

        liftMotor = hardwareMap.get(DcMotor.class, "jack");
        MotorUtils.setZeroPowerMode(DcMotor.ZeroPowerBehavior.BRAKE, liftMotor);

        if(liftMotor.getCurrentPosition() != 0) {
            powerLiftRunToPosition(1.0, 0);
        }

        MotorUtils.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER, liftMotor);
        MotorUtils.setMode(DcMotor.RunMode.RUN_USING_ENCODER, liftMotor);
    }

    public void powerLift(double power) {
        powerLift(power, -1);
    }

    public void powerLift(double power, int constraint) {
        synchronized (liftMotor) {
            if(liftBusy) {
                return;
            }

            if (constraint != -1) {
                int currentPos = liftMotor.getCurrentPosition();
                if (power > 0) {
                    power = 0;
                }
            }

            liftMotor.setPower(power);
        }
    }

    public void powerLiftRunToPosition(final double power, final int targetPosition) {
        synchronized (liftMotor) {
            try {
                liftBusy = true;
                MotorUtils.setMode(DcMotor.RunMode.RUN_TO_POSITION, liftMotor);

                double adjustedPower = power;

                int startPos = liftMotor.getCurrentPosition();

                liftMotor.setTargetPosition(targetPosition);
                liftMotor.setPower(adjustedPower);

                runtime.reset();
                int curPos = -1;
                while (OpModeUtils.opModeIsActive() && liftMotor.isBusy()) {
                    if(curPos == -1 && runtime.seconds() > 1) {
                        if(curPos != -1 && curPos == liftMotor.getCurrentPosition()) {
                            break;
                        }
                    }

                    curPos = liftMotor.getCurrentPosition();
                    runtime.reset();

                    if(targetPosition > 0 && curPos > (targetPosition - 500)) {
                        liftMotor.setPower(0.3);
                    } else if(targetPosition == 0 && curPos < (targetPosition + 500)) {
                        liftMotor.setPower(0.3);
                    }

                    Telemetry telemetry = OpModeUtils.getGlobalStore().getTelemetry();
                    telemetry.addData("lift", liftMotor.getCurrentPosition());
                    telemetry.update();
                }

                liftMotor.setPower(0d);
                MotorUtils.setMode(DcMotor.RunMode.RUN_USING_ENCODER, liftMotor);
            } finally {
                liftBusy = false;
            }
        }
    }

    public void printTelemetry() {
        Telemetry telemetry = OpModeUtils.getGlobalStore().getTelemetry();
        telemetry.addData("lift", liftMotor.getCurrentPosition());
    }
}
