package org.firstinspires.ftc.teamcode.team12538.TurnTableApp;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Created by Emily Liew on 8/22/18.
 */

public class TurnTable {
    public enum Direction { ClockWise, CounterClockWise }

    private double maxPower;
    private DcMotor motor;

    public TurnTable(double maxPower) {
        this.maxPower = maxPower;
    }

    public void init(HardwareMap hMap) {
        this.motor = hMap.get(DcMotor.class, "turnTableMotor");
    }

    public void start(double power, Direction direction) {
        if(power > maxPower) {
            power = maxPower;
        }

        if(direction == Direction.ClockWise) {
            motor.setDirection(DcMotorSimple.Direction.FORWARD);
        } else {
            motor.setDirection(DcMotorSimple.Direction.REVERSE);
        }

        while(Math.abs(motor.getPower()) < power) {
            motor.setPower(motor.getPower() + 0.01);
        }
    }

    public void increasePower() {
        double power = motor.getPower() + 0.01;

        if(power > maxPower) {
            power = maxPower;
        }

        motor.setPower(power);
    }

    public void decreasePower() {
        double power = motor.getPower();

        if(power > 0d) {
            power -= 0.01;
        }

        motor.setPower(power);
    }

    public void stop() {
        double currentPower = motor.getPower();

        while(currentPower > 0d) {
            currentPower -= 0.01;
            motor.setPower(currentPower);
        }
    }

    public void print(Telemetry telemetry) {
        telemetry.addData("Motor Power: ", motor.getPower());
    }
}
