package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.Map;
import java.util.TreeMap;

public class ShooterUtils {
    private static Map<Double, Double> presetLevel = new TreeMap<>();

    private static final double ENCODER_TICKS_PER_REV = 29d;

    static {
        presetLevel.put(7.5, 42.28);
        presetLevel.put(15.0, 48.1);
        presetLevel.put(22.5, 54.14);
        presetLevel.put(30.0, 60.38);
        presetLevel.put(37.5, 66.84);
        presetLevel.put(45.0, 73.5);
        presetLevel.put(52.5, 80.37);
        presetLevel.put(60.0, 87.45);
        presetLevel.put(67.5, 94.73);
        presetLevel.put(75.0, 102.22);
    }

    public static double getShooterServoAngle(Pose2d towerPose) {
        double shooterAngleDeg = Math.toDegrees(calculateHighGoalAngle(towerPose));
        double l_angle = 0d, ls_angle = 0d, h_angle = 0d, hs_angle = 0d;
        for(Map.Entry<Double, Double> entry : presetLevel.entrySet()) {
            if(shooterAngleDeg >= entry.getKey() &&  shooterAngleDeg <= entry.getKey() + 7.5) {
                l_angle = entry.getKey();
                ls_angle = entry.getValue();
                h_angle = entry.getKey() + 7.5;
                hs_angle = presetLevel.get(h_angle);
                break;
            }
        }
        double theta = (hs_angle - ls_angle)/7.5 * (shooterAngleDeg - l_angle) + ls_angle;
        return 0.152 / 54.0 * (theta - 36) + 0.349;
    }

    public static double calculateHighGoalAngle(Pose2d towerPose) {
        Pose2d currPose = OpModeUtils.getGlobalStore().getDrive().getPoseEstimate();
        double hDist = toMeter(
                Math.sqrt(
                        Math.pow(towerPose.getX() - currPose.getX(), 2) +
                                Math.pow(towerPose.getY() - currPose.getY(), 2)));

        Telemetry telemetry = OpModeUtils.getTelemetry();
        telemetry.addData("hDist", (hDist * 39.37));

        double vDist = toMeter(35);
        double yVelocity = Math.sqrt(19.6 * vDist);
        double xVelocity = hDist/(Math.sqrt(vDist/4.9));

        return Math.atan(yVelocity/xVelocity);
    }

    public static double calculateShooterVelocity(Pose2d towerPose) {
        Pose2d currPose = OpModeUtils.getGlobalStore().getDrive().getPoseEstimate();
        double hDist = toMeter(
                Math.sqrt(
                        Math.pow(towerPose.getX() - currPose.getX(), 2) +
                                Math.pow(towerPose.getY() - currPose.getY(), 2)));

        Telemetry telemetry = OpModeUtils.getTelemetry();
        telemetry.addData("hDist", (hDist * 39.37));

        double vDist = toMeter(35);
        double yVelocity = Math.sqrt(19.6 * vDist);
        double xVelocity = hDist/(Math.sqrt(vDist/4.9));

        return Math.sqrt(Math.pow(xVelocity,2) + Math.pow(yVelocity, 2)) *  2/0.010668;
    }



    private static double toMeter(double inputInInches) {
        return inputInInches/39.37;
    }
}
