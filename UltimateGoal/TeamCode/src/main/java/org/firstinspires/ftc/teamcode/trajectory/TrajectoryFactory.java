package org.firstinspires.ftc.teamcode.trajectory;

import org.firstinspires.ftc.teamcode.detectors.StarterRingsDetector.RingCount;
import org.firstinspires.ftc.teamcode.robot.AutoRobot;
import org.firstinspires.ftc.teamcode.util.AutoConstant;
import org.firstinspires.ftc.teamcode.util.AutonomousColor;
import org.firstinspires.ftc.teamcode.util.GlobalStorage;

import java.util.HashMap;
import java.util.List;
import java.util.Map;

import lombok.Data;

@Data
public class TrajectoryFactory {
    Map<RingCount, List<TrajectoryWrapper>> trajectoryMap = new HashMap<>();

    public void init(AutoRobot robot) {
        if(GlobalStorage.autoColor == AutonomousColor.Red) {
            if(GlobalStorage.side == AutoConstant.Side.Left) {
                trajectoryMap.putAll(TrajectoryRedLeftFactory.generateTrajectories(robot));
            } else {
                trajectoryMap.putAll(TrajectoryRedRightFactory.generateTrajectories(robot));
            }
        } else {
            if(GlobalStorage.side == AutoConstant.Side.Left) {
                trajectoryMap.putAll(TrajectoryBlueLeftFactory.generateTrajectories(robot));
            } else {
                trajectoryMap.putAll(TrajectoryBlueRightFactory.generateTrajectories(robot));
            }
        }
    }

    public List<TrajectoryWrapper> getTrajectories(RingCount ringCount) {
        return trajectoryMap.get(ringCount);
    }
}