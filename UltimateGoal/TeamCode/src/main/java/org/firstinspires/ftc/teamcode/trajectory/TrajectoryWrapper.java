package org.firstinspires.ftc.teamcode.trajectory;

import com.acmerobotics.roadrunner.trajectory.Trajectory;

import lombok.Data;

@Data
public class TrajectoryWrapper {
    public enum TrajectoryType { Path, Turn }
    private Trajectory trajectory;
    private TrajectoryType trajectoryType;
    private Double turnAngle;

    public TrajectoryWrapper(Trajectory trajectory) {
        this.trajectory = trajectory;
        this.trajectoryType = TrajectoryType.Path;
    }

    public TrajectoryWrapper(double turnAngle) {
        this.turnAngle = Math.toRadians(turnAngle);
        this.trajectoryType = TrajectoryType.Turn;
    }

    public <T> T get() {
        if(trajectoryType == TrajectoryType.Path) {
            return (T) trajectory;
        }

        return (T) turnAngle;
    }
}
