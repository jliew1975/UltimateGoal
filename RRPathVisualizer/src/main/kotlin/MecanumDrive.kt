import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.path.Path
import com.acmerobotics.roadrunner.trajectory.Trajectory
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder
import com.acmerobotics.roadrunner.trajectory.constraints.DriveConstraints
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumConstraints
import com.acmerobotics.roadrunner.util.Angle
import kotlin.math.PI

class MecanumDrive(private val driveConstraints: DriveConstraints, private val trackWidth: Double) {
    private val combinedConstraints: MecanumConstraints = MecanumConstraints(driveConstraints, trackWidth)

    fun trajectoryBuilder(startPose: Pose2d): TrajectoryBuilder {
        return TrajectoryBuilder(startPose, false, combinedConstraints)
    }

    fun trajectoryBuilder(startPose: Pose2d, reverse: Boolean): TrajectoryBuilder {
        return TrajectoryBuilder(startPose, reverse, combinedConstraints)
    }

    fun trajectoryBuilder(startPose: Pose2d, endTangent: Double): TrajectoryBuilder {
        return TrajectoryBuilder(startPose, endTangent, combinedConstraints)
    }

    fun turn(currPose: Pose2d, angle: Double): Trajectory {
        var startPose = currPose
        val endPose = Pose2d(currPose.x + 1.0, currPose.y, currPose.heading + angle)
        if(angle < 1) {
            startPose = Pose2d(currPose.x, currPose.y, currPose.heading + (-15.0).toRadians)
        }

        return trajectoryBuilder(startPose).lineToLinearHeading(endPose).build()
    }
}