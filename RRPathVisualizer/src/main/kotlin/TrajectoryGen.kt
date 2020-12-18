import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.trajectory.Trajectory
import com.acmerobotics.roadrunner.trajectory.constraints.DriveConstraints
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumConstraints


object TrajectoryGen {
    // Remember to set these constraints to the same values as your DriveConstants.java file in the quickstart
    private val driveConstraints = DriveConstraints(60.0, 60.0, 0.0, 180.0.toRadians, 180.0.toRadians, 0.0)

    // Remember to set your track width to an estimate of your actual bot to get accurate trajectory profile duration!
    private const val trackWidth = 13.2

    private val combinedConstraints = MecanumConstraints(driveConstraints, trackWidth)

    private val globalStore: GlobalStore = GlobalStore()

    fun createTrajectory(): ArrayList<Trajectory> {
        val list = ArrayList<Trajectory>()

        globalStore.wobbleCount = 2

        // list.addAll(createRedLeftNone())
        // list.addAll(createRedLeftOne())
        // list.addAll(createRedLeftFour())

        // list.addAll(createRedRightNone())
        // list.addAll(createRedRightOne())
        // list.addAll(createRedRightFour())

        // list.addAll(createBlueLeftNone())
        // list.addAll(createBlueLeftOne())
        // list.addAll(createBlueLeftFour())

        // list.addAll(createBlueRightNone())
        // list.addAll(createBlueRightOne())
        list.addAll(createBlueRightFour())

        return list
    }

    fun createRedLeftNone(): ArrayList<Trajectory> {
        val list = ArrayList<Trajectory>()

        val startPose = Pose2d(-63.0, -26.0, 180.0.toRadians)
        val drive = MecanumDrive(driveConstraints, trackWidth)

        val toZone1: Trajectory = drive.trajectoryBuilder(startPose, true)
            .splineToSplineHeading(Pose2d(20.0, -40.0, Math.toRadians(90.0)), Math.toRadians(-90.0))
            .build()

        val forward1: Trajectory = drive.trajectoryBuilder(toZone1.end())
            .forward(10.0)
            .build()

        val toPickupWobble: Trajectory = drive.trajectoryBuilder(forward1.end(), Math.toRadians(90.0))
            .splineToLinearHeading(Pose2d(-33.0, -53.0, 0.0), Math.toRadians(180.0))
            .build()

        val turn = drive.turn(toPickupWobble.end(), (-180.0).toRadians)

        val toZone2: Trajectory =
            drive.trajectoryBuilder(turn.end(), true)
                .splineToLinearHeading(Pose2d(-5.0, -55.0, Math.toRadians(180.0)), Math.toRadians(-15.0))
                .build()

        val forward2: Trajectory = drive.trajectoryBuilder(toZone2.end())
            .forward(10.0)
            .build()

        val toLaunchZone: Trajectory =
            drive.trajectoryBuilder(if (globalStore.wobbleCount > 1) forward2.end() else forward1.end())
                .lineToLinearHeading(Pose2d(-5.0, -40.0, 0.0))
                .build()

        val toParking: Trajectory = drive.trajectoryBuilder(toLaunchZone.end())
            .lineToLinearHeading(Pose2d(10.0, -40.0, 0.0))
            .build()

        list.add(toZone1)
        list.add(forward1)
        if(globalStore.wobbleCount > 1) {
            list.add(toPickupWobble)
            list.add(turn)
            list.add(toZone2)
            list.add(forward2)
        }
        list.add(toLaunchZone)
        list.add(toParking)

        return list
    }

    fun createRedLeftOne(): ArrayList<Trajectory> {
        val list = ArrayList<Trajectory>()

        val startPose = Pose2d(-63.0, -26.0, 180.0.toRadians)
        val drive = MecanumDrive(driveConstraints, trackWidth)

        val toZone1: Trajectory = drive.trajectoryBuilder(startPose, true)
            .splineToSplineHeading(Pose2d(30.0, -24.0, Math.toRadians(145.0)), Math.toRadians(-35.0))
            .build()

        val forward1: Trajectory = drive.trajectoryBuilder(toZone1.end())
            .forward(10.0)
            .build()

        val toPickupWobble: Trajectory = drive.trajectoryBuilder(forward1.end(), true)
            .splineToSplineHeading(Pose2d(-33.5, -53.0, Math.toRadians(-5.0)), Math.toRadians(180.0))
            .build()

        val turn = drive.turn(toPickupWobble.end(), (-175.0).toRadians)

        val toZone2: Trajectory = drive.trajectoryBuilder(turn.end(), true)
            .splineToSplineHeading(Pose2d(25.0, -48.0, Math.toRadians(-130.0)), 20.0.toRadians)
            .build()

        val forward2: Trajectory = drive.trajectoryBuilder(toZone2.end())
            .forward(10.0)
            .build()

        val toLaunchZone: Trajectory =
            drive.trajectoryBuilder(if (globalStore.wobbleCount > 1) forward2.end() else forward1.end())
                .lineToLinearHeading(Pose2d(-5.0, -40.0, 0.0))
                .build()

        val toParking: Trajectory = drive.trajectoryBuilder(toLaunchZone.end())
            .lineToLinearHeading(Pose2d(10.0, -40.0, 0.0))
            .build()

        list.add(toZone1)
        list.add(forward1)
        if(globalStore.wobbleCount > 1) {
            list.add(toPickupWobble)
            list.add(turn)
            list.add(toZone2)
            list.add(forward2)
        }
        list.add(toLaunchZone)
        list.add(toParking)

        return list
    }

    fun createRedLeftFour(): ArrayList<Trajectory> {
        val list = ArrayList<Trajectory>()

        val startPose = Pose2d(-63.0, -26.0, 180.0.toRadians)
        val drive = MecanumDrive(driveConstraints, trackWidth)

        val toZone1 = drive.trajectoryBuilder(startPose, true)
            .splineToSplineHeading(Pose2d(60.0, -40.0, Math.toRadians(100.0)), Math.toRadians(-70.0))
            .build()

        val forward1 = drive.trajectoryBuilder(toZone1.end())
            .forward(10.0)
            .build()

        val toPickupWobble = drive.trajectoryBuilder(forward1.end())
            .splineToLinearHeading(Pose2d(-33.0, -51.0, (-15.0).toRadians), Math.toRadians(180.0))
            .build()

        val turn = drive.turn(toPickupWobble.end(), (-175.0).toRadians)

        val toZone2 = drive.trajectoryBuilder(turn.end(), true)
            .lineToLinearHeading(Pose2d(40.0, -58.0, Math.toRadians(180.0)))
            .build()

        val forward2 = drive.trajectoryBuilder(toZone2.end())
            .forward(10.0)
            .build()

        val toLaunchZone = drive.trajectoryBuilder(if(globalStore.wobbleCount > 1) forward2.end() else forward1.end())
            .lineToLinearHeading(Pose2d(-5.0, -40.0, 0.0))
            .build()

        val toParking = drive.trajectoryBuilder(toLaunchZone.end())
            .lineToLinearHeading(Pose2d(10.0, -40.0, 0.0))
            .build()

        list.add(toZone1)
        list.add(forward1)
        if(globalStore.wobbleCount > 1) {
            list.add(toPickupWobble)
            list.add(turn)
            list.add(toZone2)
            list.add(forward2)
        }
        list.add(toLaunchZone)
        list.add(toParking)

        return list
    }

    fun createRedRightNone(): ArrayList<Trajectory> {
        val list = ArrayList<Trajectory>()

        val startPose = Pose2d(-63.0, -50.0, 180.0.toRadians)
        val drive = MecanumDrive(driveConstraints, trackWidth)

        val toZone1 = drive.trajectoryBuilder(startPose, true)
            .splineToSplineHeading(Pose2d(5.0, -55.0, Math.toRadians(180.0)), 0.0)
            .build()

        val forward1 = drive.trajectoryBuilder(toZone1.end(), Math.toRadians(90.0))
            .forward(if(globalStore.wobbleCount > 1) 35.0 else 10.0)
            .build()

        val turn1 = drive.trajectoryBuilder(forward1.end())
            .lineToLinearHeading(Pose2d(-35.0, -55.0, Math.toRadians(-55.0)))
            .build()

        val toPickupWobble = drive.trajectoryBuilder(turn1.end())
            .back(20.0)
            .build()

        val reverse = drive.trajectoryBuilder(toPickupWobble.end())
            .forward(20.0)
            .build()

        val toZone2 = drive.trajectoryBuilder(reverse.end(), true)
            .lineToLinearHeading(Pose2d(-5.0, -55.0, Math.toRadians(180.0)))
            .build()

        val forward2 = drive.trajectoryBuilder(toZone2.end())
            .forward(10.0)
            .build()

        val toLaunchZone = drive.trajectoryBuilder(if(globalStore.wobbleCount > 1) forward2.end() else forward1.end())
            .lineToLinearHeading(Pose2d(-5.0, -40.0, 0.0))
            .build()

        var toParking = drive.trajectoryBuilder(toLaunchZone.end())
            .lineToLinearHeading(Pose2d(10.0, -40.0, 0.0))
            .build()

        list.add(toZone1)
        list.add(forward1)
        if(globalStore.wobbleCount > 1) {
            list.add(turn1)
            list.add(toPickupWobble)
            list.add(reverse)
            list.add(toZone2)
            list.add(forward2)
        }
        list.add(toLaunchZone)
        list.add(toParking)

        return list
    }

    fun createRedRightOne(): ArrayList<Trajectory> {
        val list = ArrayList<Trajectory>()

        val startPose = Pose2d(-63.0, -50.0, 180.0.toRadians)
        val drive = MecanumDrive(driveConstraints, trackWidth)

        val toZone1 = drive.trajectoryBuilder(startPose, true)
            .splineToSplineHeading(Pose2d(25.0, -45.0, Math.toRadians(-130.0)), Math.toRadians(35.0))
            .build()

        val forward1 = drive.trajectoryBuilder(toZone1.end())
            .forward(10.0)
            .build()

        val turn1 = drive.trajectoryBuilder(forward1.end())
            .lineToLinearHeading(Pose2d(-33.0, -55.0, Math.toRadians(-55.0)))
            .build()

        val toPickupWobble = drive.trajectoryBuilder(turn1.end())
            .back(20.0)
            .build()

        val turn2 = drive.trajectoryBuilder(toPickupWobble.end())
            .lineToLinearHeading(Pose2d(-40.0, -30.0, Math.toRadians(-120.0)))
            .build()

        val toZone2 = drive.trajectoryBuilder(turn2.end(), Math.toRadians(30.0))
            .splineToLinearHeading(Pose2d(30.0, -20.0, Math.toRadians(130.0)), Math.toRadians(-30.0))
            .build()

        val forward2 = drive.trajectoryBuilder(toZone2.end())
            .forward(10.0)
            .build()

        val toLaunchZone = drive.trajectoryBuilder(if(globalStore.wobbleCount > 1) forward2.end() else forward1.end())
            .lineToLinearHeading(Pose2d(-5.0, -40.0, 0.0))
            .build()

        val toParking = drive.trajectoryBuilder(toLaunchZone.end())
            .lineToLinearHeading(Pose2d(10.0, -40.0, 0.0))
            .build()

        list.add(toZone1)
        list.add(forward1)
        if(globalStore.wobbleCount > 1) {
            list.add(turn1)
            list.add(toPickupWobble)
            list.add(turn2)
            list.add(toZone2)
            list.add(forward2)
        }
        list.add(toLaunchZone)
        list.add(toParking)

        return list
    }

    fun createRedRightFour(): ArrayList<Trajectory> {
        val list = ArrayList<Trajectory>()

        val startPose = Pose2d(-63.0, -50.0, 180.0.toRadians)
        val drive = MecanumDrive(driveConstraints, trackWidth)

        val toZone1 = drive.trajectoryBuilder(startPose, true)
            .splineToSplineHeading(Pose2d(53.0, -55.0, Math.toRadians(180.0)), 0.0)
            .build()

        val forward1 = drive.trajectoryBuilder(toZone1.end())
            .forward(if(globalStore.wobbleCount > 1) 80.0 else 10.0)
            .build()

        val turn1 = drive.trajectoryBuilder(forward1.end())
            .lineToLinearHeading(Pose2d(-33.0, -55.0, Math.toRadians(-55.0)))
            .build()

        val toPickupWobble = drive.trajectoryBuilder(turn1.end())
            .back(20.0)
            .build()

        val forward2 = drive.trajectoryBuilder(toPickupWobble.end())
            .forward(20.0)
            .build()

        val turn2 = drive.trajectoryBuilder(forward2.end())
            .lineToLinearHeading(Pose2d(-28.0, -55.0, Math.toRadians(180.0)))
            .build()

        val toZone2 = drive.trajectoryBuilder(turn2.end(), true)
            .lineToLinearHeading(Pose2d(40.0, -58.0, Math.toRadians(180.0)))
            .build()

        val forward3 = drive.trajectoryBuilder(toZone2.end())
            .forward(10.0)
            .build()

        val toLaunchZone = drive.trajectoryBuilder(if(globalStore.wobbleCount > 1) forward3.end() else forward1.end())
            .lineToLinearHeading(Pose2d(-5.0, -40.0, 0.0))
            .build()

        val toParking = drive.trajectoryBuilder(toLaunchZone.end())
            .lineToLinearHeading(Pose2d(10.0, -40.0, 0.0))
            .build()

        list.add(toZone1)
        list.add(forward1)
        if(globalStore.wobbleCount > 1) {
            list.add(turn1)
            list.add(toPickupWobble)
            list.add(forward2)
            list.add(turn2)
            list.add(toZone2)
            list.add(forward3)
        }
        list.add(toLaunchZone)
        list.add(toParking)

        return list
    }

    fun createBlueLeftNone(): ArrayList<Trajectory> {
        val list = ArrayList<Trajectory>()

        val startPose = Pose2d(-63.0, 50.0, 180.0.toRadians)
        val drive = MecanumDrive(driveConstraints, trackWidth)

        val toZone1 = drive.trajectoryBuilder(startPose, true)
            .lineToLinearHeading(Pose2d(5.0, 60.0, Math.toRadians(180.0)))
            .build()

        val forward1 = drive.trajectoryBuilder(toZone1.end(), false)
            .forward(10.0)
            .build()

        val turn1 = drive.trajectoryBuilder(forward1.end(), false)
            .lineToLinearHeading(Pose2d(-10.0, 45.0, Math.toRadians(15.0)))
            .build()

        val toPickupWobble = drive.trajectoryBuilder(turn1.end(), false)
            .lineToLinearHeading(Pose2d(-38.0, 28.0, 0.0))
            .build()

        val toZone2 = drive.trajectoryBuilder(toPickupWobble.end(), Math.toRadians(90.0))
            .splineToLinearHeading(Pose2d(-5.0, 60.0, Math.toRadians(180.0)), Math.toRadians(0.0))
            .build()

        val forward2 = drive.trajectoryBuilder(toZone2.end())
            .forward(10.0)
            .build()

        val toLaunchZone =
            drive.trajectoryBuilder(if(globalStore.wobbleCount > 1) forward2.end() else forward1.end())
                .lineToLinearHeading(Pose2d(-5.0, 40.0, 0.0))
                .build()

        val toParking = drive.trajectoryBuilder(toLaunchZone.end())
            .lineToLinearHeading(Pose2d(10.0, 40.0, 0.0))
            .build()

        list.add(toZone1)
        list.add(forward1)
        if(globalStore.wobbleCount > 1) {
            list.add(turn1)
            list.add(toPickupWobble)
            list.add(toZone2)
            list.add(forward2)
        }
        list.add(toLaunchZone)
        list.add(toParking)

        return list
    }

    fun createBlueLeftOne(): ArrayList<Trajectory> {
        val list = ArrayList<Trajectory>()

        val startPose = Pose2d(-63.0, 50.0, 180.0.toRadians)
        val drive = MecanumDrive(driveConstraints, trackWidth)

        val toZone1 = drive.trajectoryBuilder(startPose, true)
            .splineToSplineHeading(Pose2d(30.0, 50.0, Math.toRadians(160.0)), Math.toRadians(-10.0))
            .build()

        val forward1 = drive.trajectoryBuilder(toZone1.end())
            .forward(10.0)
            .build()

        val turn1 = drive.trajectoryBuilder(forward1.end(), false)
            .lineToLinearHeading(Pose2d(15.0, 30.0, Math.toRadians(0.0)))
            .build()

        val toPickupWobble = drive.trajectoryBuilder(turn1.end(), Math.toRadians(180.0))
            .splineToLinearHeading(Pose2d(-35.0, 23.0, Math.toRadians(-15.0)), Math.toRadians(140.0))
            .build()

        val reverse = drive.trajectoryBuilder(toPickupWobble.end(), false)
            .lineToLinearHeading(Pose2d(-25.0, 23.0, Math.toRadians(0.0)))
            .build()

        val toZone2 = drive.trajectoryBuilder(reverse.end(), false)
            .splineToSplineHeading(Pose2d(18.0, 38.0, Math.toRadians(180.0)), 0.0)
            .build()

        val forward2 = drive.trajectoryBuilder(toZone2.end())
            .forward(10.0)
            .build()

        val toLaunchZone = drive.trajectoryBuilder(if(globalStore.wobbleCount > 1) forward2.end() else forward1.end())
            .lineToLinearHeading(Pose2d(-5.0, 40.0, 0.0))
            .build()

        val toParking = drive.trajectoryBuilder(toLaunchZone.end())
            .lineToLinearHeading(Pose2d(10.0, 40.0, 0.0))
            .build()

        list.add(toZone1)
        list.add(forward1)
        if(globalStore.wobbleCount > 1) {
            list.add(turn1)
            list.add(toPickupWobble)
            list.add(reverse)
            list.add(toZone2)
            list.add(forward2)
        }
        list.add(toLaunchZone)
        list.add(toParking)

        return list
    }

    fun createBlueLeftFour(): ArrayList<Trajectory> {
        val list = ArrayList<Trajectory>()

        val startPose = Pose2d(-63.0, 50.0, 180.0.toRadians)
        val drive = MecanumDrive(driveConstraints, trackWidth)

        val toZone1 = drive.trajectoryBuilder(startPose, true)
            .splineToSplineHeading(Pose2d(50.0, 58.0, Math.toRadians(-150.0)), Math.toRadians(10.0))
            .build()

        val forward1 = drive.trajectoryBuilder(toZone1.end())
            .forward(10.0)
            .build()

        val turn1 = drive.trajectoryBuilder(forward1.end())
            .lineToLinearHeading(Pose2d(15.0, 20.0, Math.toRadians(0.0)))
            .build()

        val toPickupWobble = drive.trajectoryBuilder(turn1.end())
            .lineToLinearHeading(Pose2d(-35.0, 23.0, Math.toRadians(-15.0)))
            .build()

        val turn2 = drive.trajectoryBuilder(toPickupWobble.end())
            .lineToLinearHeading(Pose2d(-34.0, 23.0, Math.toRadians(180.0)))
            .build()

        val toZone2 = drive.trajectoryBuilder(turn2.end(), true)
            .splineToLinearHeading(Pose2d(48.0, 48.0, Math.toRadians(-110.0)), Math.toRadians(30.0))
            .build()

        val forward2 = drive.trajectoryBuilder(toZone2.end())
            .forward(10.0)
            .build()

        val toLaunchZone =
            drive.trajectoryBuilder(if(globalStore.wobbleCount > 1) forward2.end() else forward1.end())
                .lineToLinearHeading(Pose2d(-5.0, 40.0, 0.0))
                .build()

        val toParking = drive.trajectoryBuilder(toLaunchZone.end())
            .lineToLinearHeading(Pose2d(10.0, 40.0, 0.0))
            .build()

        list.add(toZone1)
        list.add(forward1)
        if(globalStore.wobbleCount > 1) {
            list.add(turn1)
            list.add(toPickupWobble)
            list.add(turn2)
            list.add(toZone2)
            list.add(forward2)
        }
        list.add(toLaunchZone)
        list.add(toParking)

        return list
    }

    fun createBlueRightNone(): ArrayList<Trajectory> {
        val list = ArrayList<Trajectory>()

        val startPose = Pose2d(-63.0, 26.0, 180.0.toRadians)
        val drive = MecanumDrive(driveConstraints, trackWidth)

        val toZone1 = drive.trajectoryBuilder(startPose, true)
            .splineToSplineHeading(Pose2d(15.0, 45.0, Math.toRadians(-90.0)), Math.toRadians(90.0))
            .build()

        val forward1 = drive.trajectoryBuilder(toZone1.end(), false)
            .forward(10.0)
            .build()

        val toPickupWobble = drive.trajectoryBuilder(forward1.end(), false)
            .lineToLinearHeading(Pose2d(-35.0, 45.0, 0.0))
            .build()

        val toZone2 = drive.trajectoryBuilder(toPickupWobble.end())
            .splineToLinearHeading(Pose2d(-5.0, 59.0, Math.toRadians(-170.0)), Math.toRadians(0.0))
            .build()

        val forward2 = drive.trajectoryBuilder(toZone2.end())
            .forward(10.0)
            .build()

        val toLaunchZone = drive.trajectoryBuilder(if(globalStore.wobbleCount > 1) forward2.end() else forward1.end())
            .lineToLinearHeading(Pose2d(-5.0, 40.0, 0.0))
            .build()

        val toParking = drive.trajectoryBuilder(toLaunchZone.end())
            .lineToLinearHeading(Pose2d(10.0, 40.0, 0.0))
            .build()

        list.add(toZone1)
        list.add(forward1)
        if(globalStore.wobbleCount > 1) {
            list.add(toPickupWobble)
            list.add(toZone2)
            list.add(forward2)
        }
        list.add(toLaunchZone)
        list.add(toParking)

        return list
    }

    fun createBlueRightOne(): ArrayList<Trajectory> {
        val list = ArrayList<Trajectory>()

        val startPose = Pose2d(-63.0, 26.0, 180.0.toRadians)
        val drive = MecanumDrive(driveConstraints, trackWidth)

        val toZone1 = drive.trajectoryBuilder(startPose, true)
            .splineToSplineHeading(Pose2d(28.0, 30.0, Math.toRadians(-140.0)), Math.toRadians(40.0))
            .build()

        val forward1 = drive.trajectoryBuilder(toZone1.end())
            .forward(10.0)
            .build()

        val turn1 = drive.trajectoryBuilder(forward1.end())
            .lineToLinearHeading(Pose2d(-10.0, 55.0, Math.toRadians(0.0)))
            .build()

        val toPickupWobble = drive.trajectoryBuilder(turn1.end(), Math.toRadians(90.0))
            .lineToLinearHeading(Pose2d(-35.0, 50.0, Math.toRadians(15.0)))
            .build()

        val turn2 = drive.trajectoryBuilder(toPickupWobble.end(), Math.toRadians(40.0))
            .splineToLinearHeading(Pose2d(-5.0, 55.0, Math.toRadians(-180.0)), 0.0)
            .build()

        val toZone2 = drive.trajectoryBuilder(turn2.end(), false)
            .lineToLinearHeading(Pose2d(25.0, 50.0, Math.toRadians(140.0)))
            .build()

        val forward2 = drive.trajectoryBuilder(toZone2.end())
            .forward(10.0)
            .build()

        val toLaunchZone =
            drive.trajectoryBuilder(if(globalStore.wobbleCount > 1) forward2.end() else forward1.end())
                .lineToLinearHeading(Pose2d(-5.0, 40.0, 0.0))
                .build()

        val toParking = drive.trajectoryBuilder(toLaunchZone.end())
            .lineToLinearHeading(Pose2d(10.0, 40.0, 0.0))
            .build()

        list.add(toZone1)
        list.add(forward1)
        if(globalStore.wobbleCount > 1) {
            list.add(turn1)
            list.add(toPickupWobble)
            list.add(turn2)
            list.add(toZone2)
            list.add(forward2)
        }
        list.add(toLaunchZone)
        list.add(toParking)

        return list
    }

    fun createBlueRightFour(): ArrayList<Trajectory> {
        val list = ArrayList<Trajectory>()

        val startPose = Pose2d(-63.0, 26.0, 180.0.toRadians)
        val drive = MecanumDrive(driveConstraints, trackWidth)

        val toZone1 = drive.trajectoryBuilder(startPose, Math.toRadians(-20.0))
            .splineToSplineHeading(Pose2d(50.0, 55.0, Math.toRadians(-140.0)), Math.toRadians(30.0))
            .build()

        val forward1 = drive.trajectoryBuilder(toZone1.end())
            .forward(10.0)
            .build()

        val turn1 = drive.trajectoryBuilder(forward1.end())
            .lineToLinearHeading(Pose2d(20.0, 55.0, Math.toRadians(0.0)))
            .build()

        val toPickupWobble = drive.trajectoryBuilder(turn1.end(), Math.toRadians(90.0))
            .lineToLinearHeading(Pose2d(-35.0, 50.0, Math.toRadians(15.0)))
            .build()

        val turn2 = drive.trajectoryBuilder(toPickupWobble.end(), Math.toRadians(40.0))
            .splineToLinearHeading(Pose2d(0.0, 55.0, Math.toRadians(180.0)), 0.0)
            .build()

        val toZone2 = drive.trajectoryBuilder(turn2.end(), true)
            .lineToLinearHeading(Pose2d(40.0, 60.0, Math.toRadians(-170.0)))
            .build()

        val forward2 = drive.trajectoryBuilder(toZone2.end())
            .forward(10.0)
            .build()

        val toLaunchZone = drive.trajectoryBuilder(if(globalStore.wobbleCount > 1) forward2.end() else forward1.end())
            .lineToLinearHeading(Pose2d(-5.0, 40.0, 0.0))
            .build()

        val toParking = drive.trajectoryBuilder(toLaunchZone.end())
            .lineToLinearHeading(Pose2d(10.0, 40.0, 0.0))
            .build()

        list.add(toZone1)
        list.add(forward1)
        if(globalStore.wobbleCount > 1) {
            list.add(turn1)
            list.add(toPickupWobble)
            list.add(turn2)
            list.add(toZone2)
            list.add(forward2)
        }
        list.add(toLaunchZone)
        list.add(toParking)

        return list
    }

    fun drawOffbounds() {
        // GraphicsUtil.fillRect(Vector2d(0.0, -63.0), 18.0, 18.0) // robot against the wall
    }
}

val Double.toRadians get() = (Math.toRadians(this))
