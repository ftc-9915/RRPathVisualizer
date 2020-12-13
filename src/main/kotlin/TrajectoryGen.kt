import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.acmerobotics.roadrunner.trajectory.Trajectory
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder
import com.acmerobotics.roadrunner.trajectory.constraints.DriveConstraints
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumConstraints

object TrajectoryGen {
    // Remember to set these constraints to the same values as your DriveConstants.java file in the quickstart
    private val driveConstraints = DriveConstraints(60.0, 60.0, 0.0, 270.0.toRadians, 270.0.toRadians, 0.0)

    // Remember to set your track width to an estimate of your actual bot to get accurate trajectory profile duration!
    private const val trackWidth = 16.0

    private val combinedConstraints = MecanumConstraints(driveConstraints, trackWidth)

    private val startPoseBlue1 = Pose2d(-63.0, 48.0, 0.0.toRadians)
    private val startPoseBlue2 = Pose2d(-48.0, -48.0, 90.0.toRadians)


    fun createTrajectory(): ArrayList<Trajectory> {
        var list = ArrayList<Trajectory>()


        //Declare Routes
        val blue1_RingFourRoute = ArrayList<Trajectory>();
        val blue1_RingOneRoute  = ArrayList<Trajectory>();
        val blue1_RingNoneRoute = ArrayList<Trajectory>();


        // -- Prose and Trajectory Library --

        val shootPoseBlue1 = Pose2d(0.0, 48.0, 0.0.toRadians)
        val shootPoseBlue1PowerShot1 = Pose2d(0.0, 48.0, -4.0.toRadians)
        val shootPoseBlue1PowerShot2 = Pose2d(0.0, 48.0, -8.0.toRadians)
        val shootPoseBlue1PowerShot3 = Pose2d(0.0, 48.0, -12.0.toRadians)
        val zoneCPoseBlue = Pose2d(48.0, 48.0, 20.0.toRadians)
        val ringPoseMiddleBlue1 = Pose2d(0.0, 40.0, 180.0.toRadians)
        val ringPoseBlue1 = Pose2d(-24.0, 36.0, 180.0.toRadians)
        val wobbleGoalBlue1 = Pose2d(-37.0, 32.0, 190.0.toRadians)
        val parkingPositionBlue1 = Pose2d(12.0, 42.0, 20.0.toRadians)

        //Version with power shot
        val blue1ToShootPowerShot = TrajectoryBuilder(startPoseBlue1, startPoseBlue1.heading, combinedConstraints)
        blue1ToShootPowerShot
            .splineTo(shootPoseBlue1PowerShot1.vec(), shootPoseBlue1PowerShot1.heading)

        val blue1PowerShotToZoneC = TrajectoryBuilder(shootPoseBlue1PowerShot3, shootPoseBlue1PowerShot3.heading, combinedConstraints)
        blue1PowerShotToZoneC
            .lineToSplineHeading(zoneCPoseBlue)

        //Version with high goal
        val blue1ToShootHighGoal = TrajectoryBuilder(startPoseBlue1, startPoseBlue1.heading, combinedConstraints)
        blue1ToShootHighGoal
            .splineTo(shootPoseBlue1.vec(), shootPoseBlue1.heading)

        val blue1HighGoalToZoneC = TrajectoryBuilder(shootPoseBlue1, shootPoseBlue1.heading, combinedConstraints)
        blue1HighGoalToZoneC
            .lineToSplineHeading(zoneCPoseBlue)

        val blue1ZoneCToRingWobbleGoal = TrajectoryBuilder(zoneCPoseBlue, zoneCPoseBlue.heading, combinedConstraints)
        blue1ZoneCToRingWobbleGoal
            .lineToSplineHeading(ringPoseMiddleBlue1)
            .lineToConstantHeading(ringPoseBlue1.vec())
            .splineTo(wobbleGoalBlue1.vec(), wobbleGoalBlue1.heading)

        val blue1WobbleGoalToShoot = TrajectoryBuilder(wobbleGoalBlue1, wobbleGoalBlue1.heading, combinedConstraints)
        blue1WobbleGoalToShoot
            .lineToSplineHeading(shootPoseBlue1)

        val blue1ShootToZoneC = TrajectoryBuilder(shootPoseBlue1, shootPoseBlue1.heading, combinedConstraints)
        blue1ShootToZoneC
            .lineToSplineHeading(zoneCPoseBlue)

        val blue1ZoneCToPark = TrajectoryBuilder(zoneCPoseBlue, zoneCPoseBlue.heading, combinedConstraints)
        blue1ZoneCToPark
            .lineToSplineHeading(parkingPositionBlue1)


        //Version with power shot
//        blue1_RingFourRoute.add(blue1ToShootPowerShot.build())
//        blue1_RingFourRoute.add(blue1PowerShotToZoneC.build())

        //Version with high goal
        blue1_RingFourRoute.add(blue1ToShootHighGoal.build())
        blue1_RingFourRoute.add(blue1HighGoalToZoneC.build())

        blue1_RingFourRoute.add(blue1ZoneCToRingWobbleGoal.build())
        blue1_RingFourRoute.add(blue1WobbleGoalToShoot.build())
        blue1_RingFourRoute.add(blue1ShootToZoneC.build())
        blue1_RingFourRoute.add(blue1ZoneCToPark.build())



        return blue1_RingFourRoute
    }

    fun drawOffbounds() {
        GraphicsUtil.fillRect(Vector2d(0.0, -63.0), 18.0, 18.0) // robot against the wall
    }
}

val Double.toRadians get() = (Math.toRadians(this))

