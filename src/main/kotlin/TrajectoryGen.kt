import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.acmerobotics.roadrunner.trajectory.Trajectory
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder
import com.acmerobotics.roadrunner.trajectory.constraints.DriveConstraints
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumConstraints

import PoseLibrary.*


object TrajectoryGen {
    // Remember to set these constraints to the same values as your DriveConstants.java file in the quickstart
    private val driveConstraints = DriveConstraints(58.75, 75.0, 0.0, 180.0.toRadians, 180.0.toRadians, 0.0)

    // Remember to set your track width to an estimate of your actual bot to get accurate trajectory profile duration!
    private const val trackWidth = 16.0

    private val combinedConstraints = MecanumConstraints(driveConstraints, trackWidth)

    private val startPoseBlue1 = Pose2d(-63.0, 48.0, 0.0.toRadians)
    private val startPoseBlue2 = Pose2d(-48.0, -48.0, 90.0.toRadians)


    fun createTrajectory(): ArrayList<Trajectory> {

        //Declare Routes
        val blue2_A = ArrayList<Trajectory>();
        val blue2_B  = ArrayList<Trajectory>();
        val blue2_C = ArrayList<Trajectory>();


        // Path-A
        var goToShootingPosePt1_A = TrajectoryBuilder(PoseLibrary.START_POS_BLUE_2, PoseLibrary.START_POS_BLUE_2.heading, combinedConstraints )
            .splineTo(placeGoalAndShootingPose1_PathA.vec(), placeGoalAndShootingPose1_PathA.heading)
            .build()

        var goToShootingPosePt2_A = TrajectoryBuilder(goToShootingPosePt1_A.end(), goToShootingPosePt1_A.end().heading, combinedConstraints )
            .splineTo(placeGoalAndShootingPose2_PathA.vec(), placeGoalAndShootingPose2_PathA.heading)
            .build()

        var goToPickUpGoalPose1_A = TrajectoryBuilder(goToShootingPosePt2_A.end(), goToShootingPosePt2_A.end().heading,combinedConstraints)
            .lineToLinearHeading(pickUpGoalPose1_PathA)
            .build()

        var goToPickUpGoalPose2_A = TrajectoryBuilder(goToPickUpGoalPose1_A.end(), goToPickUpGoalPose1_A.end().heading, combinedConstraints)
            .lineToConstantHeading(pickUpGoalPose2_PathA.vec())
            .build()


        var goToPlaceSecondGoalPart1_A = TrajectoryBuilder(goToPickUpGoalPose2_A.end(), goToPickUpGoalPose2_A.end().heading, combinedConstraints)
            .lineToSplineHeading(placeSecondGoalPose_PathA)
            .build()

        var goToParkingPosePt1_A = TrajectoryBuilder(goToPlaceSecondGoalPart1_A.end(), goToPlaceSecondGoalPart1_A.end().heading, combinedConstraints)
            .lineToConstantHeading(parkingPose1_PathA.vec())
            .build()

        var goToParkingPosePt2_A = TrajectoryBuilder(goToParkingPosePt1_A.end(), goToParkingPosePt1_A.end().heading, combinedConstraints)
            .lineToConstantHeading(parkingPose2_PathA.vec())
            .build()

        // Path-B
        var goToPowerShotPosePt1_B = TrajectoryBuilder(PoseLibrary.START_POS_BLUE_2, PoseLibrary.START_POS_BLUE_2.heading, combinedConstraints )
            .splineTo(POWER_SHOT_POSE_3.vec(), POWER_SHOT_POSE_3.heading)
            .build()

        var goToPowerShotPosePt2_B = TrajectoryBuilder(goToPowerShotPosePt1_B.end(), goToPowerShotPosePt1_B.end().heading, combinedConstraints )
            .lineToConstantHeading(POWER_SHOT_POSE_2.vec())
            .build()


        var goToPowerShotPosePt3_B = TrajectoryBuilder(goToPowerShotPosePt2_B.end(), goToPowerShotPosePt2_B.end().heading, combinedConstraints )
            .lineToConstantHeading(POWER_SHOT_POSE_1.vec())
            .build()

        var goToPlaceGoalPose_B = TrajectoryBuilder(goToPowerShotPosePt3_B.end(), goToPowerShotPosePt3_B.end().heading, combinedConstraints )
            .splineTo(placeGoalPose_PathB.vec(), placeGoalPose_PathB.heading)
            .build()

        var goToPickUpRingAndGoalPose_B = TrajectoryBuilder(goToPlaceGoalPose_B.end(), goToPlaceGoalPose_B.end().heading,combinedConstraints)
            .splineTo(pickUpGoalAndRingPose1_PathB.vec(), pickUpGoalAndRingPose1_PathB.heading)
            .splineTo(pickUpGoalAndRingPose2_PathB.vec(), pickUpGoalAndRingPose2_PathB.heading)
            .build()


        var goToShootingPose_B = TrajectoryBuilder(goToPickUpRingAndGoalPose_B.end(), goToPickUpRingAndGoalPose_B.end().heading,  combinedConstraints)
            .lineToSplineHeading(SHOOTING_POSE_BC)
            .build()

        var goToPlaceSecondGoalPart1_B = TrajectoryBuilder(goToShootingPose_B.end(), goToShootingPose_B.end().heading,  combinedConstraints)
            .lineToSplineHeading(placeSecondGoalPose2_PathB)
            .build()


        var goToParkingPose_B = TrajectoryBuilder(goToPlaceSecondGoalPart1_B.end(), goToPlaceSecondGoalPart1_B.end().heading, combinedConstraints)
            .lineToConstantHeading(parkPose_PathB.vec())
            .build()


        // Path-C
        var goToShootingPosePt1_C = TrajectoryBuilder(PoseLibrary.START_POS_BLUE_2, PoseLibrary.START_POS_BLUE_2.heading, combinedConstraints )
            .splineTo(shootingPosePt1_PathC.vec(), shootingPosePt1_PathC.heading)
            .splineTo(shootingPosePt2_PathC.vec(), shootingPosePt2_PathC.heading)
            .build()

        var goToPlaceGoalPose_C = TrajectoryBuilder(shootingPosePt2_PathC, shootingPosePt2_PathC.heading, combinedConstraints )
            .splineTo(placeGoalPose_PathC.vec(), placeGoalPose_PathC.heading)
            .build()

        var goToPickUpGoalPose1_C = TrajectoryBuilder(placeGoalPose_PathC, placeGoalPose_PathC.heading,combinedConstraints)
            .lineToLinearHeading(pickUpGoalPose1_PathC)
            .build()

        var goToPickUpGoalPose2_C = TrajectoryBuilder(pickUpGoalPose1_PathC, pickUpGoalPose1_PathC.heading, combinedConstraints)
            .lineToConstantHeading(pickUpGoalPose2_PathC.vec())
            .build()


        var goToPlaceSecondGoalPart1_C = TrajectoryBuilder(pickUpGoalPose2_PathC, pickUpGoalPose2_PathC.heading, combinedConstraints)
            .lineToSplineHeading(placeSecondGoalPose1_PathC)
            .build()

        var goToParkingPose_C = TrajectoryBuilder(placeSecondGoalPose1_PathC, placeSecondGoalPose1_PathC.heading, combinedConstraints)
            .lineToConstantHeading(parkPose_PathC.vec())
            .build()


        blue2_A.add(goToShootingPosePt1_A)
        blue2_A.add(goToShootingPosePt2_A)
        blue2_A.add(goToPickUpGoalPose1_A)
        blue2_A.add(goToPickUpGoalPose2_A)
        blue2_A.add(goToPlaceSecondGoalPart1_A)
        blue2_A.add(goToParkingPosePt1_A)
        blue2_A.add(goToParkingPosePt2_A)




        blue2_B.add(goToPowerShotPosePt1_B)
        blue2_B.add(goToPowerShotPosePt2_B)
        blue2_B.add(goToPowerShotPosePt3_B)
        blue2_B.add(goToPlaceGoalPose_B)
        blue2_B.add(goToPickUpRingAndGoalPose_B)
        blue2_B.add(goToShootingPose_B)
        blue2_B.add(goToPlaceSecondGoalPart1_B)
        blue2_B.add(goToParkingPose_B)


        blue2_C.add(goToShootingPosePt1_C)
        blue2_C.add(goToPlaceGoalPose_C)
        blue2_C.add(goToPickUpGoalPose1_C)
        blue2_C.add(goToPickUpGoalPose2_C)
        blue2_C.add(goToPlaceSecondGoalPart1_C)
        blue2_C.add(goToParkingPose_C)

        return blue2_B
    }

    fun drawOffbounds() {
        GraphicsUtil.fillRect(Vector2d(0.0, -63.0), 18.0, 18.0) // robot against the wall
    }
}

val Double.toRadians get() = (Math.toRadians(this))

