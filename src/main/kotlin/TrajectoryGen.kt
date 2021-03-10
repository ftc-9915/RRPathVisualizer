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

        var goToShootingPosePt2_A = TrajectoryBuilder(PoseLibrary.placeGoalAndShootingPose1_PathA, PoseLibrary.placeGoalAndShootingPose1_PathA.heading, combinedConstraints )
            .splineTo(placeGoalAndShootingPose2_PathA.vec(), placeGoalAndShootingPose2_PathA.heading)
            .build()

        var goToPickUpGoalPose1_A = TrajectoryBuilder(placeGoalAndShootingPose2_PathA, placeGoalAndShootingPose2_PathA.heading,combinedConstraints)
            .lineToLinearHeading(pickUpGoalPose1_PathA)
            .build()

        var goToPickUpGoalPose2_A = TrajectoryBuilder(pickUpGoalPose1_PathA, pickUpGoalPose1_PathA.heading, combinedConstraints)
            .lineToConstantHeading(pickUpGoalPose2_PathA.vec())
            .build()


        var goToPlaceSecondGoalPart1_A = TrajectoryBuilder(pickUpGoalPose2_PathA, pickUpGoalPose2_PathA.heading, combinedConstraints)
            .lineToSplineHeading(placeSecondGoalPose_PathA)
            .build()

        var goToParkingPosePt1_A = TrajectoryBuilder(placeSecondGoalPose_PathA, placeSecondGoalPose_PathA.heading, combinedConstraints)
            .lineToConstantHeading(parkingPose1_PathA.vec())
            .build()

        var goToParkingPosePt2_A = TrajectoryBuilder(parkingPose1_PathA, parkingPose1_PathA.heading, combinedConstraints)
            .lineToConstantHeading(parkingPose2_PathA.vec())
            .build()

        // Path-B
        var goToShootingPosePt1_B = TrajectoryBuilder(PoseLibrary.START_POS_BLUE_2, PoseLibrary.START_POS_BLUE_2.heading, combinedConstraints )
            .splineTo(shootingPosePt1_PathB.vec(), shootingPosePt1_PathB.heading)
            .splineTo(shootingPosePt2_PathB.vec(), shootingPosePt2_PathB.heading)
            .build()

        var goToPlaceGoalPose_B = TrajectoryBuilder(shootingPosePt2_PathB, shootingPosePt2_PathB.heading, combinedConstraints )
            .splineTo(placeGoalPose_PathB.vec(), placeGoalPose_PathB.heading)
            .build()

        var goToPickUpGoalPose1_B = TrajectoryBuilder(placeGoalPose_PathB, placeGoalPose_PathB.heading,combinedConstraints)
            .lineToLinearHeading(pickUpGoalPose1_PathB)
            .build()

        var goToPickUpGoalPose2_B = TrajectoryBuilder(pickUpGoalPose1_PathB, pickUpGoalPose1_PathB.heading, combinedConstraints)
            .lineToConstantHeading(pickUpGoalPose2_PathB.vec())
            .build()


        var goToPlaceSecondGoalPart1_B = TrajectoryBuilder(pickUpGoalPose2_PathB, pickUpGoalPose2_PathB.heading, combinedConstraints)
            .lineToSplineHeading(placeSecondGoalPose1_PathB)
            .build()

        var goToPlaceSecondGoalPart2_B = TrajectoryBuilder(placeSecondGoalPose1_PathB, placeSecondGoalPose1_PathB.heading, combinedConstraints)
            .lineToConstantHeading(placeSecondGoalPose2_PathB.vec())
            .build()

        var goToParkingPose_B = TrajectoryBuilder(placeSecondGoalPose2_PathB, placeSecondGoalPose2_PathB.heading, combinedConstraints)
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
       // blue2_A.add(goToPickUpGoalPose1_A)
       // blue2_A.add(goToPickUpGoalPose2_A)
       // blue2_A.add(goToPlaceSecondGoalPart1_A)
       // blue2_A.add(goToParkingPosePt1_A)
       // blue2_A.add(goToParkingPosePt2_A)




        blue2_B.add(goToShootingPosePt1_B)
        blue2_B.add(goToPlaceGoalPose_B)
        blue2_B.add(goToPickUpGoalPose1_B)
        blue2_B.add(goToPickUpGoalPose2_B)
        blue2_B.add(goToPlaceSecondGoalPart1_B)
        blue2_B.add(goToPlaceSecondGoalPart2_B)
        blue2_B.add(goToParkingPose_B)


        blue2_C.add(goToShootingPosePt1_C)
        blue2_C.add(goToPlaceGoalPose_C)
        blue2_C.add(goToPickUpGoalPose1_C)
        blue2_C.add(goToPickUpGoalPose2_C)
        blue2_C.add(goToPlaceSecondGoalPart1_C)
        blue2_C.add(goToParkingPose_C)

        return blue2_C
    }

    fun drawOffbounds() {
        GraphicsUtil.fillRect(Vector2d(0.0, -63.0), 18.0, 18.0) // robot against the wall
    }
}

val Double.toRadians get() = (Math.toRadians(this))

