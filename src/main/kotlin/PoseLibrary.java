import com.acmerobotics.roadrunner.geometry.Pose2d;

public class PoseLibrary {

    // Starting positions
    public static Pose2d START_POS_BLUE_2 = new Pose2d(-54.75, 26.5, Math.toRadians(0.0));
    public static Pose2d START_POS_BLUE_1 = new Pose2d(-54.75, 48, Math.toRadians(0.0));

    //transfer pose between opmodes, defaults to auto starting pose for testing
    public static Pose2d AUTO_ENDING_POSE = new Pose2d(-55, 26.5, Math.toRadians(0.0));

    //common shooting poses
    public static Pose2d SHOOTING_POSE_A = new Pose2d(-4, 55, Math.toRadians(-27.5)); //used in auto path A
    public static Pose2d SHOOTING_POSE_BC = new Pose2d(6.8066, 26.37388, Math.toRadians(-5)); //used in auto path B and C


    //automatic power shot poses
    public static Pose2d POWER_SHOT_START_POSE = new Pose2d(10, 72-8.5, Math.toRadians(0.0));
    public static Pose2d POWER_SHOT_POSE_1 = new Pose2d(0, 31.5, Math.toRadians(-21));
    public static Pose2d POWER_SHOT_POSE_2 = new Pose2d(0, 24, Math.toRadians(-21));
    public static Pose2d POWER_SHOT_POSE_3 = new Pose2d(0, 16.5, Math.toRadians(-21));

    public static Pose2d[] POWER_SHOT_POSES = {POWER_SHOT_START_POSE, POWER_SHOT_POSE_1, POWER_SHOT_POSE_2, POWER_SHOT_POSE_3};



    //PATH-A
    public static Pose2d placeGoalAndShootingPose1_PathA = SHOOTING_POSE_A;
    public static Pose2d placeGoalAndShootingPose2_PathA = new Pose2d(-5, 55, Math.toRadians(-0.1));
    public static Pose2d pickUpGoalPose1_PathA = new Pose2d(-24, 59, Math.toRadians(180.0));
    public static Pose2d pickUpGoalPose2_PathA = new Pose2d(-26, 59, Math.toRadians(180.0));
    public static Pose2d placeSecondGoalPose_PathA = new Pose2d(0, 57, Math.toRadians(0.0));
    public static Pose2d parkingPose1_PathA = new Pose2d(0, 30, Math.toRadians(0.0));
    public static Pose2d parkingPose2_PathA = new Pose2d(15, 30, Math.toRadians(0.0));

    //PATH-B
    public static Pose2d shootingPosePt1_PathB = new Pose2d (-10,17);
    public static Pose2d placeGoalPose_PathB = new Pose2d(22, 25, Math.toRadians(0.0));
    public static Pose2d pickUpGoalAndRingPose1_PathB = new Pose2d(-8, 36, Math.toRadians(180));
    public static Pose2d pickUpGoalAndRingPose2_PathB = new Pose2d(-24, 40, Math.toRadians(125.0));
    public static Pose2d placeSecondGoalPose1_PathB = new Pose2d(27, 57, Math.toRadians(0.0));
    public static Pose2d placeSecondGoalPose2_PathB = new Pose2d(23, 33, Math.toRadians(0.0));
    public static Pose2d parkPose_PathB = new Pose2d(17, 27, Math.toRadians(0.0));

    //PATH-C
    public static  Pose2d shootingPosePt1_PathC = new Pose2d (-24,21);
    public static Pose2d shootingPosePt2_PathC = SHOOTING_POSE_BC;
    public static Pose2d placeGoalPose_PathC = new Pose2d(48, 52, Math.toRadians(-0.1));
    public static Pose2d pickUpGoalPose1_PathC = new Pose2d(-24, 57, Math.toRadians(180.0));
    public static Pose2d pickUpGoalPose2_PathC = new Pose2d(-25, 57, Math.toRadians(180.0));
    public static Pose2d placeSecondGoalPose1_PathC = new Pose2d(48, 55, Math.toRadians(0.0));
    public static Pose2d parkPose_PathC = new Pose2d(19, 50, Math.toRadians(0.0));


}
