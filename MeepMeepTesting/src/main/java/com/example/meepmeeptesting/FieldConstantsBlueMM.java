package com.example.meepmeeptesting;


import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;


public final class FieldConstantsBlueMM {

    /*
     *
     *
     * */

    //basket

    public static double pickUpArmEX = 4;
    public static double driveTo = 4;
    public static Pose2d basketSideStartPose = new Pose2d(36, 72 - Constants.RobotConstants.width / 2, Math.toRadians(180));
    public static Pose2d basketDeliverPose = new Pose2d(56, 56, Math.toRadians(-135));


    public static Pose2d innerYellowPickupPose = new Pose2d(48.7, 25.5 + Constants.RobotConstants.length / 2 + pickUpArmEX, Math.toRadians(-90));
    public static Pose2d innerYellowPrePickupPose = new Pose2d(48.7, 25.5 + Constants.RobotConstants.length / 2 + pickUpArmEX + driveTo, Math.toRadians(-90));
    public static Pose2d midYellowPickupPose = new Pose2d(58.7, 25.5 + Constants.RobotConstants.length / 2 + pickUpArmEX, Math.toRadians(-90));
    public static Pose2d midYellowPrePickupPose = new Pose2d(58.7, 25.5 + Constants.RobotConstants.length / 2 + pickUpArmEX + driveTo, Math.toRadians(-90));

    public static Pose2d outerYellowApproachPose = new Pose2d(66 - Constants.RobotConstants.length / 2 - pickUpArmEX - driveTo, 25.5, Math.toRadians(0));
    public static Pose2d outerYellowPickupPose = new Pose2d(72 - Constants.RobotConstants.length / 2 - pickUpArmEX - driveTo, 25.5, Math.toRadians(0));

    public static Pose2d ascentZoneParkPose = new Pose2d(24, -12, Math.toRadians(180));
    public static Pose2d ascentZonePickupPose = new Pose2d(24, -12, Math.toRadians(180));

    //specimen

    public static double specimenDropAngle = Math.toRadians(90);
    public static double specimenPickupAngle = Math.toRadians(-90);


    public static Pose2d specimenSideStartPose = new Pose2d(-12, Constants.FieldConstants.length / 2 - Constants.RobotConstants.length / 2, Math.toRadians(90));

    public static Pose2d sample1ObservationZoneDropPose = new Pose2d(-48, 60 - Constants.RobotConstants.width / 2 + 8, Math.toRadians(180));
    public static Pose2d sample2ObservationZoneDropPose = new Pose2d(-58, 60 - Constants.RobotConstants.width / 2 + 8, Math.toRadians(180));

    public static Pose2d firstStagePushInnerPose = new Pose2d(-36, 42, Math.toRadians(180));
    public static Vector2d secondStagePushInnerVector = new Vector2d(-36, 10);
    public static Vector2d thirdStagePushInnerVector = new Vector2d(-48, 10);

    public static Pose2d firstStagePushMidPose = new Pose2d(-48, 42, Math.toRadians(180));
    public static Vector2d secondStagePushMidVector = new Vector2d(-48, 10);
    public static Vector2d thirdStagePushMidVector = new Vector2d(-58, 10);


    public static Pose2d specimenDeliverPose1 = new Pose2d(-12, 24 + Constants.RobotConstants.length / 2, Math.toRadians(90));
    public static Pose2d specimenDeliverPose2 = new Pose2d(-9, 24 + Constants.RobotConstants.length / 2, Math.toRadians(90));
    public static Pose2d specimenDeliverPose3 = new Pose2d(-3, 24 + Constants.RobotConstants.length / 2, Math.toRadians(90));
    public static Pose2d specimenDeliverPose4 = new Pose2d(0, 24 + Constants.RobotConstants.length / 2, Math.toRadians(90));

    public static Pose2d specimenDeliverApproachPose1 = new Pose2d(-12, 24 + Constants.RobotConstants.length / 2 + 6, Math.toRadians(90));
    public static Pose2d specimenDeliverApproachPose2 = new Pose2d(-9, 24 + Constants.RobotConstants.length / 2 + 6, Math.toRadians(90));
    public static Pose2d specimenDeliverApproachPose3 = new Pose2d(-3, 24 + Constants.RobotConstants.length / 2 + 6, Math.toRadians(90));
    public static Pose2d specimenDeliverApproachPose4 = new Pose2d(0, 24 + Constants.RobotConstants.length / 2 + 6, Math.toRadians(90));


    public static Pose2d specimenPickupPose = new Pose2d(-36, 72 - Constants.RobotConstants.width / 2, Math.toRadians(-90));
    public static Pose2d specimenPrePickupPose = new Pose2d(-36, specimenPickupPose.position.y - 9, Math.toRadians(-90));


    public static Pose2d parkPose = new Pose2d(-48, 60 - Constants.RobotConstants.width / 2 + 8, Math.toRadians(180));

}



