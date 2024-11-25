package com.example.meepmeeptesting;


import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;


public final class FieldConstantsBlueMM {

    /*
     *
     *
     * */

    //basket

    public static double pickUpArmEXT = 8;
    public static double driveTo = 4;
    public static double clawEXT = 4;
    public static double specimenPlaceApproach = 6;
    public static double specimenPlckupApproach = 6;
    public static double sampleDropToWall = 8;


    public static Pose2d basketSideStartPose = new Pose2d(36, Constants.FieldConstants.length / 2 - Constants.RobotConstants.width / 2, Math.toRadians(180));
    public static Pose2d basketDeliverPose = new Pose2d(56, 56, Math.toRadians(-135));


    public static Pose2d innerYellowPickupPose = new Pose2d(48.7, 25.5 + Constants.RobotConstants.length / 2 + pickUpArmEXT, Math.toRadians(-90));
    public static Pose2d innerYellowPrePickupPose = new Pose2d(48.7, 25.5 + Constants.RobotConstants.length / 2 + pickUpArmEXT + driveTo, Math.toRadians(-90));
    public static Pose2d midYellowPickupPose = new Pose2d(58.7, 25.5 + Constants.RobotConstants.length / 2 + pickUpArmEXT, Math.toRadians(-90));
    public static Pose2d midYellowPrePickupPose = new Pose2d(58.7, 25.5 + Constants.RobotConstants.length / 2 + pickUpArmEXT + driveTo, Math.toRadians(-90));

    public static Pose2d outerYellowApproachPose = new Pose2d(66 - Constants.RobotConstants.length / 2 - pickUpArmEXT - driveTo, 25.5, Math.toRadians(0));
    public static Pose2d outerYellowPickupPose = new Pose2d(72 - Constants.RobotConstants.length / 2 - pickUpArmEXT - driveTo, 25.5, Math.toRadians(0));

    public static Pose2d ascentZoneParkPose = new Pose2d(24, -12, Math.toRadians(180));
    public static Pose2d ascentZonePickupPose = new Pose2d(24, -12, Math.toRadians(180));

    //specimen

    public static double specimenDropAngle = Math.toRadians(90);
    public static double specimenPickupAngle = Math.toRadians(-90);


    public static Pose2d specimenSideStartPose = new Pose2d(-12, Constants.FieldConstants.length / 2 - Constants.RobotConstants.length / 2 - clawEXT, FieldConstantsBlueMM.specimenDropAngle);

    public static Pose2d sample1ObservationZoneDropPose = new Pose2d(-48, 60 - Constants.RobotConstants.width / 2 + sampleDropToWall, Math.toRadians(180));
    public static Pose2d sample3ObservationZoneDropPose = new Pose2d(-48, 60 - Constants.RobotConstants.width / 2 + sampleDropToWall, FieldConstantsBlueMM.specimenPickupAngle);

    public static Pose2d sample2ObservationZoneDropPose = new Pose2d(-58, 60 - Constants.RobotConstants.width / 2 + sampleDropToWall, FieldConstantsBlueMM.specimenPickupAngle);

    public static Pose2d firstStagePushInnerPose = new Pose2d(-36, 42, Math.toRadians(180));
    public static Vector2d secondStagePushInnerVector = new Vector2d(-36, 10);
    public static Vector2d thirdStagePushInnerVector = new Vector2d(-48, 10);

    public static Pose2d firstStagePushMidPose = new Pose2d(-40, 10, FieldConstantsBlueMM.specimenPickupAngle);
    public static Vector2d secondStagePushMidVector = new Vector2d(-48, 10);
    public static Vector2d thirdStagePushMidVector = new Vector2d(-58, 10);


    public static Pose2d specimenDeliverPose1 = new Pose2d(-12, 24 + Constants.RobotConstants.length / 2 + clawEXT, Math.toRadians(90));
    public static Pose2d specimenDeliverPose2 = new Pose2d(-9, 24 + Constants.RobotConstants.length / 2 + clawEXT, Math.toRadians(90));
    public static Pose2d specimenDeliverPose3 = new Pose2d(-3, 24 + Constants.RobotConstants.length / 2 + clawEXT, Math.toRadians(90));
    public static Pose2d specimenDeliverPose4 = new Pose2d(0, 24 + Constants.RobotConstants.length / 2 + clawEXT, Math.toRadians(90));

    public static Pose2d specimenDeliverApproachPose1 = new Pose2d(-12, 24 + Constants.RobotConstants.length / 2 + clawEXT + specimenPlaceApproach, Math.toRadians(90));
    public static double specimenExpectedDeliverDistance = specimenDeliverPose1.position.y - specimenDeliverApproachPose1.position.y;
    public static Pose2d specimenDeliverApproachPose2 = new Pose2d(-9, 24 + Constants.RobotConstants.length / 2 + clawEXT + specimenPlaceApproach, Math.toRadians(90));
    public static Pose2d specimenDeliverApproachPose3 = new Pose2d(-3, 24 + Constants.RobotConstants.length / 2 + clawEXT + specimenPlaceApproach, Math.toRadians(90));
    public static Pose2d specimenDeliverApproachPose4 = new Pose2d(0, 24 + Constants.RobotConstants.length / 2 + clawEXT + specimenPlaceApproach, Math.toRadians(90));


    public static Pose2d specimenPickupPose = new Pose2d(-36, Constants.FieldConstants.length / 2 - Constants.RobotConstants.width / 2 - clawEXT, Math.toRadians(-90));
    public static Pose2d specimenPickupApproachPose = new Pose2d(-36, specimenPickupPose.position.y - specimenPlckupApproach, Math.toRadians(-90));


    //these will be compared to distance sensor readings and the deliver pose adjusted if sensor is too far off
    public static double specimenExpectedPickupDistance = specimenPickupPose.position.y - specimenPickupApproachPose.position.y;
    public static Pose2d specimenParkPose = new Pose2d(-48, 60 - Constants.RobotConstants.width / 2 + 8, Math.toRadians(180));

}
