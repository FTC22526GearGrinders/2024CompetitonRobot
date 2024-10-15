package com.example.meepmeeptesting;


import com.acmerobotics.roadrunner.geometry.Pose2d;


public final class FieldConstantsBlue {

    /*
     *
     *
     * */

    //basket

    public static Pose2d basketSideStartPose = new Pose2d(45, 58,  Math.toRadians(225));
    public static Pose2d basketDeliverPose = new Pose2d(55, 55,Math.toRadians(225));
    public static Pose2d innerYellowPickupPose = new Pose2d(48.7, 36.1, Math.toRadians(270));
    public static Pose2d midYellowPickupPose = new Pose2d(57.7, 36.1, Math.toRadians(270));
    public static Pose2d outerYellowApproachPose = new Pose2d(50, 25.5, Math.toRadians(0));
    public static Pose2d outerYellowPickupPose = new Pose2d(60, 25.5, Math.toRadians(0));

    //specimen

    public static Pose2d specimenSideStartPose = new Pose2d(12, -64, Math.toRadians(90));
    public static Pose2d specimenDeliverPose1 = new Pose2d(12, -32, Math.toRadians(90));
    public static Pose2d specimenDeliverPose2 = new Pose2d(9, -32, Math.toRadians(90));
    public static Pose2d specimenDeliverPose3 = new Pose2d(3, -32, Math.toRadians(90));
    public static Pose2d specimenDeliverPose4 = new Pose2d(0, -32, Math.toRadians(90));


    public static Pose2d samplePickupPose = new Pose2d(-57.7, 62, Math.toRadians(270));

    public static Pose2d innerBluePickupPose = new Pose2d(-45, 31, Math.toRadians(225));
    public static Pose2d midBluePickupPose = new Pose2d(-55, 31, Math.toRadians(225));
    public static Pose2d outerBluePickupPose = new Pose2d(-60, 25.5, Math.toRadians(180));


}






