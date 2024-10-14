package org.firstinspires.ftc.teamcode;


import com.acmerobotics.roadrunner.Pose2d;

public final class FieldConstantsRed {


    //field values


    public static final double basketSideStartAngle = Math.toRadians(45);
    public static final double basketDeliverAngle = Math.toRadians(45);
    public static Pose2d basketSideStartPose = new Pose2d(-45, -58, basketSideStartAngle);
    public static Pose2d basketDeliverPose = new Pose2d(-55, -55, basketDeliverAngle);
    public static Pose2d innerYellowPickupPose = new Pose2d(-48.5, -36.1, Math.toRadians(90));
    public static Pose2d midYellowPickupPose = new Pose2d(-57.7, -36.1, Math.toRadians(90));
    public static Pose2d outerYellowApproachPose = new Pose2d(-50, -25.5, Math.toRadians(180));
    public static Pose2d outerYellowPickupPose = new Pose2d(-60, -25.5, Math.toRadians(180));


//
    //Specimen poses


    public static Pose2d specimenSideStartPose = new Pose2d(0, -64, Math.toRadians(270));
    public static Pose2d specimenDeliverPose1 = new Pose2d(0, -32, Math.toRadians(270));
    public static Pose2d specimenDeliverPose2 = new Pose2d(3, -32, Math.toRadians(270));
    public static Pose2d specimenDeliverPose3 = new Pose2d(6, -32, Math.toRadians(270));
    public static Pose2d specimenDeliverPose4 = new Pose2d(9, -32, Math.toRadians(270));

    public static double specimenDeliverPullbackInches = 12;


    public static Pose2d sampleDeliverPose = new Pose2d(48.5, -54, Math.toRadians(90));
    public static Pose2d samplePickupPose = new Pose2d(57.7, -62, Math.toRadians(90));

    public static Pose2d innerRedPickupPose = new Pose2d(45, -31, Math.toRadians(45));
    public static Pose2d midRedPickupPose = new Pose2d(55, -31, Math.toRadians(45));
    public static Pose2d outerRedApproachPose = flipToSamplePoseIncHeading(outerYellowApproachPose);
    public static Pose2d outerRedPickupPose = flipToSamplePoseIncHeading(outerYellowPickupPose);


    public static Pose2d flipToSamplePose(Pose2d pose) {
        double x = -pose.position.x;
        double y = pose.position.y;
        double heading = pose.heading.toDouble();
        return new Pose2d(x, y, heading);
    }

    public static Pose2d flipToSamplePoseIncHeading(Pose2d pose) {
        double x = -pose.position.x;
        double y = pose.position.y;
        double heading = pose.heading.toDouble() + Math.PI;
        return new Pose2d(x, y, heading);
    }

}






