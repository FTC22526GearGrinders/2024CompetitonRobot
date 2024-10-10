package com.example.meepmeeptesting;


import com.acmerobotics.roadrunner.geometry.Pose2d;


public final class FieldConstantsBlue {

    /*
     *
     *
     * */

    public static Pose2d basketSideStartPose = flipXYH(FieldConstantsRed.basketSideStartPose);
    public static Pose2d basketDeliverPose = flipXYH(FieldConstantsRed.basketDeliverPose);

    public static Pose2d innerYellowPickupPose = flipXYH(FieldConstantsRed.innerYellowPickupPose);
    public static Pose2d midYellowPickupPose = flipXYH(FieldConstantsRed.midYellowPickupPose);
    public static Pose2d outerYellowApproachPose = flipXYH(FieldConstantsRed.outerYellowApproachPose);
    public static Pose2d outerYellowPickupPose = flipXYH(FieldConstantsRed.outerYellowPickupPose);




    public static Pose2d sampleSideStartPose =flipXH(FieldConstantsRed.sampleSideStartPose);
    public static Pose2d specimenDeliverPose =new Pose2d(10, -32,Math.toRadians(90));
    public static Pose2d sampleDeliverPose =new Pose2d(60, -54,Math.toRadians(90));
    public static Pose2d samplePickupPose =new Pose2d(60, -62,Math.toRadians(90));
    public static Pose2d innerRedPickupPose =(innerYellowPickupPose);
    public static Pose2d midRedPickupPose =(midYellowPickupPose);
    public static Pose2d outerRedApproachPose =(outerYellowApproachPose);
    public static Pose2d outerRedPickupPose =(outerYellowPickupPose);





    public static Pose2d flipXYH(Pose2d pose) {
        double x = -pose.getX();
        double y = -pose.getY();
        double heading = pose.getHeading() + Math.PI;
        return new Pose2d(x, y, heading);
    }


    public static Pose2d flipXH(Pose2d pose) {
        double x = -pose.getX();
        double y = pose.getY();
        double heading = pose.getHeading()+Math.PI;
        return new Pose2d(x, y, heading);
    }

    public static Pose2d flipToSamplePoseIncHeading(Pose2d pose) {
        double x = -pose.getX();
        double y = pose.getY();
        double heading = pose.getHeading()+Math.PI;
        return new Pose2d(x, y, heading);
    }


}






