package org.firstinspires.ftc.teamcode;


import com.acmerobotics.roadrunner.Pose2d;


public final class FieldConstantsRed {

    /*
     *
     *
     * */


    public static final class FieldConstants {

        static final double length = 144;
        static final double width = 144;
        static final double tileSize = 24;
        static final double tileTeeth = .75;
        static final double distanceBetwwenSpikeMarks = 10;
    }

    public static final class SampleConstants{
        static final double length = 3.5;
        static final double side = 1.5;
    }
    public static final double basketSideStartAngle = Math.toRadians(90);
    public static final double observationSideStartAngle = Math.toRadians(90);

    public static final double observationZoneCenterPose = Math.toRadians(90);
    public static final double spikeMarkYCenter = -24 - FieldConstants.tileTeeth - SampleConstants.length / 2;


    public static Pose2d basketSideStartPose = new Pose2d(55, 60, basketSideStartAngle);
    public static Pose2d observationSideStartPose = new Pose2d(55, 60, observationSideStartAngle);

    public static Pose2d basketPose = new Pose2d(-64, -64, Math.toRadians(-45));


    public static Pose2d parkPose = new Pose2d(48, -50, Math.toRadians(45));
    public static Pose2d specimenDropOffPose = new Pose2d(0, -24, Math.toRadians(0));


    public static Pose2d wallSideYellowSample = new Pose2d(-60, spikeMarkYCenter, Math.toRadians(0));
    public static Pose2d centerYellowSample = new Pose2d(-50,  spikeMarkYCenter, Math.toRadians(0));
    public static Pose2d middleYellowSample = new Pose2d(-40,  spikeMarkYCenter, Math.toRadians(0));


    public static Pose2d wallSideRedSample = new Pose2d(55,  spikeMarkYCenter, Math.toRadians(0));
    public static Pose2d centerRedSample = new Pose2d(55,  spikeMarkYCenter, Math.toRadians(0));
    public static Pose2d middleRedSample = new Pose2d(55,  spikeMarkYCenter, Math.toRadians(0));


//


}






