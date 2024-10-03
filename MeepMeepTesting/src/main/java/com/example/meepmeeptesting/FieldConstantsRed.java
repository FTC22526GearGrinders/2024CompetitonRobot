package com.example.meepmeeptesting;


import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

public final class FieldConstantsRed {


    //field values


    public static final double basketSideStartAngle = Math.toRadians(90);
    public static final double specimenSideStartAngle = Math.toRadians(90);
    public static final double observationSideStartAngle = Math.toRadians(0);


    //specimen values
    public static final double observationZoneCenterPose = Math.toRadians(90);
    static final double distanceBetweenSpikeMarks = 10;

    static final double basketDeliverAngle = Math.toRadians(-135);
    static final double specimenPickupAngle = Math.toRadians(180);

    static final double specimenLength = 3.5;
    public static final double spikeMarkYCenter = -24 - Constants.FieldConstants.tileTeeth - specimenLength / 2;
    public static Vector2d wallSideYellowSample = new Vector2d(-60, spikeMarkYCenter);
    public static Vector2d centerYellowSample = new Vector2d(-50, spikeMarkYCenter);
    public static Vector2d middleYellowSample = new Vector2d(-40, spikeMarkYCenter);
    public static Vector2d alSideRedSample = new Vector2d(55, spikeMarkYCenter);
    public static Vector2d centerRedSample = new Vector2d(55, spikeMarkYCenter);
    public static Vector2d middleRedSample = new Vector2d(55, spikeMarkYCenter);
    static final double specimenSide = 1.5;
    public static Pose2d basketPose = new Pose2d(-55, -55, basketDeliverAngle);
    public static Pose2d parkPose = new Pose2d(48, -50, basketDeliverAngle);


    //auto start locationa
    public static Pose2d basketSideStartPose = new Pose2d(-30, -55, basketSideStartAngle);
    public static Pose2d observationSideStartPose = new Pose2d(-55, 60, observationSideStartAngle);
    public static Pose2d specimenStartPose = new Pose2d(-10, -48, specimenSideStartAngle);
    public static Pose2d specimenDropPose = new Pose2d(-10, -30, specimenSideStartAngle);
    public static Pose2d specimenDropBackupPose = new Pose2d(-36, -48, specimenSideStartAngle);

    public static Pose2d nearInnerSpecimenPose = new Pose2d(-36, -25, specimenPickupAngle);
    public static Pose2d pickupInnerSpecimenPose = new Pose2d(-40, -25, specimenPickupAngle);

    public static Pose2d nearMiddleSpecimenPose = new Pose2d(-40, -25, specimenPickupAngle);
    public static Pose2d pickupMiddleSpecimenPose = new Pose2d(-54, -25, specimenPickupAngle);

    public static Pose2d nearOuterSpecimenPose = new Pose2d(-50, -26, specimenPickupAngle);
    public static Pose2d pickupOuterSpecimenPose = new Pose2d(-60, -26, specimenPickupAngle);


    public static Pose2d adjustPose(Pose2d pose, boolean invertX, boolean invertY, boolean invertHeading) {

        double x = pose.getX();
        double y = pose.getY();
        double heading = pose.getHeading();
        if (invertX) {
            x = -x;
        }
        if (invertY) {
            y = -y;
        }

        if (invertX && invertY && heading != 180)
            heading = -heading;


        if (invertX && invertY && heading == 180)
            heading = 0;


        return new Pose2d(x, y, heading);


    }

//


}






