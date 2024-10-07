package org.firstinspires.ftc.teamcode;


import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;

public final class FieldConstantsRed {


    //field values


    public static final double basketSideStartAngle = Math.toRadians(45);
    public static final double specimenSideStartAngle = Math.toRadians(-90);
    public static final double observationSideStartAngle = Math.toRadians(0);
    public static Pose2d innerYellowPickupPose = new Pose2d(-48.5,-36.1,Math.toRadians(90));
    public static Pose2d midYellowPickupPose= new Pose2d(-57.7,-36.1,Math.toRadians(90));
    public static Pose2d outerYellowPickupPose= new Pose2d(-60,-27.1,Math.toRadians(170));


    //specimen values
    public static final double observationZoneCenterPose = Math.toRadians(90);
    static final double distanceBetweenSpikeMarks = 10;

    static final double basketDeliverAngle = Math.toRadians(45);
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
    public static Pose2d basketDeliverPose = new Pose2d(-55, -55, basketDeliverAngle);
    public static Pose2d parkPose = new Pose2d(48, -50, basketDeliverAngle);


    public static Pose2d basketSideStartPose = new Pose2d(-45, -58, basketSideStartAngle);








//


}






