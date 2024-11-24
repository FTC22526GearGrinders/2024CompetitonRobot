package com.example.meepmeeptesting;


import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;

public final class FieldConstantsRedMM {


    public static Pose2d basketSideStartPose = flipBlueToRedPose(FieldConstantsBlueMM.basketSideStartPose);

    //basket
    public static Pose2d basketDeliverPose = flipBlueToRedPose(FieldConstantsBlueMM.basketDeliverPose);
    public static Pose2d innerYellowPickupPose = flipBlueToRedPose(FieldConstantsBlueMM.innerYellowPickupPose);
    public static Pose2d innerYellowPrePickupPose = flipBlueToRedPose(FieldConstantsBlueMM.innerYellowPrePickupPose);
    public static Pose2d midYellowPickupPose = flipBlueToRedPose(FieldConstantsBlueMM.midYellowPickupPose);
    public static Pose2d midYellowPrePickupPose = flipBlueToRedPose(FieldConstantsBlueMM.midYellowPrePickupPose);
    public static Pose2d outerYellowApproachPose = flipBlueToRedPose(FieldConstantsBlueMM.outerYellowApproachPose);
    public static Pose2d outerYellowPickupPose = flipBlueToRedPose(FieldConstantsBlueMM.outerYellowPickupPose);
    public static Pose2d ascentZonePickupPose = flipBlueToRedPose(FieldConstantsBlueMM.ascentZonePickupPose);
    public static Pose2d ascentZoneParkPose = flipBlueToRedPose(FieldConstantsBlueMM.ascentZoneParkPose);
    //specimen

    //flip here is intentional DO NOT CHANGE
    public static double specimenDropAngle = FieldConstantsBlueMM.specimenPickupAngle;
    public static double specimenPickupAngle = FieldConstantsBlueMM.specimenDropAngle;

    public static Pose2d specimenSideStartPose = flipBlueToRedPose(FieldConstantsBlueMM.specimenSideStartPose);
    public static Pose2d sample1ObservationZoneDropPose = flipBlueToRedPose(FieldConstantsBlueMM.sample1ObservationZoneDropPose);
    public static Pose2d sample2ObservationZoneDropPose = flipBlueToRedPose(FieldConstantsBlueMM.sample2ObservationZoneDropPose);
    public static Pose2d specimenDeliverPose1 = flipBlueToRedPose(FieldConstantsBlueMM.specimenDeliverPose1);
    public static Pose2d specimenDeliverPose2 = flipBlueToRedPose(FieldConstantsBlueMM.specimenDeliverPose2);
    public static Pose2d specimenDeliverPose3 = flipBlueToRedPose(FieldConstantsBlueMM.specimenDeliverPose3);
    public static Pose2d specimenDeliverPose4 = flipBlueToRedPose(FieldConstantsBlueMM.specimenDeliverPose4);
    public static Pose2d specimenDeliverApproachPose1 = flipBlueToRedPose(FieldConstantsBlueMM.specimenDeliverApproachPose1);
    public static Pose2d specimenDeliverApproachPose2 = flipBlueToRedPose(FieldConstantsBlueMM.specimenDeliverApproachPose2);
    public static Pose2d specimenDeliverApproachPose3 = flipBlueToRedPose(FieldConstantsBlueMM.specimenDeliverApproachPose3);
    public static Pose2d specimenDeliverApproachPose4 = flipBlueToRedPose(FieldConstantsBlueMM.specimenDeliverApproachPose4);
    public static Pose2d specimenPickupPose = flipBlueToRedPose(FieldConstantsBlueMM.specimenPickupPose);
    public static Pose2d specimenPrePickupPose = flipBlueToRedPose(FieldConstantsBlueMM.specimenPrePickupPose);
    public static Pose2d firstStagePushInnerPose = flipBlueToRedPose(FieldConstantsBlueMM.firstStagePushInnerPose);
    public static Vector2d secondStagePushInnerVector = flipBlueToRedVector(FieldConstantsBlueMM.secondStagePushInnerVector);
    public static Vector2d thirdStagePushInnerVector = flipBlueToRedVector(FieldConstantsBlueMM.thirdStagePushInnerVector);
    public static Pose2d firstStagePushMidPose = flipBlueToRedPose(FieldConstantsBlueMM.firstStagePushMidPose);
    public static Vector2d secondStagePushMidVector = flipBlueToRedVector(FieldConstantsBlueMM.secondStagePushMidVector);
    public static Vector2d thirdStagePushMidVector = flipBlueToRedVector(FieldConstantsBlueMM.thirdStagePushMidVector);
    public static Pose2d parkPose = flipBlueToRedPose(FieldConstantsBlueMM.parkPose);

    static Vector2d flipBlueToRedVector(Vector2d blue) {
        return new Vector2d(-blue.x, -blue.y);
    }

    static double flipBlueToRedHeading(Pose2d blue) {
        return blue.heading.toDouble() >= 0 ? blue.heading.toDouble() - Math.PI : blue.heading.toDouble() + Math.PI;
    }

    static Pose2d flipBlueToRedPose(Pose2d blue) {
        double heading = blue.heading.toDouble() >= 0 ? blue.heading.toDouble() - Math.PI : blue.heading.toDouble() + Math.PI;
        return new Pose2d(new Vector2d(-blue.position.x, -blue.position.y), heading);
    }


}



