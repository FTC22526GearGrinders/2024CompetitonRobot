package com.example.meepmeeptesting;


import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;


public final class FieldConstantsSelect {

    /*
     *
     *
     * */
    //basket
    public double pickUpArmEX = 4;
    public double driveTo = 4;
    public Pose2d basketSideStartPose;
    public Pose2d basketDeliverPose;
    public Pose2d innerYellowPickupPose;
    public Pose2d innerYellowPrePickupPose;
    public Pose2d midYellowPickupPose;
    public Pose2d midYellowPrePickupPose;
    public Pose2d outerYellowApproachPose;
    public Pose2d outerYellowPickupPose;
    public Pose2d ascentZoneParkPose;
    public Pose2d ascentZonePickupPose;
    public double specimenDropAngle;
    //specimen
    public double specimenPickupAngle;
    public Pose2d specimenSideStartPose;
    public Pose2d sample1ObservationZoneDropPose;
    public Pose2d sample3ObservationZoneDropPose;
    public Pose2d sample2ObservationZoneDropPose;
    public Pose2d firstStagePushInnerPose;
    public Vector2d secondStagePushInnerVector;
    public Vector2d thirdStagePushInnerVector;
    public Pose2d firstStagePushMidPose;
    public Vector2d secondStagePushMidVector;
    public Vector2d thirdStagePushMidVector;
    public Pose2d specimenDeliverPose1;
    public Pose2d specimenDeliverPose2;
    public Pose2d specimenDeliverPose3;
    public Pose2d specimenDeliverPose4;
    public Pose2d specimenDeliverApproachPose1;
    public double specimenExpectedDeliverDistance;
    public Pose2d specimenDeliverApproachPose2;
    public Pose2d specimenDeliverApproachPose3;
    public Pose2d specimenDeliverApproachPose4;
    public Pose2d specimenPickupPose;
    public Pose2d specimenPickupApproachPose;
    //these will be compared to distance sensor readings and the deliver pose adjusted if sensor is too far off
    public double specimenExpectedPickupDistance;
    public Pose2d specimenParkPose;

    public FieldConstantsSelect() {

        setBlue();

    }

    public void setBlue() {

        basketSideStartPose = new Pose2d(36, 72 - Constants.RobotConstants.width / 2, Math.toRadians(180));
        basketDeliverPose = new Pose2d(56, 56, Math.toRadians(-135));


        innerYellowPickupPose = new Pose2d(48.7, 25.5 + Constants.RobotConstants.length / 2 + pickUpArmEX, Math.toRadians(-90));
        innerYellowPrePickupPose = new Pose2d(48.7, 25.5 + Constants.RobotConstants.length / 2 + pickUpArmEX + driveTo, Math.toRadians(-90));
        midYellowPickupPose = new Pose2d(58.7, 25.5 + Constants.RobotConstants.length / 2 + pickUpArmEX, Math.toRadians(-90));
        midYellowPrePickupPose = new Pose2d(58.7, 25.5 + Constants.RobotConstants.length / 2 + pickUpArmEX + driveTo, Math.toRadians(-90));

        outerYellowApproachPose = new Pose2d(66 - Constants.RobotConstants.length / 2 - pickUpArmEX - driveTo, 25.5, Math.toRadians(0));
        outerYellowPickupPose = new Pose2d(72 - Constants.RobotConstants.length / 2 - pickUpArmEX - driveTo, 25.5, Math.toRadians(0));

        ascentZoneParkPose = new Pose2d(24, -12, Math.toRadians(180));
        ascentZonePickupPose = new Pose2d(24, -12, Math.toRadians(180));

        //specimen
        specimenDropAngle = Math.toRadians(90);
        specimenPickupAngle = Math.toRadians(-90);


        specimenSideStartPose = new Pose2d(-12, Constants.FieldConstants.length / 2 - Constants.RobotConstants.length / 2, Math.toRadians(90));

        sample1ObservationZoneDropPose = new Pose2d(-48, 60 - Constants.RobotConstants.width / 2 + 8, Math.toRadians(180));
        sample2ObservationZoneDropPose = new Pose2d(-58, 60 - Constants.RobotConstants.width / 2 + 8, specimenPickupAngle);

        firstStagePushInnerPose = new Pose2d(-36, 42, Math.toRadians(180));
        secondStagePushInnerVector = new Vector2d(-36, 10);
        thirdStagePushInnerVector = new Vector2d(-48, 10);

        firstStagePushMidPose = new Pose2d(-24, 42, Math.toRadians(180));
        secondStagePushMidVector = new Vector2d(-48, 0);
        thirdStagePushMidVector = new Vector2d(-58, 10);


        specimenDeliverPose1 = new Pose2d(-12, 24 + Constants.RobotConstants.length / 2, Math.toRadians(90));
        specimenDeliverPose2 = new Pose2d(-9, 24 + Constants.RobotConstants.length / 2, Math.toRadians(90));
        specimenDeliverPose3 = new Pose2d(-3, 24 + Constants.RobotConstants.length / 2, Math.toRadians(90));
        specimenDeliverPose4 = new Pose2d(0, 24 + Constants.RobotConstants.length / 2, Math.toRadians(90));

        specimenDeliverApproachPose1 = new Pose2d(-12, 24 + Constants.RobotConstants.length / 2 + 6, Math.toRadians(90));
        specimenDeliverApproachPose2 = new Pose2d(-9, 24 + Constants.RobotConstants.length / 2 + 6, Math.toRadians(90));
        specimenDeliverApproachPose3 = new Pose2d(-3, 24 + Constants.RobotConstants.length / 2 + 6, Math.toRadians(90));
        specimenDeliverApproachPose4 = new Pose2d(0, 24 + Constants.RobotConstants.length / 2 + 6, Math.toRadians(90));

        specimenPickupPose = new Pose2d(-36, 72 - Constants.RobotConstants.width / 2, Math.toRadians(-90));
        specimenPickupApproachPose = new Pose2d(-36, specimenPickupPose.position.y - 9, Math.toRadians(-90));
        specimenParkPose = new Pose2d(-48, 60 - Constants.RobotConstants.width / 2 + 8, Math.toRadians(180));
    }

    public void setRed() {
        basketSideStartPose = flipBlueToRedPose(basketSideStartPose);
        basketDeliverPose = flipBlueToRedPose(basketDeliverPose);


        innerYellowPickupPose = flipBlueToRedPose(innerYellowPickupPose);
        innerYellowPrePickupPose = flipBlueToRedPose(innerYellowPrePickupPose);
        midYellowPickupPose = flipBlueToRedPose(midYellowPickupPose);
        midYellowPrePickupPose = flipBlueToRedPose(midYellowPrePickupPose);

        outerYellowApproachPose = flipBlueToRedPose(outerYellowApproachPose);
        outerYellowPickupPose = flipBlueToRedPose(outerYellowPickupPose);

        ascentZoneParkPose = flipBlueToRedPose(ascentZoneParkPose);
        ascentZonePickupPose = flipBlueToRedPose(ascentZonePickupPose);

        //specimen
        specimenDropAngle = Math.toRadians(-90);
        specimenPickupAngle = Math.toRadians(90);


        specimenSideStartPose = flipBlueToRedPose(specimenSideStartPose);

        sample1ObservationZoneDropPose = flipBlueToRedPose(sample1ObservationZoneDropPose);
        sample2ObservationZoneDropPose = flipBlueToRedPose(sample2ObservationZoneDropPose);

        firstStagePushInnerPose = flipBlueToRedPose(firstStagePushInnerPose);
        secondStagePushInnerVector = flipBlueToRedVector(secondStagePushInnerVector);
        thirdStagePushInnerVector = flipBlueToRedVector(thirdStagePushInnerVector);

        firstStagePushMidPose = flipBlueToRedPose(firstStagePushMidPose);
        secondStagePushMidVector = flipBlueToRedVector(secondStagePushMidVector);
        thirdStagePushMidVector = flipBlueToRedVector(thirdStagePushMidVector);


        specimenDeliverPose1 = flipBlueToRedPose(specimenDeliverPose1);
        specimenDeliverPose2 = flipBlueToRedPose(specimenDeliverPose2);
        specimenDeliverPose3 = flipBlueToRedPose(specimenDeliverPose3);
        specimenDeliverPose4 = flipBlueToRedPose(specimenDeliverPose4);

        specimenDeliverApproachPose1 = flipBlueToRedPose(specimenDeliverApproachPose1);
        specimenDeliverApproachPose2 = flipBlueToRedPose(specimenDeliverApproachPose2);
        specimenDeliverApproachPose3 = flipBlueToRedPose(specimenDeliverApproachPose3);
        specimenDeliverApproachPose4 = flipBlueToRedPose(specimenDeliverApproachPose4);

        specimenPickupPose = flipBlueToRedPose(specimenPickupPose);
        specimenPickupApproachPose = flipBlueToRedPose(specimenPickupApproachPose);
        specimenParkPose = flipBlueToRedPose(specimenParkPose);
    }

    Vector2d flipBlueToRedVector(Vector2d blue) {
        return new Vector2d(-blue.x, -blue.y);
    }

    double flipBlueToRedHeading(Pose2d blue) {
        return blue.heading.toDouble() >= 0 ? blue.heading.toDouble() - Math.PI : blue.heading.toDouble() + Math.PI;
    }

    Pose2d flipBlueToRedPose(Pose2d blue) {
        double heading = blue.heading.toDouble() >= 0 ? blue.heading.toDouble() - Math.PI : blue.heading.toDouble() + Math.PI;
        return new Pose2d(new Vector2d(-blue.position.x, -blue.position.y), heading);
    }
}


