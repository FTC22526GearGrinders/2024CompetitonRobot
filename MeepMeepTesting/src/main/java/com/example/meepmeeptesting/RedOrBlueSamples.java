package com.example.meepmeeptesting;


import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueLight;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.DriveShim;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;


public class RedOrBlueSamples {


    public static void main(String[] args) {

        Action firstSampleStrafeMove;
        Action firstSampleDeliverMove;

        Action firstTurn;
        Action secondTurn;
        TrajectoryActionBuilder secondSampleDeliverMove;
        Action secondSampleDeliverComplete;
        TrajectoryActionBuilder thirdSampleDeliverMove;
        Action thirdSampleDeliverComplete;
        TrajectoryActionBuilder fourthSampleDeliverMove;
        Action fourthSampleDeliverComplete;


        TrajectoryActionBuilder secondSamplePickupMove;
        Action secondSamplePickupComplete;
        TrajectoryActionBuilder thirdSamplePickupMove;
        Action thirdSamplePickupComplete;
        TrajectoryActionBuilder fourthSamplePickupMove;
        Action fourthSamplePickupComplete;
        Action parkAction;

        TranslationalVelConstraint approachVel;
        ProfileAccelConstraint approachAccel;

        Action placeSpecimenAction = new SleepAction(2);
        Action pickupSampleAction = new SleepAction(2);
        Action transferSampleToBucketAction = new SleepAction(2);
        Action dropSampleAction = new SleepAction(2);


        FieldConstantsSelect fcs;
        Action deliverFourSamples;

        MeepMeep meepMeep = new MeepMeep(800);
        fcs = new FieldConstantsSelect();
        approachVel = new TranslationalVelConstraint(10.0);
        approachAccel = new ProfileAccelConstraint(-20.0, 20.0);
        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(20, 20, Math.toRadians(180), Math.toRadians(180), 15)
                .setDimensions(Constants.RobotConstants.length, Constants.RobotConstants.length)
                .setColorScheme(new ColorSchemeBlueLight())
                .setStartPose(fcs.basketSideStartPose)
                .build();

        DriveShim drive = myBot.getDrive();

        //  fcs.setRed();

        firstSampleStrafeMove = drive.actionBuilder(fcs.basketSideStartPose)
                .strafeTo(fcs.basketSideStrafePose.position).build();


        firstSampleDeliverMove = drive.actionBuilder(fcs.basketSideStrafePose)
                .strafeToLinearHeading(fcs.basketDeliverPose.position, fcs.basketDeliverPose.heading)
                .build();//move to place first specimen

        firstTurn = drive.actionBuilder(fcs.basketDeliverPose)
                .turn(Math.toRadians(5)).build();

        secondSamplePickupMove = drive.actionBuilder(fcs.basketDeliverPoseTurned)
                .strafeToLinearHeading(fcs.innerYellowPickupPose.position, fcs.innerYellowPickupPose.heading);


        secondSampleDeliverMove = drive.actionBuilder(fcs.innerYellowPickupPose)
                .strafeToLinearHeading(fcs.basketDeliverPose.position, fcs.basketDeliverPose.heading);

        secondTurn = drive.actionBuilder(fcs.basketDeliverPose)
                .turn(Math.toRadians(10)).build();


        thirdSamplePickupMove = drive.actionBuilder(fcs.basketDeliverPoseTurned1)
                .strafeToLinearHeading(fcs.midYellowPickupPose.position, fcs.midYellowPickupPose.heading);


        thirdSampleDeliverMove = drive.actionBuilder(fcs.midYellowPickupPose)
                .strafeToLinearHeading(fcs.basketDeliverPose.position, fcs.basketDeliverPose.heading);

        fourthSamplePickupMove = drive.actionBuilder(fcs.basketDeliverPose)
                .strafeToLinearHeading(fcs.outerYellowPrePose.position, fcs.outerYellowPrePose.heading);


        fourthSampleDeliverMove = drive.actionBuilder(fcs.outerYellowPickupPose)
                .strafeToLinearHeading(fcs.basketDeliverPose.position, fcs.basketDeliverPose.heading);


        parkAction = drive.actionBuilder(fcs.basketDeliverPose)
                .strafeToLinearHeading(fcs.ascentZoneParkPose.position, fcs.ascentZoneParkPose.heading)
                .build();


        deliverFourSamples = new SequentialAction(
                new SleepAction(1),

                new ParallelAction(
                        new SequentialAction(
                                firstSampleStrafeMove,
                                firstSampleDeliverMove),
                        new SleepAction(2)),

                dropSampleAction,
                firstTurn,
                secondSamplePickupMove.build(),
                pickupSampleAction,

                secondSampleDeliverMove.build(),
                dropSampleAction,

                secondTurn,
                thirdSamplePickupMove.build(),
                pickupSampleAction,

                thirdSampleDeliverMove.build(),
                dropSampleAction,

//                fourthSamplePickupMove.build(),
//                fourthSamplePickupComplete,
//                pickupSampleAction,
//
//                fourthSampleDeliverMove.build(),
//                dropSampleAction,

                parkAction);

        myBot.setPose(fcs.basketSideStartPose);
        myBot.runAction(deliverFourSamples);

        ShowField.showIt(meepMeep, myBot);
        meepMeep.start();
    }

}



