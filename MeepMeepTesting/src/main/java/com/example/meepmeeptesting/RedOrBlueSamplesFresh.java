package com.example.meepmeeptesting;


import com.acmerobotics.roadrunner.Action;
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


public class RedOrBlueSamplesFresh {


    public static void main(String[] args) {

        Action firstSampleStrafeMove;
        Action firstSampleDeliverMove;
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
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .setDimensions(Constants.RobotConstants.length, Constants.RobotConstants.length)
                .setColorScheme(new ColorSchemeBlueLight())
                .setStartPose(fcs.basketSideStartPose)
                .build();

        DriveShim drive = myBot.getDrive();

        // fcs.setRed();

        firstSampleStrafeMove = drive.actionBuilder(fcs.basketSideStartPose)
                .strafeTo(fcs.basketSideStrafePose.position).build();


        firstSampleDeliverMove = drive.actionBuilder(fcs.basketSideStrafePose)
                .strafeToLinearHeading(fcs.basketDeliverPose.position, fcs.basketDeliverPose.heading)
                .build();//move to place first specimen


        firstSampleDeliverMove = drive.actionBuilder(fcs.altBasketStartPose)
                .strafeToLinearHeading(fcs.basketDeliverPose.position, fcs.basketDeliverPose.heading)
                .build();//move to place first specimen

        secondSamplePickupMove = drive.actionBuilder(fcs.basketDeliverPose)
                .strafeToLinearHeading(fcs.innerYellowPickupPose.position, fcs.innerYellowPickupPose.heading);


        secondSampleDeliverMove = drive.actionBuilder(fcs.innerYellowPickupPose)
                .strafeToLinearHeading(fcs.basketDeliverPose.position, fcs.basketDeliverPose.heading);


        thirdSamplePickupMove = drive.actionBuilder(fcs.basketDeliverPose)
                .strafeToLinearHeading(fcs.midYellowPickupPose.position, fcs.midYellowPickupPose.heading);


        thirdSampleDeliverMove = drive.actionBuilder(fcs.midYellowPickupPose)
                .strafeToLinearHeading(fcs.basketDeliverPose.position, fcs.basketDeliverPose.heading);

        fourthSamplePickupMove = drive.actionBuilder(fcs.basketDeliverPose)
                .strafeToLinearHeading(fcs.outerYellowPrePose.position, fcs.outerYellowPrePose.heading);


        fourthSamplePickupComplete = fourthSamplePickupMove.endTrajectory().fresh()
                .strafeToLinearHeading(fcs.outerYellowPickupPose.position, fcs.outerYellowPickupPose.heading,
                        approachVel, approachAccel).build();


        fourthSampleDeliverMove = drive.actionBuilder(fcs.outerYellowPickupPose)
                .strafeToLinearHeading(fcs.basketDeliverPose.position, fcs.basketDeliverPose.heading);


        parkAction = drive.actionBuilder(fcs.basketDeliverPose)
                .strafeToLinearHeading(fcs.ascentZoneParkPose.position, fcs.ascentZoneParkPose.heading)
                .build();


        deliverFourSamples = new SequentialAction(
                // new SleepAction(10),
                //  firstSampleStrafeMove,

                firstSampleDeliverMove,
                dropSampleAction,

                secondSamplePickupMove.build(),
                pickupSampleAction,

                secondSampleDeliverMove.build(),
                dropSampleAction,


                thirdSamplePickupMove.build(),
                pickupSampleAction,

                thirdSampleDeliverMove.build(),
                dropSampleAction,

                fourthSamplePickupMove.build(),
                fourthSamplePickupComplete,
                pickupSampleAction,

                fourthSampleDeliverMove.build(),
                dropSampleAction,

                parkAction);

        myBot.setPose(fcs.basketSideStartPose);
        myBot.runAction(deliverFourSamples);

        ShowField.showIt(meepMeep, myBot);
        meepMeep.start();
    }

}



