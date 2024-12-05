package com.example.meepmeeptesting;


import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueLight;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.DriveShim;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;


public class RedOrBlueSamples {


    public static void main(String[] args) {

        Action firstSampleDeliverMoveAction;
        Action secondSampleDeliverMoveAction;
        Action thirdSampleDeliverMoveAction;
        Action fourthSampleDeliverMoveAction;
        Action fifthSampleDeliverMoveAction;

        Action secondSamplePickupMoveAction;
        Action thirdSamplePickupMoveAction;
        Action fourthSamplePickupMoveAction;
        Action fifthSamplePickupMoveAction;

        Action parkAction;

        Action placeSpecimenAction = new SleepAction(2);
        Action pickupSampleAction = new SleepAction(2);
        Action transferSampleToBucketAction = new SleepAction(2);
        Action dropSampleAction = new SleepAction(2);


        FieldConstantsSelect fcs;
        Action deliverFourSamples;

        MeepMeep meepMeep = new MeepMeep(800);
        fcs = new FieldConstantsSelect();
        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .setDimensions(Constants.RobotConstants.width, Constants.RobotConstants.length)
                .setColorScheme(new ColorSchemeBlueLight())
                .setStartPose(fcs.basketSideStartPose)
                .build();

        DriveShim drive = myBot.getDrive();

        firstSampleDeliverMoveAction = drive.actionBuilder(fcs.basketSideStartPose)
                .strafeToLinearHeading(fcs.basketDeliverPose.position, fcs.basketDeliverPose.heading)
                .build();//move to place first specimen

        secondSamplePickupMoveAction = drive.actionBuilder(fcs.basketDeliverPose)
                .strafeToLinearHeading(fcs.innerYellowPrePickupPose.position, fcs.innerYellowPrePickupPose.heading)
                .strafeToLinearHeading(fcs.innerYellowPickupPose.position, fcs.innerYellowPickupPose.heading)
                .build();


        secondSampleDeliverMoveAction = drive.actionBuilder(fcs.innerYellowPickupPose)
                .strafeToLinearHeading(fcs.basketDeliverPose.position, fcs.basketDeliverPose.heading)
                .build();//move to place first specimen

        thirdSamplePickupMoveAction = drive.actionBuilder(fcs.basketDeliverPose)
                .strafeToLinearHeading(fcs.midYellowPrePickupPose.position, fcs.midYellowPrePickupPose.heading)
                .strafeToLinearHeading(fcs.midYellowPickupPose.position, fcs.midYellowPickupPose.heading)
                .build();

        thirdSampleDeliverMoveAction = drive.actionBuilder(fcs.midYellowPickupPose)
                .strafeToLinearHeading(fcs.basketDeliverPose.position, fcs.basketDeliverPose.heading)
                .build();//move to place first specimen

        fourthSamplePickupMoveAction = drive.actionBuilder(fcs.basketDeliverPose)
                .strafeToLinearHeading(fcs.outerYellowPrePose.position, fcs.outerYellowPrePose.heading)
                .build();

        fourthSampleDeliverMoveAction = drive.actionBuilder(fcs.outerYellowPickupPose)
                .lineToX(fcs.outerYellowPrePose.position.x)
                .strafeToLinearHeading(fcs.basketDeliverPose.position, fcs.basketDeliverPose.heading)
                .build();//

        fifthSamplePickupMoveAction = drive.actionBuilder(fcs.basketDeliverPose)
                .strafeToLinearHeading(fcs.ascentZonePickupPose.position, Math.toRadians(180))
                .build();

        fifthSampleDeliverMoveAction = drive.actionBuilder(fcs.ascentZonePickupPose)
                .strafeToLinearHeading(fcs.basketDeliverPose.position, fcs.basketDeliverPose.heading)
                .build();

        parkAction = drive.actionBuilder(fcs.basketDeliverPose)
                .strafeToLinearHeading(fcs.ascentZoneParkPose.position, Math.toRadians(180))
                .build();

        deliverFourSamples = new SequentialAction(
                new SleepAction(.3),
                firstSampleDeliverMoveAction,
                dropSampleAction,
                new ParallelAction(
                        secondSamplePickupMoveAction,
                        pickupSampleAction),
                secondSampleDeliverMoveAction,
                dropSampleAction,
                new ParallelAction(
                        thirdSamplePickupMoveAction,
                        pickupSampleAction),
                thirdSampleDeliverMoveAction,
                dropSampleAction,
                new ParallelAction(
                        fourthSamplePickupMoveAction,
                        pickupSampleAction),
                fourthSampleDeliverMoveAction,
                dropSampleAction,
                fifthSamplePickupMoveAction,
                pickupSampleAction,
                fifthSampleDeliverMoveAction,
                parkAction


        );


        myBot.runAction(deliverFourSamples);
        ShowField.showIt(meepMeep, myBot);
        meepMeep.start();
    }

}



