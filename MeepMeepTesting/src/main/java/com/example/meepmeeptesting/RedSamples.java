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

public class RedSamples {


    public static void main(String[] args) {

        Action firstSampleDeliverMoveAction;
        Action secondSampleDeliverMoveAction;
        Action thirdSampleDeliverMoveAction;
        Action fourthSampleDeliverMoveAction;


        Action secondSamplePickupMoveAction;
        Action thirdSamplePickupMoveAction;
        Action fourthSamplePickupMoveAction;

        Action placeSpecimenAction = new SleepAction(2);
        Action pickupSampleAction = new SleepAction(2);
        Action transferSampleToBucketAction = new SleepAction(2);
        Action dropSampleAction = new SleepAction(2);


        Action deliverFourSamples;


        MeepMeep meepMeep = new MeepMeep(800);
        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .setDimensions(Constants.RobotConstants.width, Constants.RobotConstants.length)
                .setColorScheme(new ColorSchemeBlueLight())
                .setStartPose(FieldConstantsRedMM.basketSideStartPose)
                .build();

        DriveShim drive = myBot.getDrive();

        firstSampleDeliverMoveAction = drive.actionBuilder(FieldConstantsRedMM.basketSideStartPose)
                .strafeTo(FieldConstantsRedMM.basketDeliverPose.position)
                .build();//move to place first specimen

        secondSamplePickupMoveAction = drive.actionBuilder(FieldConstantsRedMM.basketDeliverPose)
                .strafeToLinearHeading(FieldConstantsRedMM.innerYellowPickupPose.position, FieldConstantsRedMM.innerYellowPickupPose.heading)
                .build();

        secondSampleDeliverMoveAction = drive.actionBuilder(FieldConstantsRedMM.innerYellowPickupPose)
                .strafeToLinearHeading(FieldConstantsRedMM.basketDeliverPose.position, FieldConstantsRedMM.basketDeliverPose.heading)
                .build();//move to place first specimen

        thirdSamplePickupMoveAction = drive.actionBuilder(FieldConstantsRedMM.basketDeliverPose)
                .strafeToLinearHeading(FieldConstantsRedMM.midYellowPickupPose.position, FieldConstantsRedMM.midYellowPickupPose.heading)
                .build();

        thirdSampleDeliverMoveAction = drive.actionBuilder(FieldConstantsRedMM.midYellowPickupPose)
                .strafeToLinearHeading(FieldConstantsRedMM.basketDeliverPose.position, FieldConstantsRedMM.basketDeliverPose.heading)
                .build();//move to place first specimen

        fourthSamplePickupMoveAction = drive.actionBuilder(FieldConstantsRedMM.basketDeliverPose)
                .strafeToLinearHeading(FieldConstantsRedMM.outerYellowApproachPose.position, FieldConstantsRedMM.outerYellowApproachPose.heading)
                .waitSeconds(.1)
                .lineToX(FieldConstantsRedMM.outerYellowPickupPose.position.x)

                .build();

        fourthSampleDeliverMoveAction = drive.actionBuilder(FieldConstantsRedMM.outerYellowPickupPose)
                .lineToX(FieldConstantsRedMM.outerYellowApproachPose.position.x)
                .strafeToLinearHeading(FieldConstantsRedMM.basketDeliverPose.position, FieldConstantsRedMM.basketDeliverPose.heading)
                .build();//move to place first specimen


        deliverFourSamples = new SequentialAction(
                new SleepAction(3),
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
                dropSampleAction


        );


        myBot.runAction(deliverFourSamples);
        ShowField.showIt(meepMeep, myBot);
        meepMeep.start();
    }

}



