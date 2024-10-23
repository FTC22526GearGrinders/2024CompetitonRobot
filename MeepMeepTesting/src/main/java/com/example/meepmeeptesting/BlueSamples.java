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


public class BlueSamples {


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
                .setStartPose(FieldConstantsBlueMM.basketSideStartPose)
                .build();

        DriveShim drive = myBot.getDrive();

        firstSampleDeliverMoveAction = drive.actionBuilder(FieldConstantsBlueMM.basketSideStartPose)
                .strafeTo(FieldConstantsBlueMM.basketDeliverPose.position)
                .build();//move to place first specimen

        secondSamplePickupMoveAction = drive.actionBuilder(FieldConstantsBlueMM.basketDeliverPose)
                .strafeToLinearHeading(FieldConstantsBlueMM.innerYellowPickupPose.position, FieldConstantsBlueMM.innerYellowPickupPose.heading)
                .build();

        secondSampleDeliverMoveAction = drive.actionBuilder(FieldConstantsBlueMM.innerYellowPickupPose)
                .strafeToLinearHeading(FieldConstantsBlueMM.basketDeliverPose.position, FieldConstantsBlueMM.basketDeliverPose.heading)
                .build();//move to place first specimen

        thirdSamplePickupMoveAction = drive.actionBuilder(FieldConstantsBlueMM.basketDeliverPose)
                .strafeToLinearHeading(FieldConstantsBlueMM.midYellowPickupPose.position, FieldConstantsBlueMM.midYellowPickupPose.heading)
                .build();

        thirdSampleDeliverMoveAction = drive.actionBuilder(FieldConstantsBlueMM.midYellowPickupPose)
                .strafeToLinearHeading(FieldConstantsBlueMM.basketDeliverPose.position, FieldConstantsBlueMM.basketDeliverPose.heading)
                .build();//move to place first specimen

        fourthSamplePickupMoveAction = drive.actionBuilder(FieldConstantsBlueMM.basketDeliverPose)
                .strafeToLinearHeading(FieldConstantsBlueMM.outerYellowApproachPose.position, FieldConstantsBlueMM.outerYellowApproachPose.heading)
                .waitSeconds(.1)
                .lineToX(FieldConstantsBlueMM.outerYellowPickupPose.position.x)

                .build();

        fourthSampleDeliverMoveAction = drive.actionBuilder(FieldConstantsBlueMM.outerYellowPickupPose)
                .lineToX(FieldConstantsBlueMM.outerYellowApproachPose.position.x)
                .strafeToLinearHeading(FieldConstantsBlueMM.basketDeliverPose.position, FieldConstantsBlueMM.basketDeliverPose.heading)
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



