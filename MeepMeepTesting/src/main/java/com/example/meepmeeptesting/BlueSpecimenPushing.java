package com.example.meepmeeptesting;


import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueLight;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.DriveShim;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class BlueSpecimenPushing {


    public static void main(String[] args) {
        Action firstSpecimenDeliverMoveAction;
        Action secondSpecimenDeliverMoveAction;
        Action thirdSpecimenDeliverMoveAction;
        Action fourthSpecimenDeliverMoveAction;

        Action firstSpecimenDeliverBackupAction;
        Action secondSpecimenDeliverBackupAction;
        Action thirdSpecimenDeliverBackupAction;

        Action secondSpecimenPickupMoveAction;
        Action thirdSpecimenPickupMoveAction;
        Action fourthSpecimenPickupMoveAction;

        Action firstSamplePickupMoveAction;
        Action secondSamplePickupMoveAction;
        Action thirdSamplePickupMoveAction;

        Action firstSampleDropOffMoveAction;

        Action placeSpecimenAction = new SleepAction(2);
        Action pickupSampleAction = new SleepAction(2);
        Action transferSampleToBucketAction = new SleepAction(2);
        Action dropSampleAction = new SleepAction(2);
        Action collectSpecimenAction = new SleepAction(2);

        Action deliverFourSpecimens;


        MeepMeep meepMeep = new MeepMeep(800);
        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(45, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .setDimensions(Constants.RobotConstants.width, Constants.RobotConstants.length)
                .setColorScheme(new ColorSchemeBlueLight())
                .setStartPose(FieldConstantsBlueMM.specimenSideStartPose)
                .build();

        DriveShim drive = myBot.getDrive();

        firstSpecimenDeliverMoveAction = drive.actionBuilder(FieldConstantsBlueMM.specimenSideStartPose)
                .lineToY(FieldConstantsBlueMM.specimenDeliverPose1.position.y)
                .build();//move to place first specimen

        firstSpecimenDeliverBackupAction = drive.actionBuilder(FieldConstantsBlueMM.specimenDeliverPose1)
                .splineToSplineHeading(new Pose2d(-36, 42, Math.toRadians(-180)), Math.toRadians(-180))
                .waitSeconds(0.5)
                //.splineToConstantHeading(new Vector2d(-44, 12), Math.toRadians(270))
                //.strafeTo(new Vector2d(-34, 12))
                .strafeTo(new Vector2d(-36, 8))
                .splineToConstantHeading(new Vector2d(-48, 56), Math.toRadians(80))
                //  .strafeTo(new Vector2d(-44, 50))
                .build();

        firstSamplePickupMoveAction = drive.actionBuilder(FieldConstantsBlueMM.specimenDeliverApproachPose1)
                .lineToY(36)
                .build();


        firstSampleDropOffMoveAction = drive.actionBuilder(FieldConstantsBlueMM.innerBluePickupPose)
                .strafeToLinearHeading(FieldConstantsBlueMM.firstSampleDropOffPose.position, FieldConstantsBlueMM.firstSampleDropOffPose.heading)
                .build();

        secondSpecimenPickupMoveAction = drive.actionBuilder(FieldConstantsBlueMM.innerBluePickupPose)
                .strafeToLinearHeading(FieldConstantsBlueMM.specimenPickupPose.position, Math.toRadians(270))
                .build();//move to drop first sample and pick up second specimen

        secondSpecimenDeliverMoveAction = drive.actionBuilder(FieldConstantsBlueMM.specimenPickupPose)
                .splineToLinearHeading(FieldConstantsBlueMM.specimenDeliverApproachPose2, Math.toRadians(90))
                .lineToY(FieldConstantsBlueMM.specimenDeliverPose2.position.y)
                .build();//place second specimen

        secondSpecimenDeliverBackupAction = drive.actionBuilder(FieldConstantsBlueMM.specimenDeliverPose2)
                .lineToY(FieldConstantsBlueMM.specimenDeliverApproachPose2.position.y)
                .build();//clear submersible

        secondSamplePickupMoveAction = drive.actionBuilder(FieldConstantsBlueMM.specimenDeliverApproachPose2)
                .splineToLinearHeading(FieldConstantsBlueMM.midBluePickupPose, FieldConstantsBlueMM.midBluePickupPose.heading)
                .build();//move to pickup inner sample

        thirdSpecimenPickupMoveAction = drive.actionBuilder(FieldConstantsBlueMM.midBluePickupPose)
                .strafeToLinearHeading(FieldConstantsBlueMM.specimenPickupPose.position, FieldConstantsBlueMM.specimenPickupPose.heading)
                .build();//move to drop second sample and pick up third specimen

        thirdSpecimenDeliverMoveAction = drive.actionBuilder(FieldConstantsBlueMM.specimenPickupPose)
                .splineToLinearHeading(FieldConstantsBlueMM.specimenDeliverApproachPose3, Math.toRadians(90))
                .lineToY(FieldConstantsBlueMM.specimenDeliverPose3.position.y)
                .build();//place thirdspecimen

        thirdSpecimenDeliverBackupAction = drive.actionBuilder(FieldConstantsBlueMM.specimenDeliverPose3)
                .lineToY(FieldConstantsBlueMM.specimenDeliverApproachPose3.position.y)
                .build();

        thirdSamplePickupMoveAction = drive.actionBuilder(FieldConstantsBlueMM.specimenDeliverApproachPose3)
                .splineToLinearHeading(FieldConstantsBlueMM.outerBluePickupPose, FieldConstantsBlueMM.outerBluePickupPose.heading)
                .build();//move to pickup inner sample

        fourthSpecimenPickupMoveAction = drive.actionBuilder(FieldConstantsBlueMM.outerBluePickupPose)
                .strafeToLinearHeading(FieldConstantsBlueMM.specimenPickupPose.position, FieldConstantsBlueMM.specimenPickupPose.heading)
                .build();//move to drop third sample and pick up fourth specimen

        fourthSpecimenDeliverMoveAction = drive.actionBuilder(FieldConstantsBlueMM.specimenPickupPose)
                .splineToLinearHeading(FieldConstantsBlueMM.specimenDeliverApproachPose4, Math.toRadians(90))
                .lineToY(FieldConstantsBlueMM.specimenDeliverPose4.position.y)
                .build();//deliver fourth specimen


        deliverFourSpecimens = new SequentialAction(

                firstSpecimenDeliverMoveAction,
                placeSpecimenAction,
                firstSpecimenDeliverBackupAction
//                new ParallelAction(
//                        firstSamplePickupMoveAction,
//                        pickupSampleAction),
//                new ParallelAction(
//                        secondSpecimenPickupMoveAction,
//                        transferSampleToBucketAction),
//                new ParallelAction(
//                        collectSpecimenAction,
//                        dropSampleAction),
//              //  firstSampleDropOffMoveAction,
//                secondSpecimenDeliverMoveAction,
//                placeSpecimenAction,
//                secondSpecimenDeliverBackupAction,
//                new ParallelAction(
//                        secondSamplePickupMoveAction,
//                        pickupSampleAction),
//                new ParallelAction(
//                        thirdSpecimenPickupMoveAction,
//                        transferSampleToBucketAction),
//                new ParallelAction(
//                        collectSpecimenAction,
//                        dropSampleAction),
//                thirdSpecimenDeliverMoveAction,
//                placeSpecimenAction,
//                thirdSpecimenDeliverBackupAction,
//                new ParallelAction(
//                        thirdSamplePickupMoveAction,
//                        pickupSampleAction),
//                new ParallelAction(
//                        fourthSpecimenPickupMoveAction,
//                        transferSampleToBucketAction),
//                new ParallelAction(
//                        collectSpecimenAction,
//                        dropSampleAction),
//                fourthSpecimenDeliverMoveAction,
//                placeSpecimenAction


        );

        myBot.runAction(deliverFourSpecimens);
        ShowField.showIt(meepMeep, myBot);
        meepMeep.start();
    }

}


