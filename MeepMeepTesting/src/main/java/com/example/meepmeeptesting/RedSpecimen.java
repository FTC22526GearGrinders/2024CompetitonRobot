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

public class RedSpecimen {


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

        Action placeSpecimenAction = new SleepAction(2);
        Action pickupSampleAction = new SleepAction(2);
        Action transferSampleToBucketAction = new SleepAction(2);
        Action dropSampleAction = new SleepAction(2);
        Action collectSpecimenAction = new SleepAction(2);

        Action deliverFourSpecimens;

        MeepMeep meepMeep = new MeepMeep(800);
        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .setDimensions(Constants.RobotConstants.width, Constants.RobotConstants.length)
                .setColorScheme(new ColorSchemeBlueLight())
                .setStartPose(FieldConstantsRedMM.specimenSideStartPose)
                .build();

        DriveShim drive = myBot.getDrive();

        firstSpecimenDeliverMoveAction = drive.actionBuilder(drive.getPoseEstimate())
                .lineToY(FieldConstantsRedMM.specimenDeliverPose1.position.y)
                .build();//move to place first specimen

        firstSpecimenDeliverBackupAction = drive.actionBuilder(FieldConstantsRedMM.specimenDeliverPose1)
                .lineToY(FieldConstantsRedMM.specimenDeliverApproachPose1.position.y)
                .build();

        firstSamplePickupMoveAction = drive.actionBuilder(FieldConstantsRedMM.specimenDeliverApproachPose1)
                .strafeToLinearHeading(FieldConstantsRedMM.innerRedPickupPose.position, FieldConstantsRedMM.innerRedPickupPose.heading)
                .build();//move to pickup inner sample

        secondSpecimenPickupMoveAction = drive.actionBuilder(FieldConstantsRedMM.innerRedPickupPose)
                .strafeToLinearHeading(FieldConstantsRedMM.specimenPickupPose.position, FieldConstantsRedMM.specimenPickupPose.heading)
                .build();//move to drop first sample and pick up second specimen

        secondSpecimenDeliverMoveAction = drive.actionBuilder(FieldConstantsRedMM.specimenPickupPose)
                .splineToLinearHeading(FieldConstantsRedMM.specimenDeliverApproachPose2, Math.toRadians(90))
                .lineToY(FieldConstantsRedMM.specimenDeliverPose2.position.y)
                .build();//place second specimen

        secondSpecimenDeliverBackupAction = drive.actionBuilder(FieldConstantsRedMM.specimenDeliverPose2)
                .lineToY(FieldConstantsRedMM.specimenDeliverApproachPose2.position.y)
                .build();//clear submersible

        secondSamplePickupMoveAction = drive.actionBuilder(FieldConstantsRedMM.specimenDeliverApproachPose2)
                .strafeToLinearHeading(FieldConstantsRedMM.midRedPickupPose.position, FieldConstantsRedMM.midRedPickupPose.heading)
                .build();//move to pickup inner sample

        thirdSpecimenPickupMoveAction = drive.actionBuilder(FieldConstantsRedMM.midRedPickupPose)
                .strafeToLinearHeading(FieldConstantsRedMM.specimenPickupPose.position, FieldConstantsRedMM.specimenPickupPose.heading)
                .build();//move to drop second sample and pick up third specimen

        thirdSpecimenDeliverMoveAction = drive.actionBuilder(FieldConstantsRedMM.specimenPickupPose)
                .splineToLinearHeading(FieldConstantsRedMM.specimenDeliverApproachPose3, Math.toRadians(90))
                .lineToY(FieldConstantsRedMM.specimenDeliverPose3.position.y)
                .build();//place thirdspecimen

        thirdSpecimenDeliverBackupAction = drive.actionBuilder(FieldConstantsRedMM.specimenDeliverPose3)
                .lineToY(FieldConstantsRedMM.specimenDeliverApproachPose3.position.y)
                .build();

        thirdSamplePickupMoveAction = drive.actionBuilder(FieldConstantsRedMM.specimenDeliverApproachPose3)
                .strafeToLinearHeading(FieldConstantsRedMM.outerRedPickupPose.position, FieldConstantsRedMM.outerRedPickupPose.heading)
                .build();//move to pickup inner sample

        fourthSpecimenPickupMoveAction = drive.actionBuilder(FieldConstantsRedMM.outerRedPickupPose)
                .strafeToLinearHeading(FieldConstantsRedMM.specimenPickupPose.position, FieldConstantsRedMM.specimenPickupPose.heading)
                .build();//move to drop third sample and pick up fourth specimen

        fourthSpecimenDeliverMoveAction = drive.actionBuilder(FieldConstantsRedMM.specimenPickupPose)
                .splineToLinearHeading(FieldConstantsRedMM.specimenDeliverApproachPose4, Math.toRadians(90))
                .lineToY(FieldConstantsRedMM.specimenDeliverPose4.position.y)
                .build();//deliver fourth specimen


        deliverFourSpecimens = new SequentialAction(

                firstSpecimenDeliverMoveAction,
                placeSpecimenAction,
                firstSpecimenDeliverBackupAction,
                new ParallelAction(
                        firstSamplePickupMoveAction,
                        pickupSampleAction),
                new ParallelAction(
                        secondSpecimenPickupMoveAction,
                        transferSampleToBucketAction),
                new ParallelAction(
                        collectSpecimenAction,
                        dropSampleAction),
                secondSpecimenDeliverMoveAction,
                placeSpecimenAction,
                secondSpecimenDeliverBackupAction,
                new ParallelAction(
                        secondSamplePickupMoveAction,
                        pickupSampleAction),
                new ParallelAction(
                        thirdSpecimenPickupMoveAction,
                        transferSampleToBucketAction),
                new ParallelAction(
                        collectSpecimenAction,
                        dropSampleAction),
                thirdSpecimenDeliverMoveAction,
                placeSpecimenAction,
                thirdSpecimenDeliverBackupAction,
                new ParallelAction(
                        thirdSamplePickupMoveAction,
                        pickupSampleAction),
                new ParallelAction(
                        fourthSpecimenPickupMoveAction,
                        transferSampleToBucketAction),
                new ParallelAction(
                        collectSpecimenAction,
                        dropSampleAction),
                fourthSpecimenDeliverMoveAction,
                placeSpecimenAction


        );

        myBot.runAction(deliverFourSpecimens);
        ShowField.showIt(meepMeep, myBot);
        meepMeep.start();
    }

}



