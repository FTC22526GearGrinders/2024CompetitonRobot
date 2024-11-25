package com.example.meepmeeptesting;


import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueLight;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.DriveShim;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class ThreeBlueSpecimen {


    public static void main(String[] args) {
        Action firstSpecimenDeliverMoveAction;
        Action secondSpecimenDeliverMoveAction;
        Action thirdSpecimenDeliverMoveAction;


        Action secondSpecimenPickupMoveAction;
        Action thirdSpecimenPickupMoveAction;


        Action firstSampleMoveToObservationZoneAction;
        Action secondSampleMoveToObservationZoneAction;


        Action placeSpecimenAction = new SleepAction(2);
        Action pickupSampleAction = new SleepAction(2);
        Action transferSampleToBucketAction = new SleepAction(2);
        Action dropSampleAction = new SleepAction(2);
        Action collectSpecimenAction = new SleepAction(2);

        Action deliverFourSpecimens;


        MeepMeep meepMeep = new MeepMeep(800);
        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(100, 100, Math.toRadians(180), Math.toRadians(180), 15)
                .setDimensions(Constants.RobotConstants.width, Constants.RobotConstants.length)
                .setColorScheme(new ColorSchemeBlueLight())
                .setStartPose(FieldConstantsBlueMM.specimenSideStartPose)
                .build();

        DriveShim drive = myBot.getDrive();

        firstSpecimenDeliverMoveAction = drive.actionBuilder(FieldConstantsBlueMM.specimenSideStartPose)
                .strafeTo(FieldConstantsBlueMM.specimenDeliverApproachPose1.position)
                .strafeTo(FieldConstantsBlueMM.specimenDeliverPose1.position)
                .build();//move to place first specimen

        firstSampleMoveToObservationZoneAction = drive.actionBuilder(FieldConstantsBlueMM.specimenDeliverPose1)
                .strafeToLinearHeading(FieldConstantsBlueMM.firstStagePushInnerPose.position, Math.toRadians(180))
                .strafeTo(FieldConstantsBlueMM.secondStagePushInnerVector)
                .strafeToLinearHeading(FieldConstantsBlueMM.thirdStagePushInnerVector, FieldConstantsBlueMM.specimenPickupAngle)
                .strafeTo(FieldConstantsBlueMM.sample3ObservationZoneDropPose.position)
                .build();

        secondSpecimenPickupMoveAction = drive.actionBuilder(FieldConstantsBlueMM.sample3ObservationZoneDropPose)
                .strafeToLinearHeading(FieldConstantsBlueMM.specimenPickupApproachPose.position, FieldConstantsBlueMM.specimenPickupAngle)
                .strafeTo(FieldConstantsBlueMM.specimenPickupPose.position)
                .waitSeconds(1)
                .build();

        secondSpecimenDeliverMoveAction = drive.actionBuilder(FieldConstantsBlueMM.specimenPickupPose)
                .strafeToLinearHeading(FieldConstantsBlueMM.specimenDeliverApproachPose2.position, FieldConstantsBlueMM.specimenDropAngle)
                .strafeTo(FieldConstantsBlueMM.specimenDeliverPose2.position)
                .build();//place second specimen

        thirdSpecimenPickupMoveAction = drive.actionBuilder(FieldConstantsBlueMM.specimenDeliverPose2)
                .strafeToLinearHeading(FieldConstantsBlueMM.specimenPickupApproachPose.position, FieldConstantsBlueMM.specimenPickupAngle)
                .strafeTo(FieldConstantsBlueMM.specimenPickupPose.position)
                .waitSeconds(1)
                .build();


        thirdSpecimenDeliverMoveAction = drive.actionBuilder(FieldConstantsBlueMM.specimenPickupPose)
                .strafeToLinearHeading(FieldConstantsBlueMM.specimenDeliverApproachPose3.position, FieldConstantsBlueMM.specimenDropAngle)
                .strafeTo(FieldConstantsBlueMM.specimenDeliverPose3.position)
                .build();//place second specimen


        secondSampleMoveToObservationZoneAction = drive.actionBuilder(FieldConstantsBlueMM.specimenDeliverPose3)
                .strafeToLinearHeading(FieldConstantsBlueMM.firstStagePushInnerPose.position, Math.toRadians(180))
                .strafeToLinearHeading(FieldConstantsBlueMM.firstStagePushMidPose.position, Math.toRadians(180))
                .strafeTo(FieldConstantsBlueMM.thirdStagePushMidVector)
                .strafeTo(FieldConstantsBlueMM.sample2ObservationZoneDropPose.position)
                .build();


        deliverFourSpecimens =
                new SequentialAction(
                        firstSpecimenDeliverMoveAction,
                        firstSampleMoveToObservationZoneAction,
                        secondSpecimenPickupMoveAction,
                        secondSpecimenDeliverMoveAction,
                        thirdSpecimenPickupMoveAction,
                        thirdSpecimenDeliverMoveAction,
                        secondSampleMoveToObservationZoneAction
                );

        myBot.runAction(deliverFourSpecimens);
        myBot.setPose(FieldConstantsBlueMM.specimenSideStartPose);
        ShowField.showIt(meepMeep, myBot);
        meepMeep.start();
    }

}


