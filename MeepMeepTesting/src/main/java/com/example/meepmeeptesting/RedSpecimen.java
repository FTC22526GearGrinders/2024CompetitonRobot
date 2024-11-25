package com.example.meepmeeptesting;


import com.acmerobotics.roadrunner.Action;
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

        Action secondSpecimenPickupMoveAction;
        Action thirdSpecimenPickupMoveAction;
        Action fourthSpecimenPickupMoveAction;

        Action firstSampleMoveToObservationZoneAction;
        Action secondSampleMoveToObservationZoneAction;

        Action parkAction;


        Action placeSpecimenAction = new SleepAction(2);
        Action pickupSampleAction = new SleepAction(2);
        Action transferSampleToBucketAction = new SleepAction(2);
        Action dropSampleAction = new SleepAction(2);
        Action collectSpecimenAction = new SleepAction(2);

        Action deliverFourSpecimens;


        MeepMeep meepMeep = new MeepMeep(800);
        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 100, Math.toRadians(180), Math.toRadians(180), 15)
                .setDimensions(Constants.RobotConstants.width, Constants.RobotConstants.length)
                .setColorScheme(new ColorSchemeBlueLight())
                .setStartPose(FieldConstantsRedMM.specimenSideStartPose)
                .build();

        DriveShim drive = myBot.getDrive();

        firstSpecimenDeliverMoveAction = drive.actionBuilder(FieldConstantsRedMM.specimenSideStartPose)
                .strafeTo(FieldConstantsRedMM.specimenDeliverApproachPose1.position)
                .strafeTo(FieldConstantsRedMM.specimenDeliverPose1.position)
                .build();//move to place first specimen

        firstSampleMoveToObservationZoneAction = drive.actionBuilder(FieldConstantsRedMM.specimenDeliverPose1)
                .strafeToLinearHeading(FieldConstantsRedMM.firstStagePushInnerPose.position, Math.toRadians(0))
                .strafeTo(FieldConstantsRedMM.secondStagePushInnerVector)
                .strafeTo(FieldConstantsRedMM.thirdStagePushInnerVector)
                .strafeTo(FieldConstantsRedMM.sample1ObservationZoneDropPose.position)
                .build();

        secondSampleMoveToObservationZoneAction = drive.actionBuilder(FieldConstantsRedMM.sample1ObservationZoneDropPose)
                .strafeToLinearHeading(FieldConstantsRedMM.firstStagePushMidPose.position, FieldConstantsRedMM.specimenPickupAngle)
                .strafeTo(FieldConstantsRedMM.thirdStagePushMidVector)
                .strafeTo(FieldConstantsRedMM.sample2ObservationZoneDropPose.position)
                .build();

        secondSpecimenPickupMoveAction = drive.actionBuilder(FieldConstantsRedMM.sample2ObservationZoneDropPose)
                .strafeTo(FieldConstantsRedMM.specimenPrePickupPose.position)
                .strafeTo(FieldConstantsRedMM.specimenPickupPose.position)
                .waitSeconds(1)
                .build();

        secondSpecimenDeliverMoveAction = drive.actionBuilder(FieldConstantsRedMM.specimenPickupPose)
                .strafeToLinearHeading(FieldConstantsRedMM.specimenDeliverApproachPose2.position, FieldConstantsRedMM.specimenDropAngle)
                .strafeTo(FieldConstantsRedMM.specimenDeliverPose2.position)
                .build();//place second specimen

        thirdSpecimenPickupMoveAction = drive.actionBuilder(FieldConstantsRedMM.specimenDeliverPose2)
                .strafeToLinearHeading(FieldConstantsRedMM.specimenPrePickupPose.position, FieldConstantsRedMM.specimenPickupAngle)
                .strafeTo(FieldConstantsRedMM.specimenPickupPose.position)
                .waitSeconds(1)
                .build();

        thirdSpecimenDeliverMoveAction = drive.actionBuilder(FieldConstantsRedMM.specimenPickupPose)
                .strafeToLinearHeading(FieldConstantsRedMM.specimenDeliverApproachPose3.position, FieldConstantsRedMM.specimenDropAngle)
                .strafeTo(FieldConstantsRedMM.specimenDeliverPose3.position)
                .build();//place second specimen

        fourthSpecimenPickupMoveAction = drive.actionBuilder(FieldConstantsRedMM.specimenDeliverPose3)
                .strafeToLinearHeading(FieldConstantsRedMM.specimenPrePickupPose.position, FieldConstantsRedMM.specimenPickupAngle)
                .strafeTo(FieldConstantsRedMM.specimenPickupPose.position)
                .waitSeconds(1)
                .build();

        fourthSpecimenDeliverMoveAction = drive.actionBuilder(FieldConstantsRedMM.specimenPickupPose)
                .strafeToLinearHeading(FieldConstantsRedMM.specimenDeliverApproachPose4.position, FieldConstantsRedMM.specimenDropAngle)
                .strafeTo(FieldConstantsRedMM.specimenDeliverPose4.position)
                .build();//place second specimen

        parkAction = drive.actionBuilder(FieldConstantsRedMM.specimenDeliverPose4)
                .strafeTo(FieldConstantsRedMM.parkPose.position)
                .build();


        deliverFourSpecimens =
                new SequentialAction(

                        firstSpecimenDeliverMoveAction,
                        firstSampleMoveToObservationZoneAction,
                        secondSampleMoveToObservationZoneAction,
                        secondSpecimenPickupMoveAction,
                        secondSpecimenDeliverMoveAction,
                        thirdSpecimenPickupMoveAction,
                        thirdSpecimenDeliverMoveAction,
                        fourthSpecimenPickupMoveAction,
                        fourthSpecimenDeliverMoveAction,
                        parkAction


                );

        myBot.runAction(deliverFourSpecimens);
        myBot.setPose(FieldConstantsRedMM.specimenSideStartPose);
        ShowField.showIt(meepMeep, myBot);
        meepMeep.start();
    }

}
