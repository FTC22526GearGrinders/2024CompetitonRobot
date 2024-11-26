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

        FieldConstantsSelect fcs;

        Action deliverFourSpecimens;


        MeepMeep meepMeep = new MeepMeep(800);
        fcs = new FieldConstantsSelect();
        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(100, 100, Math.toRadians(180), Math.toRadians(180), 15)
                .setDimensions(Constants.RobotConstants.width, Constants.RobotConstants.length)
                .setColorScheme(new ColorSchemeBlueLight())
                .setStartPose(fcs.specimenSideStartPose)
                .build();

        DriveShim drive = myBot.getDrive();

        firstSpecimenDeliverMoveAction = drive.actionBuilder(fcs.specimenSideStartPose)
                .strafeTo(fcs.specimenDeliverApproachPose1.position)
                .strafeTo(fcs.specimenDeliverPose1.position)
                .build();//move to place first specimen

        firstSampleMoveToObservationZoneAction = drive.actionBuilder(fcs.specimenDeliverPose1)
                .strafeToLinearHeading(fcs.firstStagePushInnerPose.position, Math.toRadians(180))
                .strafeTo(fcs.secondStagePushInnerVector)
                .strafeToLinearHeading(fcs.thirdStagePushInnerVector, fcs.specimenPickupAngle)
                .strafeTo(fcs.sample3ObservationZoneDropPose.position)
                .build();

        secondSpecimenPickupMoveAction = drive.actionBuilder(fcs.sample3ObservationZoneDropPose)
                .strafeToLinearHeading(fcs.specimenPickupApproachPose.position, fcs.specimenPickupAngle)
                .strafeTo(fcs.specimenPickupPose.position)
                .waitSeconds(1)
                .build();

        secondSpecimenDeliverMoveAction = drive.actionBuilder(fcs.specimenPickupPose)
                .strafeToLinearHeading(fcs.specimenDeliverApproachPose2.position, fcs.specimenDropAngle)
                .strafeTo(fcs.specimenDeliverPose2.position)
                .build();//place second specimen

        thirdSpecimenPickupMoveAction = drive.actionBuilder(fcs.specimenDeliverPose2)
                .strafeToLinearHeading(fcs.specimenPickupApproachPose.position, fcs.specimenPickupAngle)
                .strafeTo(fcs.specimenPickupPose.position)
                .waitSeconds(1)
                .build();


        thirdSpecimenDeliverMoveAction = drive.actionBuilder(fcs.specimenPickupPose)
                .strafeToLinearHeading(fcs.specimenDeliverApproachPose3.position, fcs.specimenDropAngle)
                .strafeTo(fcs.specimenDeliverPose3.position)
                .build();//place second specimen


        secondSampleMoveToObservationZoneAction = drive.actionBuilder(fcs.specimenDeliverPose3)
                .strafeToLinearHeading(fcs.firstStagePushInnerPose.position, Math.toRadians(180))
                .strafeToLinearHeading(fcs.firstStagePushMidPose.position, Math.toRadians(180))
                .strafeTo(fcs.thirdStagePushMidVector)
                .strafeTo(fcs.sample2ObservationZoneDropPose.position)
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
        myBot.setPose(fcs.specimenSideStartPose);
        ShowField.showIt(meepMeep, myBot);
        meepMeep.start();
    }

}


