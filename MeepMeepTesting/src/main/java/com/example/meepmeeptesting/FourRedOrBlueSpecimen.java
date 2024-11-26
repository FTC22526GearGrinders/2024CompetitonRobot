package com.example.meepmeeptesting;


import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueLight;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.DriveShim;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class FourRedOrBlueSpecimen {


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

        FieldConstantsSelect fcs;

        Action deliverFourSpecimens;


        MeepMeep meepMeep = new MeepMeep(800);
        fcs = new FieldConstantsSelect();
        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 100, Math.toRadians(180), Math.toRadians(180), 15)
                .setDimensions(Constants.RobotConstants.width, Constants.RobotConstants.length)
                .setColorScheme(new ColorSchemeBlueLight())
                .setStartPose(fcs.specimenSideStartPose)
                .build();

        DriveShim drive = myBot.getDrive();

        firstSpecimenDeliverMoveAction = drive.actionBuilder(fcs.specimenSideStartPose)
                .strafeToLinearHeading(fcs.specimenDeliverApproachPose1.position, fcs.specimenDropAngle)
                .strafeToLinearHeading(fcs.specimenDeliverPose1.position, fcs.specimenDropAngle)
                .build();//move to place first specimen

        firstSampleMoveToObservationZoneAction = drive.actionBuilder(fcs.specimenDeliverPose1)
                .strafeToLinearHeading(fcs.firstStagePushInnerPose.position, Math.toRadians(180))
                .strafeTo(fcs.secondStagePushInnerVector)
                .strafeTo(fcs.thirdStagePushInnerVector)
                .strafeTo(fcs.sample1ObservationZoneDropPose.position)
                .build();


        secondSampleMoveToObservationZoneAction = drive.actionBuilder(fcs.sample1ObservationZoneDropPose)
                .strafeToLinearHeading(fcs.firstStagePushMidPose.position, fcs.specimenPickupAngle)
                .strafeTo(fcs.secondStagePushMidVector)
                .strafeTo(fcs.thirdStagePushMidVector)
                .strafeToLinearHeading(fcs.sample2ObservationZoneDropPose.position, fcs.specimenPickupAngle)
                .build();

        secondSpecimenPickupMoveAction = drive.actionBuilder(fcs.sample2ObservationZoneDropPose)
                .strafeToLinearHeading(fcs.specimenPickupApproachPose.position, fcs.specimenPickupAngle)
                .strafeToLinearHeading(fcs.specimenPickupPose.position, fcs.specimenPickupAngle)
                .waitSeconds(1)
                .build();

        secondSpecimenDeliverMoveAction = drive.actionBuilder(fcs.specimenPickupPose)
                .strafeToLinearHeading(fcs.specimenDeliverApproachPose2.position, fcs.specimenDropAngle)
                .strafeToLinearHeading(fcs.specimenDeliverPose2.position, fcs.specimenDropAngle)
                .build();//place second specimen

        thirdSpecimenPickupMoveAction = drive.actionBuilder(fcs.specimenDeliverPose2)
                .strafeToLinearHeading(fcs.specimenPickupApproachPose.position, fcs.specimenPickupAngle)
                .strafeToLinearHeading(fcs.specimenPickupPose.position, fcs.specimenPickupAngle)
                .waitSeconds(1)
                .build();

        thirdSpecimenDeliverMoveAction = drive.actionBuilder(fcs.specimenPickupPose)
                .strafeToLinearHeading(fcs.specimenDeliverApproachPose3.position, fcs.specimenDropAngle)
                .strafeToLinearHeading(fcs.specimenDeliverPose3.position, fcs.specimenDropAngle)
                .build();//place second specimen

        fourthSpecimenPickupMoveAction = drive.actionBuilder(fcs.specimenDeliverPose3)
                .strafeToLinearHeading(fcs.specimenPickupApproachPose.position, fcs.specimenPickupAngle)
                .strafeToLinearHeading(fcs.specimenPickupPose.position, fcs.specimenPickupAngle)
                .waitSeconds(1)
                .build();

        fourthSpecimenDeliverMoveAction = drive.actionBuilder(fcs.specimenPickupPose)
                .strafeToLinearHeading(fcs.specimenDeliverApproachPose4.position, fcs.specimenDropAngle)
                .strafeToLinearHeading(fcs.specimenDeliverPose3.position, fcs.specimenDropAngle)
                .build();//place second specimen

        parkAction = drive.actionBuilder(fcs.specimenDeliverPose4)
                .strafeTo(fcs.specimenParkPose.position)
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
        myBot.setPose(fcs.specimenSideStartPose);
        ShowField.showIt(meepMeep, myBot);
        meepMeep.start();
    }

}


