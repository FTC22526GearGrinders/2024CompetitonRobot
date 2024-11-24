package com.example.meepmeeptesting;


import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueLight;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.DriveShim;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class FourBlueSpecimen {


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
                .setStartPose(FieldConstantsBlueMM.specimenSideStartPose)
                .build();

        DriveShim drive = myBot.getDrive();

        firstSpecimenDeliverMoveAction = drive.actionBuilder(FieldConstantsBlueMM.specimenSideStartPose)
                .lineToY(FieldConstantsBlueMM.specimenDeliverApproachPose1.position.y)
                .lineToY(FieldConstantsBlueMM.specimenDeliverPose1.position.y)
                .build();//move to place first specimen

        firstSampleMoveToObservationZoneAction = drive.actionBuilder(FieldConstantsBlueMM.specimenDeliverPose1)
                .strafeToLinearHeading(FieldConstantsBlueMM.firstStagePushInnerPose.position, Math.toRadians(180))
                .strafeTo(FieldConstantsBlueMM.secondStagePushInnerVector)
                .strafeTo(FieldConstantsBlueMM.thirdStagePushInnerVector)
                .strafeTo(FieldConstantsBlueMM.sample1ObservationZoneDropPose.position)
                .build();


        secondSampleMoveToObservationZoneAction = drive.actionBuilder(FieldConstantsBlueMM.sample1ObservationZoneDropPose)
                .strafeTo(FieldConstantsBlueMM.firstStagePushMidPose.position)
                .strafeTo(FieldConstantsBlueMM.secondStagePushMidVector)
                .strafeTo(FieldConstantsBlueMM.thirdStagePushMidVector)
                .strafeTo(FieldConstantsBlueMM.sample2ObservationZoneDropPose.position)
                .build();

        secondSpecimenPickupMoveAction = drive.actionBuilder(FieldConstantsBlueMM.sample2ObservationZoneDropPose)
                .lineToXLinearHeading(FieldConstantsBlueMM.specimenPrePickupPose.position.x, FieldConstantsBlueMM.specimenPickupAngle)
                .strafeTo(FieldConstantsBlueMM.specimenPickupPose.position)
                .waitSeconds(1)
                .build();

        secondSpecimenDeliverMoveAction = drive.actionBuilder(FieldConstantsBlueMM.specimenPickupPose)
                .strafeToLinearHeading(FieldConstantsBlueMM.specimenDeliverApproachPose2.position, FieldConstantsBlueMM.specimenDropAngle)
                .strafeTo(FieldConstantsBlueMM.specimenDeliverPose2.position)
                .build();//place second specimen

        thirdSpecimenPickupMoveAction = drive.actionBuilder(FieldConstantsBlueMM.specimenDeliverPose2)
                .strafeToLinearHeading(FieldConstantsBlueMM.specimenPrePickupPose.position, FieldConstantsBlueMM.specimenPickupAngle)
                .strafeTo(FieldConstantsBlueMM.specimenPickupPose.position)
                .waitSeconds(1)
                .build();

        thirdSpecimenDeliverMoveAction = drive.actionBuilder(FieldConstantsBlueMM.specimenPickupPose)
                .strafeToLinearHeading(FieldConstantsBlueMM.specimenDeliverApproachPose3.position, FieldConstantsBlueMM.specimenDropAngle)
                .strafeTo(FieldConstantsBlueMM.specimenDeliverPose3.position)
                .build();//place second specimen

        fourthSpecimenPickupMoveAction = drive.actionBuilder(FieldConstantsBlueMM.specimenDeliverPose3)
                .strafeToLinearHeading(FieldConstantsBlueMM.specimenPrePickupPose.position, FieldConstantsBlueMM.specimenPickupAngle)
                .strafeTo(FieldConstantsBlueMM.specimenPickupPose.position)
                .waitSeconds(1)
                .build();

        fourthSpecimenDeliverMoveAction = drive.actionBuilder(FieldConstantsBlueMM.specimenPickupPose)
                .strafeToLinearHeading(FieldConstantsBlueMM.specimenDeliverApproachPose4.position, FieldConstantsBlueMM.specimenDropAngle)
                .strafeTo(FieldConstantsBlueMM.specimenDeliverPose4.position)
                .build();//place second specimen

        parkAction = drive.actionBuilder(FieldConstantsBlueMM.specimenDeliverPose4)
                .strafeTo(FieldConstantsBlueMM.parkPose.position)
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
        myBot.setPose(FieldConstantsBlueMM.specimenSideStartPose);
        ShowField.showIt(meepMeep, myBot);
        meepMeep.start();
    }

}


