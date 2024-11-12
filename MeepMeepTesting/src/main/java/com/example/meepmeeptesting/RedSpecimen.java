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
                .setConstraints(100, 100, Math.toRadians(180), Math.toRadians(180), 15)
                .setDimensions(Constants.RobotConstants.width, Constants.RobotConstants.length)
                .setColorScheme(new ColorSchemeBlueLight())
                .setStartPose(FieldConstantsRedMM.specimenSideStartPose)
                .build();

        DriveShim drive = myBot.getDrive();

        firstSpecimenDeliverMoveAction = drive.actionBuilder(FieldConstantsRedMM.specimenSideStartPose)
                .lineToY(FieldConstantsRedMM.specimenDeliverApproachPose1.position.y)
                .lineToY(FieldConstantsRedMM.specimenDeliverPose1.position.y)
                .build();//move to place first specimen

        firstSampleMoveToObservationZoneAction = drive.actionBuilder(FieldConstantsRedMM.specimenDeliverPose1)
                .splineToSplineHeading(new Pose2d(-36, 42, Math.toRadians(180)), Math.toRadians(180))
                .strafeTo(new Vector2d(-36, 10))
                .strafeTo(new Vector2d(-48, 10))
                .strafeTo(FieldConstantsRedMM.sample1ObservationZoneDropPose.position)
                .build();


        secondSampleMoveToObservationZoneAction = drive.actionBuilder(FieldConstantsRedMM.sample1ObservationZoneDropPose)
                .splineToSplineHeading(new Pose2d(-36, 10, Math.toRadians(180)), Math.toRadians(180))
                .strafeTo(new Vector2d(-56, 10))
                .strafeTo(FieldConstantsRedMM.sample2ObservationZoneDropPose.position)
                .build();

        secondSpecimenPickupMoveAction = drive.actionBuilder(FieldConstantsRedMM.sample2ObservationZoneDropPose)
                .splineToLinearHeading(FieldConstantsRedMM.specimenPrePickupPose, Math.toRadians(-100))
                .strafeTo(FieldConstantsRedMM.specimenPickupPose.position)
                .waitSeconds(1)
                .build();

        secondSpecimenDeliverMoveAction = drive.actionBuilder(FieldConstantsRedMM.specimenPickupPose)
                .splineToLinearHeading(FieldConstantsRedMM.specimenDeliverApproachPose2, Math.toRadians(-90))
                .lineToY(FieldConstantsRedMM.specimenDeliverPose2.position.y)
                .build();//place second specimen

        thirdSpecimenPickupMoveAction = drive.actionBuilder(FieldConstantsRedMM.specimenDeliverPose2)
                .splineToLinearHeading(FieldConstantsRedMM.specimenPrePickupPose, Math.toRadians(180))
                .strafeTo(FieldConstantsRedMM.specimenPickupPose.position)
                .waitSeconds(1)
                .build();

        thirdSpecimenDeliverMoveAction = drive.actionBuilder(FieldConstantsRedMM.specimenPickupPose)
                .splineToLinearHeading(FieldConstantsRedMM.specimenDeliverApproachPose3, Math.toRadians(-90))
                .lineToY(FieldConstantsRedMM.specimenDeliverPose3.position.y)
                .build();//place second specimen

        fourthSpecimenPickupMoveAction = drive.actionBuilder(FieldConstantsRedMM.specimenDeliverPose3)
                .lineToY(FieldConstantsRedMM.specimenDeliverApproachPose3.position.y)
                .splineToLinearHeading(FieldConstantsRedMM.specimenPrePickupPose, Math.toRadians(180))
                .strafeTo(FieldConstantsRedMM.specimenPickupPose.position)
                .waitSeconds(1)
                .build();

        fourthSpecimenDeliverMoveAction = drive.actionBuilder(FieldConstantsRedMM.specimenPickupPose)
                .splineToLinearHeading(FieldConstantsRedMM.specimenDeliverApproachPose4, Math.toRadians(-90))
                .lineToY(FieldConstantsRedMM.specimenDeliverPose4.position.y)
                .build();//place second specimen


        deliverFourSpecimens =
                new SequentialAction(

                        firstSpecimenDeliverMoveAction,
                        placeSpecimenAction,
                        firstSampleMoveToObservationZoneAction,
                        secondSampleMoveToObservationZoneAction,
                        secondSpecimenPickupMoveAction,
                        secondSpecimenDeliverMoveAction,
                        thirdSpecimenPickupMoveAction,
                        thirdSpecimenDeliverMoveAction,
                        fourthSpecimenPickupMoveAction,
                        fourthSpecimenDeliverMoveAction,
//park
                        secondSampleMoveToObservationZoneAction

                );

        myBot.runAction(deliverFourSpecimens);
        myBot.setPose(FieldConstantsRedMM.specimenSideStartPose);
        ShowField.showIt(meepMeep, myBot);
        meepMeep.start();
    }

}
