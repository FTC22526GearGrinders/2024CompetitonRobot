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

public class CompetitionRedOrBlueSpecimenFast {


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

        Action elevatorMove = new SleepAction(2);

        Action sequenceOne;
        Action sequenceTwo;
        Action sequenceThree;
        Action sequenceFour;


        FieldConstantsSelect fcs;


        MeepMeep meepMeep = new MeepMeep(800);
        fcs = new FieldConstantsSelect();
        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .setDimensions(Constants.RobotConstants.width, Constants.RobotConstants.length)
                .setColorScheme(new ColorSchemeBlueLight())
                .setStartPose(fcs.specimenSideStartPose)
                .build();

        DriveShim drive = myBot.getDrive();


        fcs.setRed();

        firstSpecimenDeliverMoveAction = drive.actionBuilder(fcs.specimenSideStartPose)
                .strafeToLinearHeading(fcs.specimenDeliverPose1.position, fcs.specimenDropAngle)
                .build();//move to place first specimen

        firstSampleMoveToObservationZoneAction = drive.actionBuilder(fcs.specimenDeliverPose1)
                .strafeToLinearHeading(fcs.firstStagePushInnerPose.position, Math.toRadians(180))
                .strafeTo(fcs.secondStagePushInnerVector)
                .strafeTo(fcs.thirdStagePushInnerVector)
                .strafeTo(fcs.sample1ObservationZoneDropPose.position)
                .build();

        secondSampleMoveToObservationZoneAction = drive.actionBuilder(fcs.sample1ObservationZoneDropPose)
                .strafeToLinearHeading(fcs.secondStagePushInnerVector, fcs.specimenPickupAngle)
                .strafeTo(fcs.thirdStagePushMidVector)
                .strafeTo(fcs.sample2ObservationZoneDropPoseFast.position)
                .build();

        secondSpecimenDeliverMoveAction = drive.actionBuilder(fcs.sample2ObservationZoneDropPoseFast)
                // .splineToSplineHeading(fcs.specimenDeliverPose2,fcs.specimenPickupAngle)
                .splineToLinearHeading(fcs.specimenDeliverPose2, Math.toRadians(90))
                .build();//move to place first specimen

        thirdSpecimenPickupMoveAction = drive.actionBuilder(fcs.specimenDeliverPose2)
                //.splineToSplineHeading(fcs.specimenPickupPose, fcs.specimenDropAngle)
                .splineToLinearHeading(fcs.specimenPickupPose, Math.toRadians(-90))
                .build();

        thirdSpecimenDeliverMoveAction = drive.actionBuilder(fcs.specimenPickupPose)
                //.splineToSplineHeading(fcs.specimenDeliverPose3, fcs.specimenPickupAngle)
                .splineToLinearHeading(fcs.specimenDeliverPose3, Math.toRadians(90))
                .build();//move to place first specimen

        fourthSpecimenPickupMoveAction = drive.actionBuilder(fcs.specimenDeliverPose3)
                .splineToLinearHeading(fcs.specimenPickupPose, Math.toRadians(-90))
                .build();

        fourthSpecimenDeliverMoveAction = drive.actionBuilder(fcs.specimenPickupPose)
                .splineToLinearHeading(fcs.specimenDeliverPose4, Math.toRadians(90))
                .build();//move to place first specimen

        parkAction = drive.actionBuilder(fcs.specimenDeliverPose4)
                .strafeTo(fcs.specimenParkPose.position)
                .build();


        sequenceOne =
                new SequentialAction(

                        new ParallelAction(
                                firstSpecimenDeliverMoveAction,
                                elevatorMove),

                        new ParallelAction(
                                firstSampleMoveToObservationZoneAction,
                                elevatorMove),
                        secondSampleMoveToObservationZoneAction,
//                        new ParallelAction(
//                                secondSpecimenPickupMoveAction,
//                                elevatorMove),
                        new SleepAction(1),

                        new ParallelAction(
                                secondSpecimenDeliverMoveAction,
                                elevatorMove),

                        new SleepAction(1),

                        new ParallelAction(
                                thirdSpecimenPickupMoveAction,
                                elevatorMove),
                        new SleepAction(1),


                        new ParallelAction(
                                thirdSpecimenDeliverMoveAction,
                                elevatorMove),
                        new SleepAction(1),

                        new ParallelAction(
                                fourthSpecimenPickupMoveAction,
                                elevatorMove),

                        new SleepAction(1),

                        new ParallelAction(
                                fourthSpecimenDeliverMoveAction,
                                elevatorMove),

                        new SleepAction(1),

                        parkAction
                );

        myBot.runAction(sequenceOne);
        myBot.setPose(fcs.specimenSideStartPose);
        ShowField.showIt(meepMeep, myBot);
        meepMeep.start();
    }


}


