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

public class CompetitionRedOrBlueSpecimen {


    public static void main(String[] args) {


        Action firstSpecimenDeliverApproachMoveAction;
        Action secondSpecimenDeliverApproachMoveAction;
        Action thirdSpecimenDeliverApproachMoveAction;
        Action fourthSpecimenDeliverApproachMoveAction;

        Action firstSpecimenDeliverMoveAction;
        Action secondSpecimenDeliverMoveAction;
        Action thirdSpecimenDeliverMoveAction;
        Action fourthSpecimenDeliverMoveAction;

        Action firstSpecimenPickupApproachMoveAction;
        Action secondSpecimenPickupApproachMoveAction;
        Action thirdSpecimenPickupApproachMoveAction;
        Action fourthSpecimenPickupApproachMoveAction;

        Action specimenPickuoMoveAction;

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
                .setConstraints(60, 100, Math.toRadians(180), Math.toRadians(180), 15)
                .setDimensions(Constants.RobotConstants.width, Constants.RobotConstants.length)
                .setColorScheme(new ColorSchemeBlueLight())
                .setStartPose(fcs.specimenSideStartPose)
                .build();

        DriveShim drive = myBot.getDrive();


        firstSpecimenDeliverApproachMoveAction = drive.actionBuilder(fcs.specimenSideStartPose)
                .strafeToLinearHeading(fcs.specimenDeliverApproachPose1.position, fcs.specimenDropAngle)
                .build();//move to place first specimen

        firstSpecimenDeliverMoveAction = drive.actionBuilder(fcs.specimenDeliverApproachPose1)
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


        secondSpecimenPickupApproachMoveAction = drive.actionBuilder(fcs.sample2ObservationZoneDropPose)
                .strafeToLinearHeading(fcs.specimenPickupApproachPose.position, fcs.specimenPickupAngle)
                .build();

        secondSpecimenDeliverApproachMoveAction = drive.actionBuilder(fcs.specimenPickupPose)
                .strafeToLinearHeading(fcs.specimenDeliverApproachPose2.position, fcs.specimenDropAngle)
                .build();//move to place first specimen

        secondSpecimenDeliverMoveAction = drive.actionBuilder(fcs.specimenDeliverApproachPose2)
                .strafeToLinearHeading(fcs.specimenDeliverPose2.position, fcs.specimenDropAngle)
                .build();//move to place first specimen

        thirdSpecimenPickupApproachMoveAction = drive.actionBuilder(fcs.specimenDeliverPose2)
                .strafeToLinearHeading(fcs.specimenPickupApproachPose.position, fcs.specimenPickupAngle)
                .build();


        thirdSpecimenDeliverApproachMoveAction = drive.actionBuilder(fcs.specimenPickupPose)
                .strafeToLinearHeading(fcs.specimenDeliverApproachPose3.position, fcs.specimenDropAngle)
                .build();//move to place first specimen

        thirdSpecimenDeliverMoveAction = drive.actionBuilder(fcs.specimenDeliverApproachPose3)
                .strafeToLinearHeading(fcs.specimenDeliverPose3.position, fcs.specimenDropAngle)
                .build();//move to place first specimen

        fourthSpecimenPickupApproachMoveAction = drive.actionBuilder(fcs.specimenDeliverPose3)
                .strafeToLinearHeading(fcs.specimenPickupApproachPose.position, fcs.specimenPickupAngle)
                .waitSeconds(1)
                .build();

        fourthSpecimenDeliverApproachMoveAction = drive.actionBuilder(fcs.specimenPickupPose)
                .strafeToLinearHeading(fcs.specimenDeliverApproachPose4.position, fcs.specimenDropAngle)
                .build();//move to place first specimen

        fourthSpecimenDeliverMoveAction = drive.actionBuilder(fcs.specimenDeliverApproachPose4)
                .strafeToLinearHeading(fcs.specimenDeliverPose4.position, fcs.specimenDropAngle)
                .build();//move to place first specimen

        parkAction = drive.actionBuilder(fcs.specimenDeliverPose4)
                .strafeTo(fcs.specimenParkPose.position)
                .build();


        specimenPickuoMoveAction = drive.actionBuilder(fcs.specimenPickupApproachPose)
                .strafeToLinearHeading(fcs.specimenPickupPose.position, fcs.specimenPickupAngle)
                .build();


        sequenceOne =
                new SequentialAction(

                        new ParallelAction(
                                firstSpecimenDeliverApproachMoveAction,
                                elevatorMove),
                        firstSpecimenDeliverMoveAction,
                        //   ears.deliverSpecimenToUpperSubmersible(),
                        new SleepAction(1),
                        new ParallelAction(
                                firstSampleMoveToObservationZoneAction,
                                elevatorMove),
                        secondSampleMoveToObservationZoneAction,

                        new ParallelAction(
                                secondSpecimenPickupApproachMoveAction,
                                elevatorMove),
                        specimenPickuoMoveAction,
                        new SleepAction(1),

                        new ParallelAction(
                                secondSpecimenDeliverApproachMoveAction,
                                elevatorMove),
                        secondSpecimenDeliverMoveAction,
                        new SleepAction(1),

//                        new ParallelAction(
//                                thirdSpecimenPickupApproachMoveAction,
//                                elevatorMove),
                        specimenPickuoMoveAction,
                        new SleepAction(1),


                        new ParallelAction(
                                thirdSpecimenDeliverApproachMoveAction,
                                elevatorMove),
                        thirdSpecimenDeliverMoveAction,
                        new SleepAction(1),

                        new ParallelAction(
                                fourthSpecimenPickupApproachMoveAction,
                                elevatorMove),
                        specimenPickuoMoveAction,
                        new SleepAction(1),

                        new ParallelAction(
                                fourthSpecimenDeliverApproachMoveAction,
                                elevatorMove),
                        fourthSpecimenDeliverMoveAction,
                        new SleepAction(1),

                        parkAction
                );

        myBot.runAction(sequenceOne);
        myBot.setPose(fcs.specimenSideStartPose);
        ShowField.showIt(meepMeep, myBot);
        meepMeep.start();
    }


}


