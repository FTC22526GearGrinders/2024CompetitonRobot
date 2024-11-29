package com.example.meepmeeptesting;


import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueLight;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.DriveShim;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class CompetitionRedOrBlueSpecimenFastTAB {


    public static void main(String[] args) {


        TrajectoryActionBuilder firstSpecimenDeliverMove;
        TrajectoryActionBuilder secondSpecimenDeliverMove;
        TrajectoryActionBuilder thirdSpecimenDeliverMove;
        TrajectoryActionBuilder fourthSpecimenDeliverMove;

        TrajectoryActionBuilder secondSpecimenPickupMove;
        TrajectoryActionBuilder thirdSpecimenPickupMove;
        TrajectoryActionBuilder fourthSpecimenPickupMove;


        TrajectoryActionBuilder firstSampleMoveToObservationZone;
        TrajectoryActionBuilder secondSampleMoveToObservationZonePickup;

        TrajectoryActionBuilder park;

        Action elevatorMove = new SleepAction(2);

        Action sequenceOne;
        TrajectoryActionBuilder sequenceTwo;
        TrajectoryActionBuilder sequenceThree;
        TrajectoryActionBuilder sequenceFour;
        TranslationalVelConstraint approachVel;
        ProfileAccelConstraint approachAccel;

        FieldConstantsSelect fcs;


        MeepMeep meepMeep = new MeepMeep(800);
        fcs = new FieldConstantsSelect();

        approachVel = new TranslationalVelConstraint(10.0);
        approachAccel = new ProfileAccelConstraint(-20.0, 20.0);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .setDimensions(Constants.RobotConstants.width, Constants.RobotConstants.length)
                .setColorScheme(new ColorSchemeBlueLight())
                .setStartPose(fcs.specimenSideStartPose)
                .build();

        DriveShim drive = myBot.getDrive();


        fcs.setRed();

        firstSpecimenDeliverMove = drive.actionBuilder(fcs.specimenSideStartPose)
                .strafeToLinearHeading(fcs.specimenDeliverApproachPose1.position, fcs.specimenDropAngle)
                .strafeTo(fcs.specimenDeliverPose1.position,
                        approachVel, approachAccel
                );

        firstSampleMoveToObservationZone = drive.actionBuilder(fcs.specimenDeliverPose1)
                .strafeToLinearHeading(fcs.firstStagePushInnerPose.position, Math.toRadians(180))
                .strafeTo(fcs.secondStagePushInnerVector)
                .strafeTo(fcs.thirdStagePushInnerVector)
                .strafeTo(fcs.sample1ObservationZoneDropPose.position);


        secondSampleMoveToObservationZonePickup = drive.actionBuilder(fcs.sample1ObservationZoneDropPose)
                .strafeToLinearHeading(fcs.secondStagePushInnerVector, fcs.specimenDropAngle)
                .strafeTo(fcs.thirdStagePushMidVector)
                .strafeTo(fcs.sample2ObservationZoneApproachPoseFast.position)
                .strafeTo(fcs.sample2ObservationZonePoseFast.position,
                        approachVel, approachAccel
                );

        secondSpecimenDeliverMove = drive.actionBuilder(fcs.sample2ObservationZonePoseFast)
                // .splineToSplineHeading(fcs.specimenDeliverPose2,fcs.specimenPickupAngle)
                .splineToLinearHeading(fcs.specimenDeliverApproachPose2, fcs.specimenPickupAngle)
                .strafeTo(fcs.specimenDeliverPose2.position,
                        approachVel, approachAccel
                );


        thirdSpecimenPickupMove = drive.actionBuilder(fcs.specimenDeliverPose2)
                .splineToLinearHeading(fcs.specimenPickupApproachPose, fcs.specimenDropAngle)
                .strafeTo(fcs.specimenPickupPose.position,
                        approachVel, approachAccel
                );

        thirdSpecimenDeliverMove = drive.actionBuilder(fcs.specimenPickupPose)
                .splineToLinearHeading(fcs.specimenDeliverApproachPose3, fcs.specimenPickupAngle)
                .strafeTo(fcs.specimenDeliverPose3.position,
                        approachVel, approachAccel
                );

        fourthSpecimenPickupMove = drive.actionBuilder(fcs.specimenDeliverPose3)
                .splineToLinearHeading(fcs.specimenPickupApproachPose, fcs.specimenDropAngle)
                .strafeTo(fcs.specimenPickupPose.position,
                        approachVel, approachAccel
                );

        fourthSpecimenDeliverMove = drive.actionBuilder(fcs.specimenPickupPose)
                .splineToLinearHeading(fcs.specimenDeliverApproachPose4, fcs.specimenPickupAngle)
                .strafeTo(fcs.specimenDeliverPose4.position,
                        approachVel, approachAccel
                );

        park = drive.actionBuilder(fcs.specimenDeliverPose4)
                .strafeTo(fcs.specimenParkPose.position);


        sequenceOne = new SequentialAction(

                new ParallelAction(
                        firstSpecimenDeliverMove.build(),
                        elevatorMove),

                new ParallelAction(
                        firstSampleMoveToObservationZone.build(),
                        elevatorMove),
                secondSampleMoveToObservationZonePickup.build(),

                new SleepAction(1),

                new ParallelAction(
                        secondSpecimenDeliverMove.build(),
                        elevatorMove),

                new SleepAction(1),

                new ParallelAction(
                        thirdSpecimenPickupMove.build(),
                        elevatorMove),
                new SleepAction(1),


                new ParallelAction(
                        thirdSpecimenDeliverMove.build(),
                        elevatorMove),
                new SleepAction(1),

                new ParallelAction(
                        fourthSpecimenPickupMove.build(),
                        elevatorMove),

                new SleepAction(1),

                new ParallelAction(
                        fourthSpecimenDeliverMove.build(),
                        elevatorMove),

                new SleepAction(1),

                park.build());


        myBot.runAction(sequenceOne);
        myBot.setPose(fcs.specimenSideStartPose);
        ShowField.showIt(meepMeep, myBot);
        meepMeep.start();
    }


}


