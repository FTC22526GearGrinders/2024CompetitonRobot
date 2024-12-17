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
import com.noahbres.meepmeep.roadrunner.DriveTrainType;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class CompetitionRedOrBlueSpecimen {


    public static void main(String[] args) {


        TrajectoryActionBuilder firstSpecimenDeliverMove;
        TrajectoryActionBuilder secondSpecimenDeliverMove;
        TrajectoryActionBuilder thirdSpecimenDeliverMove;
        TrajectoryActionBuilder fourthSpecimenDeliverMove;

        TrajectoryActionBuilder secondSpecimenPickupMove;
        TrajectoryActionBuilder thirdSpecimenPickupMove;
        TrajectoryActionBuilder fourthSpecimenPickupMove;


        TrajectoryActionBuilder firstSampleMoveToObservationZone;
        TrajectoryActionBuilder secondSampleMoveToObservationZoneApproach;
        TrajectoryActionBuilder secondSampleObservationZonePickup;

        TrajectoryActionBuilder park;

        Action elevatorMove = new SleepAction(2);

        Action sequenceOne;
        Action sequenceTwo;

        // TrajectoryActionBuilder sequenceTwo;
        TrajectoryActionBuilder sequenceThree;
        TrajectoryActionBuilder sequenceFour;
        TranslationalVelConstraint finalVel;
        ProfileAccelConstraint finalAccel;

        FieldConstantsSelect fcs;


        MeepMeep meepMeep = new MeepMeep(800);
        fcs = new FieldConstantsSelect();

        finalVel = new TranslationalVelConstraint(10.0);
        finalAccel = new ProfileAccelConstraint(-20.0, 20.0);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                .setDriveTrainType((DriveTrainType.MECANUM))
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .setDimensions(Constants.RobotConstants.width, Constants.RobotConstants.length)
                .setColorScheme(new ColorSchemeBlueLight())
                .setStartPose(fcs.specimenSideStartPose)
                .build();

        DriveShim drive = myBot.getDrive();


        //     fcs.setRed();

        firstSpecimenDeliverMove = drive.actionBuilder(fcs.specimenSideStartPose)
                .strafeToLinearHeading(fcs.specimenDeliverApproachPose1.position, fcs.specimenDropAngle)
                .strafeTo(fcs.specimenDeliverPose1.position,
                        finalVel, finalAccel
                );

        firstSampleMoveToObservationZone = drive.actionBuilder(fcs.specimenDeliverPose1)
                .strafeToLinearHeading(fcs.firstStagePushInnerPose.position, Math.toRadians(180))
                .strafeToLinearHeading(fcs.secondStagePushInnerVector, Math.toRadians(180))
                .strafeToLinearHeading(fcs.thirdStagePushInnerVector, Math.toRadians(180))
                .strafeToLinearHeading(fcs.sample1ObservationZoneDropPose.position, Math.toRadians(180));


        secondSampleMoveToObservationZoneApproach = drive.actionBuilder(fcs.sample1ObservationZoneDropPose)
                .strafeToLinearHeading(fcs.secondStagePushInnerVector, fcs.specimenPickupAngle)
                .strafeToLinearHeading(fcs.thirdStagePushMidVector, fcs.specimenPickupAngle)
                .strafeToLinearHeading(fcs.sample2ObservationZoneDropPose.position, fcs.specimenPickupAngle);

        secondSampleObservationZonePickup = drive.actionBuilder(fcs.sample2ObservationZoneDropPose)
                .strafeToLinearHeading(fcs.sample2ObservationZonePickupPose.position, fcs.specimenPickupAngle,
                        finalVel, finalAccel
                );

        secondSpecimenDeliverMove = drive.actionBuilder(fcs.sample2ObservationZonePickupPose)
                .splineToLinearHeading(fcs.specimenDeliverApproachPose2, fcs.specimenPickupAngle)
                .strafeToLinearHeading(fcs.specimenDeliverPose2.position, fcs.specimenDropAngle,
                        finalVel, finalAccel
                );


        thirdSpecimenPickupMove = drive.actionBuilder(fcs.specimenDeliverPose2)
                .splineToLinearHeading(fcs.specimenPickupApproachPose, fcs.specimenDropAngle)
                .strafeToLinearHeading(fcs.specimenPickupPose.position, fcs.specimenPickupAngle,
                        finalVel, finalAccel
                );

        thirdSpecimenDeliverMove = drive.actionBuilder(fcs.specimenPickupPose)
                .splineToLinearHeading(fcs.specimenDeliverApproachPose3, fcs.specimenPickupAngle)
                .strafeToLinearHeading(fcs.specimenDeliverPose3.position, fcs.specimenDropAngle,
                        finalVel, finalAccel
                );

        fourthSpecimenPickupMove = drive.actionBuilder(fcs.specimenDeliverPose3)
                .splineToLinearHeading(fcs.specimenPickupApproachPose, fcs.specimenDropAngle)
                .strafeToLinearHeading(fcs.specimenPickupPose.position, fcs.specimenPickupAngle,
                        finalVel, finalAccel
                );

        fourthSpecimenDeliverMove = drive.actionBuilder(fcs.specimenPickupPose)
                .splineToLinearHeading(fcs.specimenDeliverApproachPose4, fcs.specimenPickupAngle)
                .strafeToLinearHeading(fcs.specimenDeliverPose4.position, fcs.specimenDropAngle,
                        finalVel, finalAccel
                );

        park = drive.actionBuilder(fcs.specimenDeliverPose4)
                .strafeToLinearHeading(fcs.specimenParkPose.position, fcs.specimenDropAngle);


        sequenceOne = new SequentialAction(

                new ParallelAction(
                        firstSpecimenDeliverMove.build(),
                        elevatorMove),

                new ParallelAction(
                        firstSampleMoveToObservationZone.build(),
                        elevatorMove),


                secondSampleMoveToObservationZoneApproach.build(),

                secondSampleObservationZonePickup.build(),

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


