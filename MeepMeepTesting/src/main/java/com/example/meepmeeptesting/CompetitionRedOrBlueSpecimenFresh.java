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

public class CompetitionRedOrBlueSpecimenFresh {


    public static void main(String[] args) {

        Action elevatorMove = new SleepAction(2);

        Action sequenceOne;


        TrajectoryActionBuilder sequenceThree;
        TranslationalVelConstraint approachVel;
        ProfileAccelConstraint approachAccel;

        FieldConstantsSelect fcs;


        MeepMeep meepMeep = new MeepMeep(800);
        fcs = new FieldConstantsSelect();

        approachVel = new TranslationalVelConstraint(10.0);
        approachAccel = new ProfileAccelConstraint(-20.0, 20.0);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                .setDriveTrainType((DriveTrainType.MECANUM))
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(80, 20, Math.toRadians(180), Math.toRadians(180), 15)
                .setDimensions(Constants.RobotConstants.width, Constants.RobotConstants.length)
                .setColorScheme(new ColorSchemeBlueLight())
                .setStartPose(fcs.specimenSideStartPose)
                .build();

        DriveShim drive = myBot.getDrive();


        // fcs.setRed();

        Action firstSpecimenDeliverMove = drive.actionBuilder(fcs.specimenSideStartPose)
                .strafeToLinearHeading(fcs.specimenDeliverApproachPose1.position, fcs.specimenDropAngle)
                .strafeTo(fcs.specimenDeliverPose1.position,
                        approachVel, approachAccel
                ).build();

        TrajectoryActionBuilder firstSampleMoveToObservationZone = drive.actionBuilder(fcs.specimenDeliverPose1)
                //   .turn(Math.toRadians(90))
                .strafeToLinearHeading(fcs.firstStagePushInnerPose.position, Math.toRadians(180))
                .strafeToLinearHeading(fcs.secondStagePushInnerVector, Math.toRadians(180))
                .strafeToLinearHeading(fcs.thirdStagePushInnerVector, Math.toRadians(-90))
                .strafeToLinearHeading(fcs.sample2ObservationZonePickupPose.position, fcs.specimenPickupAngle);
        //  .strafeToLinearHeading(fcs.sample1ObservationZoneDropPose.position, Math.toRadians(180));

//        TrajectoryActionBuilder secondSampleMoveToObservationZoneApproach = drive.actionBuilder(fcs.sample1ObservationZoneDropPose)
//                .strafeToLinearHeading(fcs.secondStagePushMidVector, fcs.specimenPickupAngle)
//                .strafeToLinearHeading(fcs.thirdStagePushMidVector, fcs.specimenPickupAngle)
//                .strafeToLinearHeading(fcs.sample2ObservationZoneDropPose.position, fcs.specimenPickupAngle);
//
//
//        Action secondSampleApproachCloseout = secondSampleMoveToObservationZoneApproach.endTrajectory().fresh()
//                .strafeToLinearHeading(fcs.sample2ObservationZonePickupPose.position, fcs.specimenPickupAngle,
//                        approachVel, approachAccel).build();
//

        TrajectoryActionBuilder secondSpecimenDeliverMove = drive.actionBuilder(fcs.sample2ObservationZonePickupPose)
                .splineToLinearHeading(fcs.specimenDeliverApproachPose2, fcs.specimenPickupAngle);


        Action secondSpecimenDeliverCloseOut = secondSpecimenDeliverMove.endTrajectory().fresh()
                .strafeToLinearHeading(fcs.specimenDeliverPose2.position, fcs.specimenDropAngle,
                        approachVel, approachAccel).build();


        TrajectoryActionBuilder thirdSpecimenPickupMove = drive.actionBuilder(fcs.specimenDeliverPose2)
                .splineToLinearHeading(fcs.specimenPickupApproachPose, fcs.specimenDropAngle);


        Action thirdSpecimenPickupCloseout = thirdSpecimenPickupMove.endTrajectory().fresh()
                .strafeToLinearHeading(fcs.specimenPickupPose.position, fcs.specimenPickupAngle,
                        approachVel, approachAccel).build();


        TrajectoryActionBuilder thirdSpecimenDeliverMove = drive.actionBuilder(fcs.specimenPickupPose)
                .splineToLinearHeading(fcs.specimenDeliverApproachPose3, fcs.specimenPickupAngle);

        Action thirdSpecimenDeliverCloseout = thirdSpecimenDeliverMove.endTrajectory().fresh()
                .strafeToLinearHeading(fcs.specimenDeliverPose3.position, fcs.specimenDropAngle,
                        approachVel, approachAccel).build();

        TrajectoryActionBuilder fourthSpecimenPickupMove = drive.actionBuilder(fcs.specimenDeliverPose3)
                .splineToLinearHeading(fcs.specimenPickupApproachPose, fcs.specimenDropAngle);

        Action fourthSpecimenPickupCloseout = fourthSpecimenPickupMove.endTrajectory().fresh()
                .strafeToLinearHeading(fcs.specimenPickupPose.position, fcs.specimenPickupAngle,
                        approachVel, approachAccel
                ).build();


        TrajectoryActionBuilder fourthSpecimenDeliverMove = drive.actionBuilder(fcs.specimenPickupPose)
                .splineToLinearHeading(fcs.specimenDeliverApproachPose4, fcs.specimenPickupAngle);

        Action fourthSpecimenDeliverCloseout = fourthSpecimenDeliverMove.endTrajectory().fresh()
                .strafeToLinearHeading(fcs.specimenDeliverPose4.position, fcs.specimenDropAngle,
                        approachVel, approachAccel).build();

        TrajectoryActionBuilder park = drive.actionBuilder(fcs.specimenDeliverPose4)
                .strafeTo(fcs.specimenParkPose.position);


        sequenceOne = new SequentialAction(

                new ParallelAction(
                        firstSpecimenDeliverMove,
                        elevatorMove),

                new ParallelAction(
                        firstSampleMoveToObservationZone.build(),
                        elevatorMove),

//                secondSampleMoveToObservationZoneApproach.build(),
//
//                secondSampleApproachCloseout,

                new SleepAction(1),

                new ParallelAction(
                        secondSpecimenDeliverMove.build(),

                        elevatorMove),
                secondSpecimenDeliverCloseOut,
                new SleepAction(1),

                new ParallelAction(
                        thirdSpecimenPickupMove.build(),
                        elevatorMove),
                new SleepAction(1),
                thirdSpecimenPickupCloseout,

                new ParallelAction(
                        thirdSpecimenDeliverMove.build(),
                        elevatorMove),
                new SleepAction(1),
                thirdSpecimenDeliverCloseout,
                new ParallelAction(
                        fourthSpecimenPickupMove.build(),
                        elevatorMove),
                fourthSpecimenPickupCloseout,
                new SleepAction(1),

                new ParallelAction(
                        fourthSpecimenDeliverMove.build(),
                        elevatorMove),
                fourthSpecimenDeliverCloseout,
                new SleepAction(1),

                park.build());


        myBot.runAction(sequenceOne);
        myBot.setPose(fcs.specimenSideStartPose);
        ShowField.showIt(meepMeep, myBot);
        meepMeep.start();
    }


}


