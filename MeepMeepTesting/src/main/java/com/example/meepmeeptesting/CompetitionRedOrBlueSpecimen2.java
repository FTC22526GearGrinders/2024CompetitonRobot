package com.example.meepmeeptesting;


import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueLight;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.DriveShim;
import com.noahbres.meepmeep.roadrunner.DriveTrainType;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class CompetitionRedOrBlueSpecimen2 {


    public static void main(String[] args) {

        Action elevatorMove = new SleepAction(2);

        FieldConstantsSelect fcs;
        Action firstSpecimenDeliverMove;
        Action firstSampleMoveToObservationZone;
        Action secondSampleMoveToObservationZone;

        Action secondSpecimenDeliverMove;

        Action thirdSpecimenPickupMove;

        Action thirdSpecimenDeliverMove;

        Action fourthSpecimenDeliverMove;


        Action fourthSpecimenPickupMove;

        Action sequenceOne;

        Action park;


        MeepMeep meepMeep = new MeepMeep(800);
        fcs = new FieldConstantsSelect();


        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                .setDriveTrainType((DriveTrainType.MECANUM))
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .setDimensions(Constants.RobotConstants.width, Constants.RobotConstants.length)
                .setColorScheme(new ColorSchemeBlueLight())
                .setStartPose(fcs.specimenSideStartPose)
                .build();

        DriveShim drive = myBot.getDrive();


        // fcs.setRed();

        firstSpecimenDeliverMove = drive.actionBuilder(fcs.specimenSideStartPose)
                .strafeToLinearHeading(fcs.specimenDeliverPose1.position, fcs.specimenDropAngle)
                .build();

        firstSampleMoveToObservationZone = drive.actionBuilder(fcs.specimenDeliverPose1)
                .strafeToLinearHeading(fcs.firstStagePushInnerPose.position, Math.toRadians(180))
                .strafeToLinearHeading(fcs.secondStagePushInnerVector, Math.toRadians(180))
                .strafeToLinearHeading(fcs.thirdStagePushInnerVector, Math.toRadians(180))
                .strafeToLinearHeading(fcs.sample1ObservationZoneDropPose.position, Math.toRadians(180)).build();

        secondSampleMoveToObservationZone = drive.actionBuilder(fcs.sample1ObservationZoneDropPose)
                .strafeToLinearHeading(fcs.secondStagePushMidVector, fcs.specimenPickupAngle)
                .strafeToLinearHeading(fcs.thirdStagePushMidVector, fcs.specimenPickupAngle)
                .strafeToLinearHeading(fcs.sample2ObservationZonePickupPose.position, fcs.specimenPickupAngle).build();


        secondSpecimenDeliverMove = drive.actionBuilder(fcs.sample2ObservationZonePickupPose)
                .splineToLinearHeading(fcs.specimenDeliverPose2, fcs.specimenPickupAngle).build();


        thirdSpecimenPickupMove = drive.actionBuilder(fcs.specimenDeliverPose2)
                .splineToLinearHeading(fcs.specimenPickupPose, fcs.specimenDropAngle).build();


        thirdSpecimenDeliverMove = drive.actionBuilder(fcs.specimenPickupPose)
                .splineToLinearHeading(fcs.specimenDeliverPose3, fcs.specimenPickupAngle).build();


        fourthSpecimenPickupMove = drive.actionBuilder(fcs.specimenDeliverPose3)
                .splineToLinearHeading(fcs.specimenPickupPose, fcs.specimenDropAngle).build();

        fourthSpecimenDeliverMove = drive.actionBuilder(fcs.specimenPickupPose)
                .splineToLinearHeading(fcs.specimenDeliverPose4, fcs.specimenPickupAngle).build();

        park = drive.actionBuilder(fcs.specimenDeliverPose4)
                .strafeToLinearHeading(fcs.specimenParkPose.position, fcs.specimenDropAngle).build();


        sequenceOne = new SequentialAction(

                new ParallelAction(
                        firstSpecimenDeliverMove, new SleepAction(1)),
                //   elevator.elevatorToAboveUpperSubmersible()),
                // ears.deliverSpecimenToUpperSubmersible(),
                new SleepAction(2),
                new ParallelAction(
                        firstSampleMoveToObservationZone, new SleepAction(1)),
                // elevator.elevatorToHome()),
                secondSampleMoveToObservationZone,
                new SleepAction(1),
                // elevator.grabSpecimenAndClearWall(),
                new ParallelAction(
                        secondSpecimenDeliverMove, new SleepAction(1)),
                // elevator.elevatorToAboveUpperSubmersible()),
                new SleepAction(1),
                //ears.deliverSpecimenToUpperSubmersible(),
                new ParallelAction(
                        thirdSpecimenPickupMove,
                        new SleepAction(1)),
                new SleepAction(1),
                // elevator.grabSpecimenAndClearWall(),
                new ParallelAction(
                        thirdSpecimenDeliverMove, new SleepAction(1)),
                //   elevator.elevatorToAboveUpperSubmersible()),
                new SleepAction(1),
                // ears.deliverSpecimenToUpperSubmersible(),
                new ParallelAction(
                        fourthSpecimenPickupMove, new SleepAction(1)),
                // elevator.elevatorToHome()),
                new SleepAction(1),
                //  elevator.grabSpecimenAndClearWall(),
                new ParallelAction(
                        fourthSpecimenDeliverMove, new SleepAction(1)),
                //  elevator.elevatorToAboveUpperSubmersible()),
                new SleepAction(1),
                //  ears.deliverSpecimenToUpperSubmersible(),
                park);


        myBot.runAction(sequenceOne);
        myBot.setPose(fcs.specimenSideStartPose);
        ShowField.showIt(meepMeep, myBot);
        meepMeep.start();
    }


}


