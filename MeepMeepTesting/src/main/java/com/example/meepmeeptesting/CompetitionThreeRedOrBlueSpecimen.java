package com.example.meepmeeptesting;


import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueLight;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.DriveShim;
import com.noahbres.meepmeep.roadrunner.DriveTrainType;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class CompetitionThreeRedOrBlueSpecimen {


    public static void main(String[] args) {

        Action elevatorMove = new SleepAction(2);

        FieldConstantsSelect fcs;
        Action firstSpecimenPreDeliverMove;
        Action firstSpecimenDeliverMove;
        Action firstSampleMoveToObservationZone;
        Action firstSampleMoveToObservationZonePickup;

        Action secondSpecimenPreDeliverMove;
        Action secondSpecimenDeliverMove;

        Action thirdSpecimenPrePickupMove;
        Action thirdSpecimenPickupMove;

        Action thirdSpecimenPreDeliverMove;
        Action thirdSpecimenDeliverMove;

        Action park;

        Action sequenceOne;

        TranslationalVelConstraint finalVel;
        ProfileAccelConstraint finalAccel;
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


        fcs.setRed();


        firstSpecimenPreDeliverMove = drive.actionBuilder(fcs.specimenSideStartPose)
                .strafeToLinearHeading(fcs.specimenDeliverApproachPose1.position, fcs.specimenDropAngle)
                .build();

        firstSpecimenDeliverMove = drive.actionBuilder(fcs.specimenDeliverApproachPose1)
                .strafeToLinearHeading(fcs.specimenDeliverPose1.position, fcs.specimenDropAngle)
                .build();

        firstSampleMoveToObservationZone = drive.actionBuilder(fcs.specimenDeliverPose1)
                .strafeToLinearHeading(fcs.firstStagePushInnerPose.position, Math.toRadians(180))
                .strafeToLinearHeading(fcs.secondStagePushInnerVector, Math.toRadians(180))
                .strafeToLinearHeading(fcs.thirdStagePushInnerVector, fcs.specimenPickupAngle)
                .strafeToLinearHeading(fcs.sample3ObservationZoneDropPose.position, fcs.specimenPickupAngle).build();

        firstSampleMoveToObservationZonePickup = drive.actionBuilder(fcs.sample3ObservationZoneDropPose)
                .strafeToLinearHeading(fcs.sample3ObservationZonePickupPose.position, fcs.specimenPickupAngle,
                        finalVel, finalAccel).build();


        secondSpecimenPreDeliverMove = drive.actionBuilder(fcs.sample3ObservationZonePickupPose)
                .splineToLinearHeading(fcs.specimenDeliverApproachPose2, fcs.specimenPickupAngle).build();

        secondSpecimenDeliverMove = drive.actionBuilder(fcs.specimenDeliverApproachPose2)
                .strafeToLinearHeading(fcs.specimenDeliverPose2.position, fcs.specimenDropAngle,
                        finalVel, finalAccel).build();


        thirdSpecimenPrePickupMove = drive.actionBuilder(fcs.specimenDeliverPose2)
                .splineToLinearHeading(fcs.specimenPickupApproachPose, fcs.specimenDropAngle).build();

        thirdSpecimenPickupMove = drive.actionBuilder(fcs.specimenPickupApproachPose)
                .strafeToLinearHeading(fcs.specimenPickupPose.position, fcs.specimenPickupAngle,
                        finalVel, finalAccel).build();


        thirdSpecimenPreDeliverMove = drive.actionBuilder(fcs.specimenPickupPose)
                .splineToLinearHeading(fcs.specimenDeliverApproachPose3, fcs.specimenPickupAngle).build();

        thirdSpecimenDeliverMove = drive.actionBuilder(fcs.specimenDeliverApproachPose3)
                .strafeToLinearHeading(fcs.specimenDeliverPose3.position, fcs.specimenDropAngle,
                        finalVel, finalAccel).build();


        park = drive.actionBuilder(fcs.specimenDeliverPose3)
                .strafeToLinearHeading(fcs.specimenParkPose.position, fcs.specimenDropAngle).build();

        sequenceOne = new SequentialAction(

                new ParallelAction(
                        firstSpecimenPreDeliverMove, new SleepAction(1)),
                // elevator.elevatorToAboveUpperSubmersible()),

                firstSpecimenDeliverMove,

                new SleepAction(1),

                //    elevator.deliverSpecimenToNearestChamber(),


                new ParallelAction(
                        firstSampleMoveToObservationZone, new SleepAction(1)),
                //   elevator.elevatorToHome()),

                firstSampleMoveToObservationZonePickup,

                new SleepAction(1),
                //   elevator.grabSpecimenAndClearWall(),


                new ParallelAction(
                        secondSpecimenPreDeliverMove, new SleepAction(1)),
                //  elevator.elevatorToAboveUpperSubmersible()),

                secondSpecimenDeliverMove,

                new SleepAction(1),

                //  elevator.deliverSpecimenToNearestChamber(),

                new ParallelAction(
                        thirdSpecimenPrePickupMove, new SleepAction(1)),
                //   elevator.elevatorToHome()),

                thirdSpecimenPickupMove,

                new SleepAction(1),

                //  elevator.grabSpecimenAndClearWall(),

                new ParallelAction(
                        thirdSpecimenPreDeliverMove, new SleepAction(1)),
                // elevator.elevatorToAboveUpperSubmersible()),

                thirdSpecimenDeliverMove,

                new SleepAction(1),

                //   elevator.deliverSpecimenToNearestChamber(),


                new ParallelAction(new SleepAction(1)),
                // elevator.elevatorToHome(),
                park);


        myBot.runAction(sequenceOne);
        myBot.setPose(fcs.specimenSideStartPose);
        ShowField.showIt(meepMeep, myBot);
        meepMeep.start();
    }


}


