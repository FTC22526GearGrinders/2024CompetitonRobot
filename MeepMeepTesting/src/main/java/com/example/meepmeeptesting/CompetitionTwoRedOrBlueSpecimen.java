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

public class CompetitionTwoRedOrBlueSpecimen {


    public static void main(String[] args) {

        Action elevatorMove = new SleepAction(2);

        Action sequenceOne;


        FieldConstantsSelect fcs;
        Action firstSpecimenPreDeliverMove;
        Action firstSpecimenDeliverMove;
        Action firstSampleMoveToObservationZone;

        Action firstSampleMoveToObservationZonePickup;

        Action secondSampleMoveToObservationZoneDrop;
        Action secondSampleMoveToObservationZonePickup;


        Action secondSpecimenPrePickupMove;
        Action secondSpecimenPickupMove;


        Action secondSpecimenPreDeliverMove;
        Action secondSpecimenDeliverMove;

        Action thirdSpecimenPrePickupMove;
        Action thirdSpecimenPickupMove;

        Action thirdSpecimenPreDeliverMove;
        Action thirdSpecimenDeliverMove;

        Action fourthSpecimenPrePickupMove;
        Action fourthSpecimenPickupMove;

        Action fourthSpecimenPreDeliverMove;
        Action fourthSpecimenDeliverMove;


        Action park;

        TranslationalVelConstraint finalVel;
        ProfileAccelConstraint finalAccel;
        MeepMeep meepMeep = new MeepMeep(800);
        fcs = new FieldConstantsSelect();

        finalVel = new TranslationalVelConstraint(10.0);
        finalAccel = new ProfileAccelConstraint(-20.0, 20.0);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                .setDriveTrainType((DriveTrainType.MECANUM))
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(80, 20, Math.toRadians(180), Math.toRadians(180), 15)
                .setDimensions(Constants.RobotConstants.width, Constants.RobotConstants.length)
                .setColorScheme(new ColorSchemeBlueLight())
                .setStartPose(fcs.specimenSideStartPose)
                .build();

        DriveShim drive = myBot.getDrive();


        //  fcs.setRed();


        firstSpecimenPreDeliverMove = drive.actionBuilder(fcs.specimenSideStartPose)
                .strafeToLinearHeading(fcs.specimenDeliverApproachPose1.position, fcs.specimenDropAngle)
                .build();

        firstSpecimenDeliverMove = drive.actionBuilder(fcs.specimenDeliverApproachPose1)
                .strafeToLinearHeading(fcs.specimenDeliverPose1.position, fcs.specimenDropAngle,
                        finalVel, finalAccel).build();


        secondSpecimenPrePickupMove = drive.actionBuilder(fcs.specimenDeliverPose1)
                .splineToLinearHeading(fcs.specimenPickupApproachPose, fcs.specimenDropAngle).build();

        secondSpecimenPickupMove = drive.actionBuilder(fcs.specimenPickupApproachPose)
                .strafeToLinearHeading(fcs.specimenPickupPose.position, fcs.specimenPickupAngle,
                        finalVel, finalAccel).build();


        secondSpecimenPreDeliverMove = drive.actionBuilder(fcs.specimenPickupPose)
                .strafeToLinearHeading(fcs.specimenDeliverApproachPose2.position, fcs.specimenDropApproachAngle).build();

        secondSpecimenDeliverMove = drive.actionBuilder(fcs.specimenDeliverApproachPose2)
                .strafeToLinearHeading(fcs.specimenDeliverPose2.position, fcs.specimenDropAngle,
                        finalVel, finalAccel).build();

        park = drive.actionBuilder(fcs.specimenDeliverPose2)
                .strafeToLinearHeading(fcs.specimenParkPose.position, fcs.specimenDropAngle).build();


        sequenceOne = new SequentialAction(

                new ParallelAction(
                        firstSpecimenPreDeliverMove, new SleepAction(1)),
                // elevator.elevatorToAboveUpperSubmersible()),

                firstSpecimenDeliverMove,

                // elevator.deliverSpecimenToNearestChamber(),
                new SleepAction(1),

                new ParallelAction(new SleepAction(1),
                        secondSpecimenPrePickupMove),
                // elevator.elevatorToHome()),

                secondSpecimenPickupMove,

                // elevator.grabSpecimenAndClearWall(),

                new SleepAction(1),
                new ParallelAction(
                        secondSpecimenPreDeliverMove, new SleepAction(1)),
                // elevator.elevatorToAboveUpperSubmersible()),

                secondSpecimenDeliverMove,
                new SleepAction(1),
                // elevator.deliverSpecimenToNearestChamber(),

                new ParallelAction(new SleepAction(1)),
                //  elevator.elevatorToHome(),
                park);


        myBot.runAction(sequenceOne);
        myBot.setPose(fcs.specimenSideStartPose);
        ShowField.showIt(meepMeep, myBot);
        meepMeep.start();
    }


}


