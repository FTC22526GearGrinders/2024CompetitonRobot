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
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;


public class RedOrBlueSamples {


    public static void main(String[] args) {
        Action firstSampleDeliverMove;

        Action secondSampleDeliverMove;
        Action thirdSampleDeliverMove;
        Action fourthSampleDeliverMove;
        Action secondSamplePrePickupMove;
        Action secondSamplePickupMove;
        Action thirdSamplePrePickupMove;
        Action thirdSamplePickupMove;
        Action fourthSamplePrePickupMove;
        Action fourthSamplePickupMove;
        Action parkAction;

        Action placeSpecimenAction = new SleepAction(2);
        Action pickupSampleAction = new SleepAction(2);
        Action transferSampleToBucketAction = new SleepAction(2);
        Action dropSampleAction = new SleepAction(2);
        TranslationalVelConstraint finalVel;
        ProfileAccelConstraint finalAccel;

        FieldConstantsSelect fcs;
        SequentialAction deliverFourSamples;

        MeepMeep meepMeep = new MeepMeep(800);
        fcs = new FieldConstantsSelect();

        finalAccel = new ProfileAccelConstraint(-10, 10);
        finalVel = new TranslationalVelConstraint(10);
        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .setDimensions(Constants.RobotConstants.width, Constants.RobotConstants.length)
                .setColorScheme(new ColorSchemeBlueLight())
                .setStartPose(fcs.basketSideStartPose)
                .build();

        DriveShim drive = myBot.getDrive();

        firstSampleDeliverMove = drive.actionBuilder(fcs.basketSideStartPose)
                .strafeToLinearHeading(fcs.basketDeliverPose.position, fcs.basketDeliverPose.heading).build();

        secondSamplePrePickupMove = drive.actionBuilder(fcs.basketDeliverPose)
                .strafeToLinearHeading(fcs.innerYellowPrePickupPose.position, fcs.innerYellowPickupPose.heading).build();

        secondSamplePickupMove = drive.actionBuilder(fcs.innerYellowPrePickupPose)
                .strafeToLinearHeading(fcs.innerYellowPickupPose.position, fcs.innerYellowPickupPose.heading,
                        finalVel, finalAccel).build();

        secondSampleDeliverMove = drive.actionBuilder(fcs.innerYellowPickupPose)
                .strafeToLinearHeading(fcs.basketDeliverPose.position, fcs.basketDeliverPose.heading).build();

        thirdSamplePrePickupMove = drive.actionBuilder(fcs.basketDeliverPose)
                .strafeToLinearHeading(fcs.midYellowPrePickupPose.position, fcs.midYellowPrePickupPose.heading).build();

        thirdSamplePickupMove = drive.actionBuilder(fcs.midYellowPrePickupPose)
                .strafeToLinearHeading(fcs.midYellowPickupPose.position, fcs.midYellowPrePickupPose.heading,
                        finalVel, finalAccel).build();

        thirdSampleDeliverMove = drive.actionBuilder(fcs.midYellowPickupPose)
                .strafeToLinearHeading(fcs.basketDeliverPose.position, fcs.basketDeliverPose.heading).build();

        fourthSamplePrePickupMove = drive.actionBuilder(fcs.basketDeliverPose)
                .strafeToLinearHeading(fcs.outerYellowPrePose.position, fcs.outerYellowPrePose.heading).build();

        fourthSamplePickupMove = drive.actionBuilder(fcs.outerYellowPrePose)
                .strafeToLinearHeading(fcs.outerYellowPickupPose.position, fcs.outerYellowPickupPose.heading,
                        finalVel, finalAccel).build();

        fourthSampleDeliverMove = drive.actionBuilder(fcs.outerYellowPickupPose)
                .strafeToLinearHeading(fcs.basketDeliverPose.position, fcs.basketDeliverPose.heading).build();

        parkAction = drive.actionBuilder(fcs.basketDeliverPose)
                .strafeToLinearHeading(fcs.ascentZoneParkPose.position, fcs.ascentZoneParkPose.heading).build();


        deliverFourSamples = new SequentialAction(


                new ParallelAction(
                        firstSampleDeliverMove,
                        new SleepAction(1)),
                // elevator.elevatorToUpperBasket()),

                //  elevator.cycleBucket(),
                new SleepAction(1),

                new ParallelAction(
                        new SleepAction(1),
                        // elevator.elevatorToHome(),
                        secondSamplePrePickupMove),

                new ParallelAction(
                        new SleepAction(1),
                        secondSamplePickupMove),
                //   ears.armOutTiltAboveSamplesOpenClaw()),
//
                new SleepAction(1),
//                //    ears.pickupSampleDeliverToBucket(),
//
                new ParallelAction(
                        new SleepAction(1),
                        secondSampleDeliverMove),
//                //  elevator.elevatorToUpperBasket()),
//
                new SleepAction(1),
//                // elevator.cycleBucket(),
//
                new ParallelAction(
                        new SleepAction(1),
//                        //  elevator.elevatorToHome(),
                        thirdSamplePrePickupMove),
//
                new ParallelAction(
                        thirdSamplePickupMove,
                        new SleepAction(1)),
//                // ears.armOutTiltAboveSamplesOpenClaw()),
//
//
//                // ears.pickupSampleDeliverToBucket(),
                new SleepAction(1),
                new ParallelAction(
                        thirdSampleDeliverMove,
                        new SleepAction(1)),
//                // elevator.elevatorToUpperBasket()),
//
//
//                // elevator.cycleBucket(),
                new SleepAction(1),
//
                new ParallelAction(
                        new SleepAction(1),
//                        //  elevator.elevatorToHome(),
                        fourthSamplePrePickupMove),
//
//
                new ParallelAction(
                        fourthSamplePickupMove,
                        new SleepAction(1)),
//                // ears.armOutTiltAboveSamplesOpenClaw()),
//
                new SleepAction(1),
//                //  ears.pickupSampleDeliverToBucket(),
//
                new ParallelAction(
                        fourthSampleDeliverMove,
                        new SleepAction(1)),
//                        //   elevator.elevatorToUpperBasket()),
//
//                        // elevator.cycleBucket(),
                new SleepAction(1),
                new ParallelAction(
                        new SleepAction(1),
//                                // elevator.elevatorToHome(),
                        parkAction));
//

        myBot.runAction(deliverFourSamples);
        ShowField.showIt(meepMeep, myBot);
        meepMeep.start();
    }

}



