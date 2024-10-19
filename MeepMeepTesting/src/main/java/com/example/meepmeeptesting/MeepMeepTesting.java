package com.example.meepmeeptesting;


import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {


    public static void main(String[] args) {


        boolean blue = true;
        boolean basket = true;


        if (blue && basket) {
            MeepMeep meepMeep = new MeepMeep(800);
            RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                    // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                    .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                    .setDimensions(Constants.RobotConstants.width, Constants.RobotConstants.length)

                    .followTrajectorySequence(drive ->
                            drive.trajectorySequenceBuilder(
                                            FieldConstantsBlue.basketSideStartPose)
                                    .waitSeconds((1))//sample
                                    .lineToSplineHeading(FieldConstantsBlue.basketDeliverPose)
                                    .waitSeconds((1))//sample
                                    .lineToSplineHeading(FieldConstantsBlue.innerYellowPickupPose)
                                    .waitSeconds(1)
                                    .lineToSplineHeading(FieldConstantsBlue.basketDeliverPose)
                                    .waitSeconds(1)
                                    .lineToSplineHeading(FieldConstantsBlue.midYellowPickupPose)
                                    .waitSeconds(1)
                                    .lineToSplineHeading(FieldConstantsBlue.basketDeliverPose)
                                    .waitSeconds(1)
                                    .lineToSplineHeading(FieldConstantsBlue.outerYellowApproachPose)
                                    .waitSeconds(.2)
                                    .lineToSplineHeading(FieldConstantsBlue.outerYellowPickupPose)
                                    .lineToSplineHeading(FieldConstantsBlue.outerYellowApproachPose)
                                    .lineToSplineHeading(FieldConstantsBlue.basketDeliverPose)
                                    .waitSeconds((2))//basket drop
                                    .build()
                    );

            ShowField.showIt(meepMeep, myBot);
            myBot.getDrive().setPoseEstimate(FieldConstantsBlue.basketSideStartPose);
        }
        if (!blue && basket) {
            MeepMeep meepMeep = new MeepMeep(800);
            RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                    // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                    .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                    .setDimensions(Constants.RobotConstants.width, Constants.RobotConstants.length)

                    .followTrajectorySequence(drive ->
                            drive.trajectorySequenceBuilder(
                                            FieldConstantsRed.basketSideStartPose)
                                    .waitSeconds((1))//sample

                                    .lineToSplineHeading(FieldConstantsRed.basketDeliverPose)
                                    .waitSeconds((1))//sample
                                    .lineToSplineHeading(FieldConstantsRed.innerYellowPickupPose)
                                    .waitSeconds(1)
                                    .lineToSplineHeading(FieldConstantsRed.basketDeliverPose)
                                    .waitSeconds(1)
                                    .lineToSplineHeading(FieldConstantsRed.midYellowPickupPose)
                                    .waitSeconds(1)
                                    .lineToSplineHeading(FieldConstantsRed.basketDeliverPose)
                                    .waitSeconds(1)
                                    .lineToSplineHeading(FieldConstantsRed.outerYellowApproachPose)
                                    .waitSeconds(.2)
                                    .lineToSplineHeading(FieldConstantsRed.outerYellowPickupPose)
                                    .waitSeconds((2))//basket drop
                                    .lineToSplineHeading(FieldConstantsRed.outerYellowApproachPose)
                                    .lineToSplineHeading(FieldConstantsRed.basketDeliverPose)
                                    .build()
                    );

            ShowField.showIt(meepMeep, myBot);
            myBot.getDrive().setPoseEstimate(FieldConstantsRed.basketSideStartPose);
        }


        if (blue && !basket) {
            MeepMeep meepMeep = new MeepMeep(800);
            RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                    // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                    .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                    .setDimensions(Constants.RobotConstants.width, Constants.RobotConstants.length)

                    .followTrajectorySequence(drive ->
                                    drive.trajectorySequenceBuilder(
                                                    FieldConstantsBlue.specimenSideStartPose)
                                            .waitSeconds((1))//sample
                                            .lineToSplineHeading(FieldConstantsBlue.specimenDeliverPose1)
                                            .waitSeconds((1))//sample

                                            .splineToLinearHeading(FieldConstantsBlue.innerBluePickupPose, 180)

                                            .waitSeconds(1)

                                            .lineToLinearHeading(FieldConstantsBlue.samplePickupPose)
//
                                            .lineToSplineHeading(FieldConstantsBlue.specimenDeliverPose2)
                                            .waitSeconds((1))//sample

                                            .splineToLinearHeading(FieldConstantsBlue.midBluePickupPose, 180)

                                            .lineToLinearHeading(FieldConstantsBlue.samplePickupPose)

                                            .lineToSplineHeading(FieldConstantsBlue.specimenDeliverPose3)
                                            .waitSeconds((1))//sample
                                            .splineToLinearHeading(FieldConstantsBlue.outerBluePickupPose, 180)
                                            .waitSeconds((1))//sample
                                            .lineToSplineHeading(FieldConstantsBlue.samplePickupPose)
                                            .waitSeconds(1)
//
                                            .lineToSplineHeading(FieldConstantsBlue.specimenDeliverPose4)
                                            .waitSeconds(5)

                                            .build()
                    );

            ShowField.showIt(meepMeep, myBot);
            myBot.getDrive().setPoseEstimate(FieldConstantsBlue.basketSideStartPose);
        }
        if (!blue && !basket) {
            MeepMeep meepMeep = new MeepMeep(800);
            RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                    // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                    .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                    .setDimensions(Constants.RobotConstants.width, Constants.RobotConstants.length)

                    .followTrajectorySequence(drive ->
                            drive.trajectorySequenceBuilder(
                                            FieldConstantsRed.specimenSideStartPose)
                                    .waitSeconds((1))//sample
                                    .lineToSplineHeading(FieldConstantsRed.specimenDeliverPose1)
                                    .waitSeconds((1))//sample

                                    .splineToLinearHeading(FieldConstantsRed.innerRedPickupPose, 0)

                                    .waitSeconds(1)

                                    .lineToLinearHeading(FieldConstantsRed.specimenPickupPose)
                                    .waitSeconds((1))//sample
                                    .lineToSplineHeading(FieldConstantsRed.sampleDropPose)
                                    .lineToSplineHeading(FieldConstantsRed.specimenDeliverApproachPose2)
                                    .lineToSplineHeading(FieldConstantsRed.specimenDeliverPose2)
                                    .waitSeconds((1))//sample

                                    .splineToLinearHeading(FieldConstantsRed.midRedPickupPose, 0)

                                    .lineToLinearHeading(FieldConstantsRed.specimenPickupPose)
                                    .waitSeconds((1))//sample
                                    .lineToSplineHeading(FieldConstantsRed.sampleDropPose)
                                    .lineToSplineHeading(FieldConstantsRed.specimenDeliverApproachPose3)
                                    .lineToSplineHeading(FieldConstantsRed.specimenDeliverPose3)
                                    .waitSeconds((1))//sample

                                    .splineToLinearHeading(FieldConstantsRed.outerRedPickupPose, 0)
                                    .waitSeconds((1))//sample
                                    .back(6)
                                    .lineToSplineHeading(FieldConstantsRed.specimenPickupPose)
                                    .waitSeconds(1)
                                    .lineToSplineHeading(FieldConstantsRed.sampleDropPose)
                                    .lineToSplineHeading(FieldConstantsRed.specimenDeliverApproachPose4)
                                    .lineToSplineHeading(FieldConstantsRed.specimenDeliverPose4)
                                    .waitSeconds(5)

                                    .build()
                    );

            ShowField.showIt(meepMeep, myBot);
            myBot.getDrive().setPoseEstimate(FieldConstantsRed.specimenSideStartPose);
        }
    }

}
