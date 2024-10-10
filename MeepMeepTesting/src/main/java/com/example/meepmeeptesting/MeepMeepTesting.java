package com.example.meepmeeptesting;


import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {


    public static void main(String[] args) {


        boolean blue = false;
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
                                    .waitSeconds((2))//basket drop
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
                                                    FieldConstantsRed.sampleSideStartPose)
                                            .waitSeconds((1))//sample
                                            .lineToSplineHeading(FieldConstantsRed.specimenDeliverPose1)
                                            .waitSeconds((1))//sample
                                            //  .lineToSplineHeading(FieldConstantsRed.sampleSideStartPose)

//                                    .turn(Math.toRadians(-90))
                                            .setTangent(0)
                                            .lineToConstantHeading(FieldConstantsRed.innerRedPrePose)
                                            .forward(6)


                                            .lineToLinearHeading(FieldConstantsRed.samplePickupPose)
//
                                            .lineToSplineHeading(FieldConstantsRed.specimenDeliverPose2)
                                            .waitSeconds((1))//sample
                                            .lineToConstantHeading(FieldConstantsRed.midRedPrePose)
                                            .forward(6)


                                            .lineToLinearHeading(FieldConstantsRed.samplePickupPose)
                                            .lineToSplineHeading(FieldConstantsRed.specimenDeliverPose3)
                                            .waitSeconds((1))//sample

                                            .back(15)
//                                    .lineToSplineHeading(FieldConstantsRed.samplePickupApproachPose)
                                            .lineToSplineHeading(FieldConstantsRed.outerRedApproachPose)
//                                    //.waitSeconds((.2))//s
                                            .lineToSplineHeading(FieldConstantsRed.outerRedPickupPose)
                                            .waitSeconds((1))//sample
                                            .lineToSplineHeading(FieldConstantsRed.samplePickupPose)
                                            .waitSeconds(1)
//                                   // .lineToSplineHeading(FieldConstantsRed.sampleDeliverPose)
//                                  //  .waitSeconds(.2)
                                            .lineToSplineHeading(FieldConstantsRed.specimenDeliverPose4)
                                            .waitSeconds(5)

                                            .build()
                    );

            ShowField.showIt(meepMeep, myBot);
            myBot.getDrive().setPoseEstimate(FieldConstantsRed.sampleSideStartPose);
        }
    }

}
