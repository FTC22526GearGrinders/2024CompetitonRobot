package com.example.meepmeeptesting;


import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {


    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);
        boolean ix = false;
        boolean iy = false;
        boolean ih = true;

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
                                        .lineToSplineHeading(FieldConstantsBlue.outerYellowPickupPose)

                                    .waitSeconds((2))//basket drop
                                        .build()
                );

        ShowField.showIt(meepMeep, myBot);
        myBot.getDrive().setPoseEstimate(FieldConstantsBlue.basketSideStartPose);

    }
}
