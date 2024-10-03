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
                        drive.trajectorySequenceBuilder(FieldConstantsRed.adjustPose(FieldConstantsRed.specimenStartPose, ix, iy, ih))
                                .lineToSplineHeading(FieldConstantsRed.adjustPose(FieldConstantsRed.specimenDropPose, ix, iy, ih))
                                .waitSeconds((2))//sample drop
                                .lineToSplineHeading(FieldConstantsRed.adjustPose(FieldConstantsRed.specimenDropBackupPose, ix, iy, ih))
                                .lineToSplineHeading(FieldConstantsRed.adjustPose(FieldConstantsRed.nearInnerSpecimenPose, ix, iy, ih))//near inner specimen pose
                                .lineToSplineHeading(FieldConstantsRed.adjustPose(FieldConstantsRed.pickupInnerSpecimenPose, ix, iy, ih))//pickup inner specimer
                                .lineToSplineHeading(FieldConstantsRed.adjustPose(FieldConstantsRed.basketPose, ix, iy, ih))
                                .waitSeconds((2))//basket drop
                                .lineToSplineHeading(FieldConstantsRed.adjustPose(FieldConstantsRed.nearMiddleSpecimenPose, ix, iy, ih))//near mid specimen
                                .lineToSplineHeading(FieldConstantsRed.adjustPose(FieldConstantsRed.pickupMiddleSpecimenPose, ix, iy, ih))//pickup mid specimen
                                .lineToSplineHeading(FieldConstantsRed.adjustPose(FieldConstantsRed.basketPose, ix, iy, ih))
                                .waitSeconds((2))//basket drop
                                .lineToSplineHeading(FieldConstantsRed.adjustPose(FieldConstantsRed.nearOuterSpecimenPose, ix, iy, ih))//near outer specimen
                                .lineToSplineHeading(FieldConstantsRed.adjustPose(FieldConstantsRed.pickupOuterSpecimenPose, ix, iy, ih))//pickup outer specimen
                                .lineToSplineHeading(FieldConstantsRed.adjustPose(FieldConstantsRed.basketPose, ix, iy, ih))
                                .waitSeconds((2))//basket drop
                                .build()
                );

        ShowField.showIt(meepMeep, myBot);
        myBot.getDrive().setPoseEstimate(FieldConstantsRed.specimenStartPose);

    }
}
