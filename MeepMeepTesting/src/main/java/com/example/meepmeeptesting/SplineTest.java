package com.example.meepmeeptesting;


import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueLight;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.DriveShim;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class SplineTest {


    public static void main(String[] args) {

        FieldConstantsSelect fcs = new FieldConstantsSelect();

        MeepMeep meepMeep = new MeepMeep(800);
        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(45, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .setDimensions(Constants.RobotConstants.width, Constants.RobotConstants.length)
                .setColorScheme(new ColorSchemeBlueLight())
                .setStartPose(new Pose2d(0, 0, 0))
                .build();

        DriveShim drive = myBot.getDrive();

//Action test = drive.actionBuilder(new Pose2d(0,0, -Math.PI ))
//        .setTangent(0)
//        .splineToLinearHeading(new Pose2d(48, 0,Math.PI/2), 0)
//        .build();

        Action firstSampleMoveToObservationZoneAction = drive.actionBuilder(fcs.specimenDeliverPose1)
                .splineToSplineHeading(new Pose2d(-36, 42, Math.toRadians(180)), Math.toRadians(180))
                .strafeTo(new Vector2d(-36, 10))
                .strafeTo(new Vector2d(-48, 10))
                .strafeTo(fcs.sample1ObservationZoneDropPose.position)
                .build();

        Action secondSpecimenPickupMoveAction = drive.actionBuilder(fcs.sample1ObservationZoneDropPose)
                .lineToXSplineHeading(fcs.specimenPickupApproachPose.position.x, Math.toRadians(90))
                .strafeTo(fcs.specimenPickupPose.position)
                .waitSeconds(1)
                .build();
        Action test = secondSpecimenPickupMoveAction;

        myBot.runAction(test);
        ShowField.showIt(meepMeep, myBot);
        meepMeep.start();
    }

}


