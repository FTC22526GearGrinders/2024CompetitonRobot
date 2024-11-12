package org.firstinspires.ftc.teamcode.opmodes_auto;


/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */


import static com.qualcomm.robotcore.util.ElapsedTime.Resolution.SECONDS;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.FieldConstantsBlue;
import org.firstinspires.ftc.teamcode.FieldConstantsRed;
import org.firstinspires.ftc.teamcode.subsystems.LimelightSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.utils.PoseStorage;


@Autonomous(name = "Specimen", group = "Auto")
//@Disabled
public class SpecimenSideAutoOpmode extends CommandOpMode {

    public static String TEAM_NAME = "Gear Grinders"; // Enter team Name
    public static int TEAM_NUMBER = 22526; //Enter team Number
    public static Pose2d startPosition;

    Action firstSpecimenDeliverMoveAction;
    Action secondSpecimenDeliverMoveAction;
    Action thirdSpecimenDeliverMoveAction;
    Action fourthSpecimenDeliverMoveAction;

    Action secondSpecimenPickupMoveAction;
    Action thirdSpecimenPickupMoveAction;
    Action fourthSpecimenPickupMoveAction;

    Action firstSampleMoveToObservationZoneAction;
    Action secondSampleMoveToObservationZoneAction;

    Action deliverThreeSpecimens;

    Action placeSpecimenAction = new SleepAction(2);
    Action pickupSampleAction;
    Action raiseArmIntakeAction;

    Action transferSampleToBucketAction = new SleepAction(2);
    Action dropSampleAction = new SleepAction(2);
    Action collectSpecimenAction = new SleepAction(2);

    Action samplePickupAction;

    private MecanumDriveSubsystem drive;


    private LimelightSubsystem limelight;

    private TelemetryPacket packet;

    @Override
    public void initialize() {
        drive = new MecanumDriveSubsystem(this, FieldConstantsBlue.specimenSideStartPose);

        limelight = new LimelightSubsystem(this);
        packet = new TelemetryPacket();


    }

    @Override
    public void runOpMode() throws InterruptedException {

        initialize();

        selectStartingPosition();

        waitForStart();

        while (!isStopRequested() && opModeIsActive()) {

            run();

            telemetry.update();
            drive = new MecanumDriveSubsystem(this, startPosition);

            Actions.runBlocking(deliverThreeSpecimens);


        }
        PoseStorage.currentPose = drive.pose;
        PoseStorage.poseUpdatedTime = System.currentTimeMillis();

        PoseStorage.currentTeam = drive.currentteam;

        reset();
    }

    //Method to select starting position using X, Y, A, B buttons on gamepad
    public void selectStartingPosition() {
        telemetry.setAutoClear(true);
        telemetry.clearAll();
        //******select start pose*****
        while (!isStopRequested()) {
            telemetry.addData("Initializing Autonomous for Team:",
                    TEAM_NAME, " ", TEAM_NUMBER);
            telemetry.addData("---------------------------------------", "");
            telemetry.addData("Select Alliance using XA on Logitech (or ▢ΔOX on Playstayion) on gamepad 1:", "");
            telemetry.addData("    Red All Specimen   ", "(A / O)");

            telemetry.addData("    Blue All Specimen    ", "(X / ▢)");

            if (gamepad1.a) {

                telemetry.clearAll();
                telemetry.addData("RED ", "Chosen");
                telemetry.addData("Restart OpMode ", "to Change");


                drive.currentteam = PoseStorage.Team.RED;

                drive.pose = FieldConstantsRed.basketSideStartPose;


//                firstSpecimenDeliverMoveAction = drive.actionBuilder(FieldConstantsRed.specimenSideStartPose)
//                        .lineToY(FieldConstantsRed.specimenDeliverPose1.position.y)
//                        .build();//move to place first specimen
//
//                firstSpecimenDeliverBackupAction = drive.actionBuilder(FieldConstantsRed.specimenDeliverPose1)
//                        .lineToY(FieldConstantsRed.specimenDeliverApproachPose1.position.y)
//                        .build();
//
//                firstSamplePickupMoveAction = drive.actionBuilder(FieldConstantsRed.specimenDeliverApproachPose1)
//                        .strafeToLinearHeading(FieldConstantsRed.innerRedPickupPose.position, FieldConstantsRed.innerRedPickupPose.heading)
//                        .build();//move to pickup inner sample
//
//                secondSpecimenPickupMoveAction = drive.actionBuilder(FieldConstantsRed.innerRedPickupPose)
//                        .strafeToLinearHeading(FieldConstantsRed.specimenPickupPose.position, FieldConstantsRed.specimenPickupPose.heading)
//                        .build();//move to drop first sample and pick up second specimen
//
//                secondSpecimenDeliverMoveAction = drive.actionBuilder(FieldConstantsRed.specimenPickupPose)
//                        .splineToLinearHeading(FieldConstantsRed.specimenDeliverApproachPose2, FieldConstantsRed.specimenDeliverApproachPose2.heading)
//                        .lineToY(FieldConstantsRed.specimenDeliverPose2.position.y)
//                        .build();//place second specimen
//
//                secondSpecimenDeliverBackupAction = drive.actionBuilder(FieldConstantsRed.specimenDeliverPose2)
//                        .lineToY(FieldConstantsRed.specimenDeliverApproachPose2.position.y)
//                        .build();//clear submersible
//
//                secondSamplePickupMoveAction = drive.actionBuilder(FieldConstantsRed.specimenDeliverApproachPose2)
//                        .strafeToLinearHeading(FieldConstantsRed.midRedPickupPose.position, FieldConstantsRed.midRedPickupPose.heading)
//                        .build();//move to pickup inner sample
//
//                thirdSpecimenPickupMoveAction = drive.actionBuilder(FieldConstantsRed.midRedPickupPose)
//                        .strafeToLinearHeading(FieldConstantsRed.specimenPickupPose.position, FieldConstantsRed.specimenPickupPose.heading)
//                        .build();//move to drop second sample and pick up third specimen
//
//                thirdSpecimenDeliverMoveAction = drive.actionBuilder(FieldConstantsRed.specimenPickupPose)
//                        .splineToLinearHeading(FieldConstantsRed.specimenDeliverApproachPose3, FieldConstantsRed.specimenDeliverApproachPose3.heading)
//                        .lineToY(FieldConstantsRed.specimenDeliverPose3.position.y)
//                        .build();//place thirdspecimen
//
//                thirdSpecimenDeliverBackupAction = drive.actionBuilder(FieldConstantsRed.specimenDeliverPose3)
//                        .lineToY(FieldConstantsRed.specimenDeliverApproachPose3.position.y)
//                        .build();
//
//                thirdSamplePickupMoveAction = drive.actionBuilder(FieldConstantsRed.specimenDeliverApproachPose3)
//                        .strafeToLinearHeading(FieldConstantsRed.outerRedPickupPose.position, FieldConstantsRed.outerRedPickupPose.heading)
//                        .build();//move to pickup inner sample
//
//                fourthSpecimenPickupMoveAction = drive.actionBuilder(FieldConstantsRed.outerRedPickupPose)
//                        .strafeToLinearHeading(FieldConstantsRed.specimenPickupPose.position, FieldConstantsRed.specimenPickupPose.heading)
//                        .build();//move to drop third sample and pick up fourth specimen
//
//                fourthSpecimenDeliverMoveAction = drive.actionBuilder(FieldConstantsRed.specimenPickupPose)
//                        .splineToLinearHeading(FieldConstantsRed.specimenDeliverApproachPose4, FieldConstantsRed.specimenDeliverApproachPose4.heading)
//                        .lineToY(FieldConstantsRed.specimenDeliverPose4.position.y)
//                        .build();//deliver fourth specimen


//                deliverFourSpecimens = new SequentialAction(
//
//                        firstSpecimenDeliverMoveAction,
//                        placeSpecimenAction,
//                        firstSpecimenDeliverBackupAction,
//                        new ParallelAction(
//                                firstSamplePickupMoveAction,
//                                pickupSampleAction),
//                        new ParallelAction(
//                                secondSpecimenPickupMoveAction,
//                                transferSampleToBucketAction),
//                        new ParallelAction(
//                                collectSpecimenAction,
//                                dropSampleAction),
//                        secondSpecimenDeliverMoveAction,
//                        placeSpecimenAction,
//                        secondSpecimenDeliverBackupAction,
//                        new ParallelAction(
//                                secondSamplePickupMoveAction,
//                                pickupSampleAction),
//                        new ParallelAction(
//                                thirdSpecimenPickupMoveAction,
//                                transferSampleToBucketAction),
//                        new ParallelAction(
//                                collectSpecimenAction,
//                                dropSampleAction),
//                        thirdSpecimenDeliverMoveAction,
//                        placeSpecimenAction,
//                        thirdSpecimenDeliverBackupAction,
//                        new ParallelAction(
//                                thirdSamplePickupMoveAction,
//                                pickupSampleAction),
//                        new ParallelAction(
//                                fourthSpecimenPickupMoveAction,
//                                transferSampleToBucketAction),
//                        new ParallelAction(
//                                collectSpecimenAction,
//                                dropSampleAction),
//                        fourthSpecimenDeliverMoveAction,
//                        placeSpecimenAction
//
//
//                );
                break;
            }
            if (gamepad1.x) {

                telemetry.clearAll();
                telemetry.addData("BLUE ", "Chosen");
                telemetry.addData("Restart OpMode ", "to Change");


                drive.currentteam = PoseStorage.Team.BLUE;

                drive.pose = FieldConstantsBlue.basketSideStartPose;


                firstSpecimenDeliverMoveAction = drive.actionBuilder(FieldConstantsBlue.specimenSideStartPose)
                        .lineToY(FieldConstantsBlue.specimenDeliverApproachPose1.position.y)
                        .lineToY(FieldConstantsBlue.specimenDeliverPose1.position.y)
                        .build();//move to place first specimen

                firstSampleMoveToObservationZoneAction = drive.actionBuilder(FieldConstantsBlue.specimenDeliverPose1)
                        .splineToSplineHeading(new Pose2d(-36, 42, Math.toRadians(180)), Math.toRadians(180))
                        .strafeTo(new Vector2d(-36, 10))
                        .strafeTo(new Vector2d(-48, 10))
                        .strafeTo(FieldConstantsBlue.sample1ObservationZoneDropPose.position)
                        .strafeTo(new Vector2d(FieldConstantsBlue.sample1ObservationZoneDropPose.position.x, FieldConstantsBlue.sample1ObservationZoneDropPose.position.y - 10))
                        .build();

                secondSpecimenPickupMoveAction = drive.actionBuilder(FieldConstantsBlue.sample1ObservationZoneDropPose)
                        .lineToXSplineHeading(FieldConstantsBlue.specimenPrePickupPose.position.x, Math.toRadians(90))
                        .strafeTo(FieldConstantsBlue.specimenPickupPose.position)
                        .waitSeconds(1)
                        .build();

                secondSpecimenDeliverMoveAction = drive.actionBuilder(FieldConstantsBlue.specimenPickupPose)
                        .splineToLinearHeading(FieldConstantsBlue.specimenDeliverApproachPose2, Math.toRadians(-90))
                        .lineToY(FieldConstantsBlue.specimenDeliverPose2.position.y)
                        .build();//place second specimen

                thirdSpecimenPickupMoveAction = drive.actionBuilder(FieldConstantsBlue.specimenDeliverPose2)
                        //.lineToY(FieldConstantsBlue.specimenDeliverApproachPose2.position.y)
                        .splineToLinearHeading(FieldConstantsBlue.specimenPrePickupPose, Math.toRadians(0))
                        .waitSeconds(.2)
                        .strafeTo(FieldConstantsBlue.specimenPickupPose.position)
                        .waitSeconds(1)
                        .build();

                thirdSpecimenDeliverMoveAction = drive.actionBuilder(FieldConstantsBlue.specimenPickupPose)
                        .splineToLinearHeading(FieldConstantsBlue.specimenDeliverApproachPose3, Math.toRadians(-90))
                        .lineToY(FieldConstantsBlue.specimenDeliverPose3.position.y)
                        .build();//place second specimen

//        fourthSpecimenPickupMoveAction = drive.actionBuilder(FieldConstantsBlue.specimenDeliverPose3)
//                .lineToY(FieldConstantsBlue.specimenDeliverApproachPose3.position.y)
//                .splineToLinearHeading(FieldConstantsBlue.specimenPrePickupPose, Math.toRadians(180))
//                .strafeTo(FieldConstantsBlue.specimenPickupPose.position)
//                .waitSeconds(1)
//                .build();
//
//        fourthSpecimenDeliverMoveAction = drive.actionBuilder(FieldConstantsBlue.specimenPickupPose)
//                .splineToLinearHeading(FieldConstantsBlue.specimenDeliverApproachPose4, Math.toRadians(-90))
//                .lineToY(FieldConstantsBlue.specimenDeliverPose4.position.y)
//                .build();//place second specimen

                secondSampleMoveToObservationZoneAction = drive.actionBuilder(FieldConstantsBlue.specimenDeliverPose3)
                        .splineToSplineHeading(new Pose2d(-22, 48, Math.toRadians(180)), Math.toRadians(180))
                        .strafeTo(new Vector2d(-56, 10))
                        .strafeTo(FieldConstantsBlue.sample2ObservationZoneDropPose.position)

//                .strafeTo(new Vector2d(-48, 10))
//                .strafeTo(new Vector2d(-58, 10))
                        //  .strafeTo(FieldConstantsBlue.sample2ObservationZoneDropPose.position)
                        .build();

//        parkAction = drive.actionBuilder(FieldConstantsBlue.specimenDeliverPose4)
//                .splineToLinearHeading(FieldConstantsBlue.parkPose, Math.toRadians(180))
//
//                .waitSeconds(1)
//                .build();

                deliverThreeSpecimens =
                        new SequentialAction(

                                firstSpecimenDeliverMoveAction,
                                placeSpecimenAction,
                                firstSampleMoveToObservationZoneAction,
                                //  secondSampleMoveToObservationZoneAction,
                                secondSpecimenPickupMoveAction,
                                secondSpecimenDeliverMoveAction,
                                thirdSpecimenPickupMoveAction,
                                thirdSpecimenDeliverMoveAction,

                                //  fourthSpecimenDeliverMoveAction,
                                //  parkAction
                                secondSampleMoveToObservationZoneAction

                        );


                break;
            }
            telemetry.update();
        }
        telemetry.clearAll();
    }


    //method to wait safely with stop button working if needed. Use this instead of sleep
    public void safeWaitSeconds(double time) {
        ElapsedTime timer = new ElapsedTime(SECONDS);
        timer.reset();
        while (!isStopRequested() && timer.time() < time) {
        }
    }


    //Define and declare Robot Starting Locations
    public enum START_POSITION {
        BLUE_LEFT,
        BLUE_RIGHT,
        RED_LEFT,
        RED_RIGHT
    }

}
