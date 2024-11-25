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


import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.FieldConstantsSelect;
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

    Action secondSpecimenPrePickupMoveAction;
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

    Action parkAction;

    FieldConstantsSelect fcs;


    private MecanumDriveSubsystem drive;


    private LimelightSubsystem limelight;

    private TelemetryPacket packet;

    @Override
    public void initialize() {
        drive = new MecanumDriveSubsystem(this, new Pose2d(0, 0, 0));
        fcs = new FieldConstantsSelect();
        limelight = new LimelightSubsystem(this);
        packet = new TelemetryPacket();


    }

    void makeMotionActions() {

        firstSpecimenDeliverMoveAction = drive.actionBuilder(fcs.specimenSideStartPose)
                .lineToY(fcs.specimenDeliverApproachPose1.position.y)
                .lineToY(fcs.specimenDeliverPose1.position.y)
                .build();//move to place first specimen

        firstSampleMoveToObservationZoneAction = drive.actionBuilder(fcs.specimenDeliverPose1)
                .strafeToLinearHeading(fcs.firstStagePushInnerPose.position, Math.toRadians(180))
                .strafeTo(fcs.secondStagePushInnerVector)
                .strafeTo(fcs.thirdStagePushInnerVector)
                .strafeTo(fcs.sample1ObservationZoneDropPose.position)
                .build();


        secondSampleMoveToObservationZoneAction = drive.actionBuilder(fcs.sample1ObservationZoneDropPose)
                .strafeTo(fcs.firstStagePushMidPose.position)
                .strafeTo(fcs.secondStagePushMidVector)
                .strafeTo(fcs.thirdStagePushMidVector)
                .strafeTo(fcs.sample2ObservationZoneDropPose.position)
                .build();

        secondSpecimenPrePickupMoveAction = drive.actionBuilder(fcs.sample2ObservationZoneDropPose)
                .lineToXLinearHeading(fcs.specimenPickupApproachPose.position.x, fcs.specimenPickupAngle)
                .strafeTo(fcs.specimenPickupPose.position)
                .waitSeconds(1)
                .build();

        secondSpecimenDeliverMoveAction = drive.actionBuilder(fcs.specimenPickupPose)
                .strafeToLinearHeading(fcs.specimenDeliverApproachPose2.position, fcs.specimenDropAngle)
                .strafeTo(fcs.specimenDeliverPose2.position)
                .build();//place second specimen

        thirdSpecimenPickupMoveAction = drive.actionBuilder(fcs.specimenDeliverPose2)
                .strafeToLinearHeading(fcs.specimenPickupApproachPose.position, fcs.specimenPickupAngle)
                .strafeTo(fcs.specimenPickupPose.position)
                .waitSeconds(1)
                .build();

        thirdSpecimenDeliverMoveAction = drive.actionBuilder(fcs.specimenPickupPose)
                .strafeToLinearHeading(fcs.specimenDeliverApproachPose3.position, fcs.specimenDropAngle)
                .strafeTo(fcs.specimenDeliverPose3.position)
                .build();//place second specimen

        fourthSpecimenPickupMoveAction = drive.actionBuilder(fcs.specimenDeliverPose3)
                .strafeToLinearHeading(fcs.specimenPickupApproachPose.position, fcs.specimenPickupAngle)
                .strafeTo(fcs.specimenPickupPose.position)
                .waitSeconds(1)
                .build();

        fourthSpecimenDeliverMoveAction = drive.actionBuilder(fcs.specimenPickupPose)
                .strafeToLinearHeading(fcs.specimenDeliverApproachPose4.position, fcs.specimenDropAngle)
                .strafeTo(fcs.specimenDeliverPose4.position)
                .build();//place second specimen

        parkAction = drive.actionBuilder(fcs.specimenDeliverPose4)
                .strafeTo(fcs.specimenParkPose.position)
                .build();


        secondSampleMoveToObservationZoneAction = drive.actionBuilder(fcs.specimenDeliverPose3)
                .splineToSplineHeading(new Pose2d(-22, 48, Math.toRadians(180)), Math.toRadians(180))
                .strafeTo(new Vector2d(-56, 10))
                .strafeTo(fcs.sample2ObservationZoneDropPose.position)

                .build();
    }


    private SequentialAction createMotionSequence() {
        return new SequentialAction(

                firstSpecimenDeliverMoveAction,
                placeSpecimenAction,
                firstSampleMoveToObservationZoneAction,
                secondSampleMoveToObservationZoneAction,
                secondSpecimenPrePickupMoveAction,
                secondSpecimenDeliverMoveAction,
                thirdSpecimenPickupMoveAction,
                thirdSpecimenDeliverMoveAction,
                secondSampleMoveToObservationZoneAction);
    }

    @Override
    public void runOpMode() throws InterruptedException {

        initialize();

        selectStartingPosition();

        deliverThreeSpecimens = createMotionSequence();

        waitForStart();

        while (!isStopRequested() && opModeIsActive()) {

            run();

            telemetry.update();


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

                fcs.setRed();

                drive.currentteam = PoseStorage.Team.RED;

                drive.pose = fcs.specimenSideStartPose;

                break;

            }
            if (gamepad1.x) {

                telemetry.clearAll();
                telemetry.addData("BLUE ", "Chosen");
                telemetry.addData("Restart OpMode ", "to Change");


                drive.currentteam = PoseStorage.Team.BLUE;

                fcs.setBlue();

                drive.pose = fcs.specimenSideStartPose;

                break;
            }


            deliverThreeSpecimens =
                    new SequentialAction(

                            firstSpecimenDeliverMoveAction,
                            placeSpecimenAction,
                            firstSampleMoveToObservationZoneAction,
                            secondSampleMoveToObservationZoneAction,
                            secondSpecimenPrePickupMoveAction,
                            secondSpecimenDeliverMoveAction,
                            thirdSpecimenPickupMoveAction,
                            thirdSpecimenDeliverMoveAction,

                            secondSampleMoveToObservationZoneAction

                    );


            break;
        }
        telemetry.update();
    }
    //    telemetry.clearAll();
}

//method to wait safely with stop button working if needed. Use this instead of sleep
//        public void safeWaitSeconds ( double time){
//            ElapsedTime timer = new ElapsedTime(SECONDS);
//            timer.reset();
//            while (!isStopRequested() && timer.time() < time) {
//            }
//        }

