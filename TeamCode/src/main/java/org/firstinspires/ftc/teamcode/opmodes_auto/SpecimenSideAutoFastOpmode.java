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
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.FieldConstantsSelect;
import org.firstinspires.ftc.teamcode.commands_actions.combined.Elevator_Arm_RotateArm_Actions;
import org.firstinspires.ftc.teamcode.subsystems.ElevatorSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ExtendArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LimelightSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.RotateArmSubsystem;
import org.firstinspires.ftc.teamcode.utils.PoseStorage;


@Autonomous(name = "Specimen Fast", group = "Auto")
//@Disabled
public class SpecimenSideAutoFastOpmode extends CommandOpMode {

    public static String TEAM_NAME = "Gear Grinders"; // Enter team Name
    public static int TEAM_NUMBER = 22526; //Enter team Number


    Action firstSpecimenDeliverMoveAction;
    Action secondSpecimenDeliverMoveAction;
    Action thirdSpecimenDeliverMoveAction;
    Action fourthSpecimenDeliverMoveAction;

    Action secondSpecimenPickupMoveAction;
    Action thirdSpecimenPickupMoveAction;
    Action fourthSpecimenPickupMoveAction;


    Action firstSampleMoveToObservationZoneAction;
    Action secondSampleMoveToObservationZoneAction;

    Action parkAction;

    SequentialAction sequenceOne;
    FieldConstantsSelect fcs;


    private MecanumDriveSubsystem drive;
    private ElevatorSubsystem elevator;
    private ExtendArmSubsystem arm;
    private RotateArmSubsystem rotate;

    private Elevator_Arm_RotateArm_Actions ears;


    private LimelightSubsystem limelight;

    private TelemetryPacket packet;

    @Override
    public void initialize() {
        drive = new MecanumDriveSubsystem(this, new Pose2d(0, 0, 0));
        elevator = new ElevatorSubsystem(this);
        arm = new ExtendArmSubsystem(this);
        rotate = new RotateArmSubsystem(this);
        ears = new Elevator_Arm_RotateArm_Actions(elevator, arm, rotate, this);
        fcs = new FieldConstantsSelect();
        limelight = new LimelightSubsystem(this);
        packet = new TelemetryPacket();
    }

    void makeMotionActions() {

        firstSpecimenDeliverMoveAction = drive.actionBuilder(fcs.specimenSideStartPose)
                .strafeToLinearHeading(fcs.specimenDeliverPose1.position, fcs.specimenDropAngle)
                .build();//move to place first specimen

        firstSampleMoveToObservationZoneAction = drive.actionBuilder(fcs.specimenDeliverPose1)
                .strafeToLinearHeading(fcs.firstStagePushInnerPose.position, Math.toRadians(180))
                .strafeTo(fcs.secondStagePushInnerVector)
                .strafeTo(fcs.thirdStagePushInnerVector)
                .strafeTo(fcs.sample1ObservationZoneDropPose.position)
                .build();

        secondSampleMoveToObservationZoneAction = drive.actionBuilder(fcs.sample1ObservationZoneDropPose)
                .strafeToLinearHeading(fcs.secondStagePushInnerVector, fcs.specimenPickupAngle)
                .strafeTo(fcs.thirdStagePushMidVector)
                .strafeTo(fcs.sample2ObservationZoneDropPoseFast.position)
                .build();

        secondSpecimenDeliverMoveAction = drive.actionBuilder(fcs.sample2ObservationZoneDropPoseFast)
                // .splineToSplineHeading(fcs.specimenDeliverPose2,fcs.specimenPickupAngle)
                .splineToLinearHeading(fcs.specimenDeliverPose2, Math.toRadians(90))
                .build();//move to place first specimen

        thirdSpecimenPickupMoveAction = drive.actionBuilder(fcs.specimenDeliverPose2)
                //.splineToSplineHeading(fcs.specimenPickupPose, fcs.specimenDropAngle)
                .splineToLinearHeading(fcs.specimenPickupPose, Math.toRadians(-90))
                .build();

        thirdSpecimenDeliverMoveAction = drive.actionBuilder(fcs.specimenPickupPose)
                //.splineToSplineHeading(fcs.specimenDeliverPose3, fcs.specimenPickupAngle)
                .splineToLinearHeading(fcs.specimenDeliverPose3, Math.toRadians(90))
                .build();//move to place first specimen

        fourthSpecimenPickupMoveAction = drive.actionBuilder(fcs.specimenDeliverPose3)
                .splineToLinearHeading(fcs.specimenPickupPose, Math.toRadians(-90))
                .build();

        fourthSpecimenDeliverMoveAction = drive.actionBuilder(fcs.specimenPickupPose)
                .splineToLinearHeading(fcs.specimenDeliverPose4, Math.toRadians(90))
                .build();//move to place first specimen

        parkAction = drive.actionBuilder(fcs.specimenDeliverPose4)
                .strafeTo(fcs.specimenParkPose.position)
                .build();


    }


    private SequentialAction createSequenceOne() {
        return
                new SequentialAction(
                        new ParallelAction(
                                elevator.closeSpecimenClaw(),
                                firstSpecimenDeliverMoveAction,
                                elevator.elevatorToAboveUpperSubmersible()),
                        ears.deliverSpecimenToUpperSubmersible(),

                        new ParallelAction(
                                firstSampleMoveToObservationZoneAction,
                                elevator.elevatorToHome()),
                        new ParallelAction(
                                secondSampleMoveToObservationZoneAction,
                                elevator.elevatorToSpecimenOnWall()),

                        elevator.closeSpecimenClaw(),

                        new ParallelAction(
                                secondSpecimenDeliverMoveAction,
                                elevator.elevatorToAboveUpperSubmersible()),
                        ears.deliverSpecimenToUpperSubmersible(),

                        new ParallelAction(
                                thirdSpecimenPickupMoveAction,
                                elevator.elevatorToSpecimenOnWall()),
                        elevator.closeSpecimenClaw(),


                        new ParallelAction(
                                thirdSpecimenDeliverMoveAction,
                                elevator.elevatorToAboveUpperSubmersible()),
                        ears.deliverSpecimenToUpperSubmersible(),

                        new ParallelAction(
                                fourthSpecimenPickupMoveAction,
                                elevator.elevatorToSpecimenOnWall()),
                        elevator.closeSpecimenClaw(),

                        new ParallelAction(
                                fourthSpecimenDeliverMoveAction,
                                elevator.elevatorToAboveUpperSubmersible()),
                        ears.deliverSpecimenToUpperSubmersible(),

                        parkAction);
    }

    @Override
    public void runOpMode() throws InterruptedException {

        initialize();

        selectStartingPosition();

        sequenceOne = createSequenceOne();

        waitForStart();

        while (!isStopRequested() && opModeIsActive()) {

            run();

            telemetry.update();


            Actions.runBlocking(sequenceOne);


        }
        PoseStorage.currentPose = drive.pose;
        PoseStorage.poseUpdatedTime = System.currentTimeMillis();


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

                PoseStorage.currentTeam = PoseStorage.Team.RED;

                fcs.setRed();
                drive.pose = fcs.specimenSideStartPose;
                drive.startRadians = fcs.specimenSideStartPose.heading.toDouble();

                makeMotionActions();

                break;

            }
            if (gamepad1.x) {

                telemetry.clearAll();
                telemetry.addData("BLUE ", "Chosen");
                telemetry.addData("Restart OpMode ", "to Change");

                PoseStorage.currentTeam = PoseStorage.Team.BLUE;

                fcs.setBlue();

                drive.pose = fcs.specimenSideStartPose;
                drive.startRadians = fcs.specimenSideStartPose.heading.toDouble();
                makeMotionActions();

                break;
            }

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

