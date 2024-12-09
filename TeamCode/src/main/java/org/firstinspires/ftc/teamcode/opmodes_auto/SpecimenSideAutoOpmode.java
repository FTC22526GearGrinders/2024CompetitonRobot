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
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.FieldConstantsSelect;
import org.firstinspires.ftc.teamcode.commands_actions.combined.Elevator_Arm_RotateArm_Actions;
import org.firstinspires.ftc.teamcode.subsystems.ElevatorSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ExtendArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.RotateArmSubsystem;
import org.firstinspires.ftc.teamcode.utils.PoseStorage;


@Autonomous(name = "Specimen Fast With Approach", group = "Auto")
//@Disabled
public class SpecimenSideAutoOpmode extends CommandOpMode {

    public static String TEAM_NAME = "Gear Grinders"; // Enter team Name
    public static int TEAM_NUMBER = 22526; //Enter team Number


    FieldConstantsSelect fcs;
    Action firstSpecimenDeliverMove;
    TrajectoryActionBuilder firstSampleMoveToObservationZone;
    TrajectoryActionBuilder secondSampleMoveToObservationZoneApproach;
    Action secondSampleApproachFinalMove;
    TrajectoryActionBuilder secondSpecimenDeliverMove;
    Action secondSpecimenDeliverFinalMove;
    TrajectoryActionBuilder thirdSpecimenPickupMove;
    Action thirdSpecimenPickupFinalMove;
    TrajectoryActionBuilder thirdSpecimenDeliverMove;
    Action thirdSpecimenDeliverFinalMove;
    TrajectoryActionBuilder fourthSpecimenPickupMove;
    Action fourthSpecimenPickupFinalMove;
    TrajectoryActionBuilder fourthSpecimenDeliverMove;
    Action fourthSpecimenDeliverFinalMove;
    TrajectoryActionBuilder park;

    TranslationalVelConstraint approachVel;
    ProfileAccelConstraint approachAccel;
    private MecanumDriveSubsystem drive;
    private ElevatorSubsystem elevator;
    private ExtendArmSubsystem arm;
    private RotateArmSubsystem rotate;
    private Elevator_Arm_RotateArm_Actions ears;

    private TelemetryPacket packet;

    private SequentialAction doSpecimens;

    @Override
    public void initialize() {
        drive = new MecanumDriveSubsystem(this, new Pose2d(0, 0, 0));
        elevator = new ElevatorSubsystem(this);
        arm = new ExtendArmSubsystem(this);
        rotate = new RotateArmSubsystem(this);
        ears = new Elevator_Arm_RotateArm_Actions(elevator, arm, rotate, this);
        fcs = new FieldConstantsSelect();
        approachVel = new TranslationalVelConstraint(10.0);
        approachAccel = new ProfileAccelConstraint(-20.0, 20.0);
        packet = new TelemetryPacket();
    }

    void createMotionActions() {

        firstSpecimenDeliverMove = drive.actionBuilder(fcs.specimenSideStartPose)
                .strafeToLinearHeading(fcs.specimenDeliverApproachPose1.position, fcs.specimenDropAngle)
                .strafeTo(fcs.specimenDeliverPose1.position,
                        approachVel, approachAccel
                ).build();

        firstSampleMoveToObservationZone = drive.actionBuilder(fcs.specimenDeliverPose1)
                .strafeToLinearHeading(fcs.firstStagePushInnerPose.position, Math.toRadians(180))
                .strafeToLinearHeading(fcs.secondStagePushInnerVector, Math.toRadians(180))
                .strafeToLinearHeading(fcs.thirdStagePushInnerVector, Math.toRadians(180))
                .strafeToLinearHeading(fcs.sample1ObservationZoneDropPose.position, Math.toRadians(180));

        secondSampleMoveToObservationZoneApproach = drive.actionBuilder(fcs.sample1ObservationZoneDropPose)
                .strafeToLinearHeading(fcs.secondStagePushMidVector, fcs.specimenPickupAngle)
                .strafeToLinearHeading(fcs.thirdStagePushMidVector, fcs.specimenPickupAngle)
                .strafeToLinearHeading(fcs.sample2ObservationZoneDropPose.position, fcs.specimenPickupAngle);


        secondSampleApproachFinalMove = secondSampleMoveToObservationZoneApproach.endTrajectory().fresh()
                .strafeToLinearHeading(fcs.sample2ObservationZonePickupPose.position, fcs.specimenPickupAngle,
                        approachVel, approachAccel).build();


        secondSpecimenDeliverMove = drive.actionBuilder(fcs.sample2ObservationZonePickupPose)
                .splineToLinearHeading(fcs.specimenDeliverApproachPose2, fcs.specimenPickupAngle);


        secondSpecimenDeliverFinalMove = secondSpecimenDeliverMove.endTrajectory().fresh()
                .strafeToLinearHeading(fcs.specimenDeliverPose2.position, fcs.specimenDropAngle,
                        approachVel, approachAccel).build();


        thirdSpecimenPickupMove = drive.actionBuilder(fcs.specimenDeliverPose2)
                .splineToLinearHeading(fcs.specimenPickupApproachPose, fcs.specimenDropAngle);


        thirdSpecimenPickupFinalMove = thirdSpecimenPickupMove.endTrajectory().fresh()
                .strafeToLinearHeading(fcs.specimenPickupPose.position, fcs.specimenPickupAngle,
                        approachVel, approachAccel).build();


        thirdSpecimenDeliverMove = drive.actionBuilder(fcs.specimenPickupPose)
                .splineToLinearHeading(fcs.specimenDeliverApproachPose3, fcs.specimenPickupAngle);

        thirdSpecimenDeliverFinalMove = thirdSpecimenDeliverMove.endTrajectory().fresh()
                .strafeToLinearHeading(fcs.specimenDeliverPose3.position, fcs.specimenDropAngle,
                        approachVel, approachAccel).build();

        fourthSpecimenPickupMove = drive.actionBuilder(fcs.specimenDeliverPose3)
                .splineToLinearHeading(fcs.specimenPickupApproachPose, fcs.specimenDropAngle);

        fourthSpecimenPickupFinalMove = fourthSpecimenPickupMove.endTrajectory().fresh()
                .strafeToLinearHeading(fcs.specimenPickupPose.position, fcs.specimenPickupAngle,
                        approachVel, approachAccel).build();


        fourthSpecimenDeliverMove = drive.actionBuilder(fcs.specimenPickupPose)
                .splineToLinearHeading(fcs.specimenDeliverApproachPose4, fcs.specimenPickupAngle);

        fourthSpecimenDeliverFinalMove = fourthSpecimenDeliverMove.endTrajectory().fresh()
                .strafeToLinearHeading(fcs.specimenDeliverPose4.position, fcs.specimenDropAngle,
                        approachVel, approachAccel).build();

        park = drive.actionBuilder(fcs.specimenDeliverPose4)
                .strafeToLinearHeading(fcs.specimenParkPose.position, fcs.specimenDropAngle);


    }

    private SequentialAction createMotionSequence() {

        return
                new SequentialAction(
                        new ParallelAction(
                                firstSpecimenDeliverMove,
                                elevator.elevatorToAboveUpperSubmersible()),
                        ears.deliverSpecimenToUpperSubmersible(),
                        new ParallelAction(
                                firstSampleMoveToObservationZone.build(),
                                elevator.elevatorToHome()),
                        secondSampleMoveToObservationZoneApproach.build(),
                        secondSampleApproachFinalMove,
                        elevator.grabSpecimenAndClearWall(),
                        new ParallelAction(
                                secondSpecimenDeliverMove.build(),
                                elevator.elevatorToAboveUpperSubmersible()),
                        secondSpecimenDeliverFinalMove,
                        ears.deliverSpecimenToUpperSubmersible(),
                        new ParallelAction(
                                thirdSpecimenPickupMove.build(),
                                elevator.elevatorToHome()),
                        thirdSpecimenPickupFinalMove,
                        elevator.grabSpecimenAndClearWall(),
                        new ParallelAction(
                                thirdSpecimenDeliverMove.build(),
                                elevator.elevatorToAboveUpperSubmersible()),
                        thirdSpecimenDeliverFinalMove,
                        ears.deliverSpecimenToUpperSubmersible(),
                        new ParallelAction(
                                fourthSpecimenPickupMove.build(),
                                elevator.elevatorToHome()),
                        fourthSpecimenPickupFinalMove,
                        elevator.grabSpecimenAndClearWall(),
                        new ParallelAction(
                                fourthSpecimenDeliverMove.build(),
                                elevator.elevatorToAboveUpperSubmersible()),
                        fourthSpecimenDeliverFinalMove,
                        ears.deliverSpecimenToUpperSubmersible(),
                        park.build());

    }

    @Override
    public void runOpMode() throws InterruptedException {

        initialize();

        selectStartingPosition();

        doSpecimens = createMotionSequence();

        waitForStart();

        while (!isStopRequested() && opModeIsActive()) {

            run();

            Actions.runBlocking(doSpecimens);

            telemetry.update();


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
            telemetry.addData("Select Alliance using XA on Logitech (or ▢ΔOX on Playstation) on gamepad 1:", "");
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

                createMotionActions();

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
                doSpecimens = createMotionSequence();

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

