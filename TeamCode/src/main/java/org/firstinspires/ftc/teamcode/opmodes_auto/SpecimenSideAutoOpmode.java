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
import com.acmerobotics.roadrunner.ftc.Actions;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.FieldConstantsSelect;
import org.firstinspires.ftc.teamcode.commands_actions.arm.PositionHoldArm;
import org.firstinspires.ftc.teamcode.commands_actions.combined.Elevator_Arm_RotateArm_Actions;
import org.firstinspires.ftc.teamcode.commands_actions.elevator.PositionHoldElevator;
import org.firstinspires.ftc.teamcode.subsystems.ElevatorSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ExtendArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.RotateArmSubsystem;
import org.firstinspires.ftc.teamcode.utils.PoseStorage;


@Autonomous(name = "Specimen With Approaches", group = "Auto")
//@Disabled
public class SpecimenSideAutoOpmode extends CommandOpMode {

    public static String TEAM_NAME = "Gear Grinders"; // Enter team Name
    public static int TEAM_NUMBER = 22526; //Enter team Number


    FieldConstantsSelect fcs;
    Action firstSpecimenDeliverMove;
    Action firstSampleMoveToObservationZone;
    Action secondSampleMoveToObservationZone;
    Action secondSampleMoveToObservationZonePickup;

    Action secondSpecimenDeliverApproachMove;
    Action secondSpecimenDeliverMove;

    Action thirdSpecimenPickupApproachMove;
    Action thirdSpecimenPickupMove;

    Action thirdSpecimenDeliverMove;
    Action thirdSpecimenDeliverApproachMove;

    Action fourthSpecimenPickupApproachMove;
    Action fourthSpecimenPickupMove;

    Action fourthSpecimenDeliverApproachMove;
    Action fourthSpecimenDeliverMove;

    Action park;

    Gamepad currentGamepad1 = new Gamepad();
    Gamepad previousGamepad1 = new Gamepad();
    boolean red = false;
    boolean blue = false;
    private MecanumDriveSubsystem drive;
    private ElevatorSubsystem elevator;
    private ExtendArmSubsystem arm;
    private RotateArmSubsystem rotate;
    private Elevator_Arm_RotateArm_Actions ears;
    private TelemetryPacket packet;

    private int showSelect = 0;

    @Override
    public void initialize() {
        drive = new MecanumDriveSubsystem(this, new Pose2d(0, 0, 0));
        elevator = new ElevatorSubsystem(this);
        arm = new ExtendArmSubsystem(this);
        rotate = new RotateArmSubsystem(this);
        ears = new Elevator_Arm_RotateArm_Actions(elevator, arm, rotate, this);
        fcs = new FieldConstantsSelect();
        packet = new TelemetryPacket();

        register(arm, elevator);

        arm.setDefaultCommand(new PositionHoldArm(arm));//set in subsystem eventually since needed in auto and teleop

        elevator.setDefaultCommand(new PositionHoldElevator(elevator));
    }


    void createMotionActions(boolean red) {

        fcs.setBlue();
        PoseStorage.currentTeam = PoseStorage.Team.BLUE;

        if (red) {
            fcs.setRed();
            PoseStorage.currentTeam = PoseStorage.Team.RED;
        }

        drive.pose = fcs.specimenSideStartPose;
        drive.startRadians = drive.pose.heading.toDouble();

        firstSpecimenDeliverMove = drive.actionBuilder(fcs.specimenSideStartPose)
                .strafeToLinearHeading(fcs.specimenDeliverPose1.position, fcs.specimenDropAngle)
                .build();

        firstSampleMoveToObservationZone = drive.actionBuilder(fcs.specimenDeliverPose1)
                .strafeToLinearHeading(fcs.firstStagePushInnerPose.position, Math.toRadians(180))
                .strafeToLinearHeading(fcs.secondStagePushInnerVector, Math.toRadians(180))
                .strafeToLinearHeading(fcs.thirdStagePushInnerVector, Math.toRadians(180))
                .strafeToLinearHeading(fcs.sample1ObservationZoneDropPose.position, Math.toRadians(180)).build();

        secondSampleMoveToObservationZone = drive.actionBuilder(fcs.sample1ObservationZoneDropPose)
                .strafeToLinearHeading(fcs.secondStagePushMidVector, fcs.specimenPickupAngle)
                .strafeToLinearHeading(fcs.thirdStagePushMidVector, fcs.specimenPickupAngle)
                .strafeToLinearHeading(fcs.sample2ObservationZoneDropPose.position, fcs.specimenPickupAngle).build();

        secondSampleMoveToObservationZonePickup = drive.actionBuilder(fcs.sample2ObservationZoneDropPose)
                .strafeToLinearHeading(fcs.sample2ObservationZonePickupPose.position, fcs.specimenPickupAngle).build();


        secondSpecimenDeliverApproachMove = drive.actionBuilder(fcs.sample2ObservationZonePickupPose)
                .splineToLinearHeading(fcs.specimenDeliverApproachPose2, fcs.specimenPickupAngle).build();

        secondSpecimenDeliverMove = drive.actionBuilder(fcs.specimenDeliverApproachPose2)
                .strafeToLinearHeading(fcs.specimenDeliverPose2.position, fcs.specimenDropAngle).build();


        thirdSpecimenPickupApproachMove = drive.actionBuilder(fcs.specimenDeliverPose2)
                .splineToLinearHeading(fcs.specimenPickupApproachPose, fcs.specimenDropAngle).build();

        thirdSpecimenPickupMove = drive.actionBuilder(fcs.specimenPickupApproachPose)
                .strafeToLinearHeading(fcs.specimenPickupPose.position, fcs.specimenPickupAngle).build();


        thirdSpecimenDeliverApproachMove = drive.actionBuilder(fcs.specimenPickupPose)
                .splineToLinearHeading(fcs.specimenDeliverApproachPose3, fcs.specimenPickupAngle).build();

        thirdSpecimenDeliverMove = drive.actionBuilder(fcs.specimenPickupApproachPose)
                .strafeToLinearHeading(fcs.specimenDeliverPose3.position, fcs.specimenDropAngle).build();


        fourthSpecimenPickupApproachMove = drive.actionBuilder(fcs.specimenDeliverPose3)
                .splineToLinearHeading(fcs.specimenPickupApproachPose, fcs.specimenDropAngle).build();

        fourthSpecimenPickupMove = drive.actionBuilder(fcs.specimenDeliverApproachPose3)
                .strafeToLinearHeading(fcs.specimenPickupPose.position, fcs.specimenPickupAngle).build();


        fourthSpecimenDeliverApproachMove = drive.actionBuilder(fcs.specimenPickupPose)
                .splineToLinearHeading(fcs.specimenDeliverApproachPose4, fcs.specimenPickupAngle).build();

        fourthSpecimenDeliverMove = drive.actionBuilder(fcs.specimenPickupApproachPose)
                .strafeToLinearHeading(fcs.specimenDeliverPose4.position, fcs.specimenPickupAngle).build();

        park = drive.actionBuilder(fcs.specimenDeliverPose4)
                .strafeToLinearHeading(fcs.specimenParkPose.position, fcs.specimenDropAngle).build();
    }

    private void runOps() {

        Actions.runBlocking(
                new ParallelAction(
                        firstSpecimenDeliverMove,
                        elevator.elevatorToAboveUpperSubmersible()));

        Actions.runBlocking(
                ears.deliverSpecimenToUpperSubmersible());

        Actions.runBlocking(
                new ParallelAction(
                        firstSampleMoveToObservationZone,
                        elevator.elevatorToHome()));

        Actions.runBlocking(
                secondSampleMoveToObservationZone);

        Actions.runBlocking(
                secondSampleMoveToObservationZonePickup);

        Actions.runBlocking(
                elevator.grabSpecimenAndClearWall());

        Actions.runBlocking(
                new ParallelAction(
                        secondSpecimenDeliverApproachMove,
                        elevator.elevatorToAboveUpperSubmersible()));

        Actions.runBlocking(
                secondSpecimenDeliverMove
        );

        Actions.runBlocking(
                ears.deliverSpecimenToUpperSubmersible());


        Actions.runBlocking(
                new ParallelAction(
                        thirdSpecimenPickupApproachMove,
                        elevator.elevatorToHome()));

        Actions.runBlocking(
                thirdSpecimenPickupMove
        );

        Actions.runBlocking(
                elevator.grabSpecimenAndClearWall());

        Actions.runBlocking(
                new ParallelAction(
                        thirdSpecimenDeliverApproachMove,
                        elevator.elevatorToAboveUpperSubmersible()));

        Actions.runBlocking(
                ears.deliverSpecimenToUpperSubmersible());

        Actions.runBlocking(
                new ParallelAction(
                        fourthSpecimenPickupApproachMove,
                        elevator.elevatorToHome()));

        Actions.runBlocking(
                fourthSpecimenPickupMove
        );

        Actions.runBlocking(
                elevator.grabSpecimenAndClearWall());

        Actions.runBlocking(
                new ParallelAction(
                        fourthSpecimenDeliverApproachMove,
                        elevator.elevatorToAboveUpperSubmersible()));

        Actions.runBlocking(
                fourthSpecimenDeliverMove
        );

        Actions.runBlocking(
                ears.deliverSpecimenToUpperSubmersible());

        Actions.runBlocking(
                new ParallelAction(
                        elevator.elevatorToHome(),
                        park));
    }

    @Override
    public void runOpMode() throws InterruptedException {

        initialize();

        selectStartingPosition();

        createMotionActions(red);

        waitForStart();

        elevator.closeSpecimenClaw();

        elevator.showSelect = Constants.ShowTelemetryConstants.showElevatorCommon;

        while (!isStopRequested() && opModeIsActive()) {

            run();

            runOps();
        }

        PoseStorage.currentPose = drive.pose;
        PoseStorage.poseUpdatedTime = System.currentTimeMillis();
        reset();
    }

    //Method to select starting position using X, Y, A, B buttons on gamepad
    public void selectStartingPosition() {
        telemetry.setAutoClear(true);
        telemetry.clearAll();
        previousGamepad1.copy(currentGamepad1);
        currentGamepad1.copy(gamepad1);
        red = false;
        blue = false;
        //******select start pose*****

        telemetry.addData("Initializing Autonomous for Team:",
                TEAM_NAME, " ", TEAM_NUMBER);
        telemetry.addData("---------------------------------------", "");
        telemetry.addData("Select Alliance using XA on Logitech (or ▢ΔOX on Playstation) on gamepad 1:", "");
        telemetry.addData("    Red All Specimen   ", "(A / O)");

        telemetry.addData("    Blue All Specimen    ", "(X / ▢)");

        telemetry.addData("Blue", blue);
        telemetry.addData("Red", red);

        red = false;
        blue = false;
        while (!isStopRequested() && !blue && !red) {

            if (gamepad1.a) {

                telemetry.clearAll();
                telemetry.addData("RED ", "Chosen");
                telemetry.addData("Restart OpMode ", "to Change");

                blue = false;
                red = true;
            }

            if (gamepad1.x) {

                telemetry.clearAll();
                telemetry.addData("BLUE ", "Chosen");
                telemetry.addData("Restart OpMode ", "to Change");

                red = false;
                blue = true;
            }


            telemetry.update();
        }

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

