package org.firstinspires.ftc.teamcode.opmodes_test;


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
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.FieldConstantsSelect;
import org.firstinspires.ftc.teamcode.commands_actions.combined.Elevator_Arm_RotateArm_Actions;
import org.firstinspires.ftc.teamcode.subsystems.ElevatorSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ExtendArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LimelightSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.RotateArmSubsystem;
import org.firstinspires.ftc.teamcode.utils.PoseStorage;


@Autonomous(name = "Basket Motion", group = "Auto")
//@Disabled
public class BasketSideMotionOpmode extends CommandOpMode {

    public String TEAM_NAME = "Gear Grinders"; // Enter team Name
    public int TEAM_NUMBER = 22526; //Enter team Number
    public TrajectoryActionBuilder firstSampleDeliverMove;

    TrajectoryActionBuilder secondSampleDeliverMove;
    TrajectoryActionBuilder thirdSampleDeliverMove;
    TrajectoryActionBuilder fourthSampleDeliverMove;


    TrajectoryActionBuilder secondSamplePickupMove;
    Action secondSamplePickupFinalMove;
    TrajectoryActionBuilder thirdSamplePickupMove;
    Action thirdSamplePickupFinalMove;
    TrajectoryActionBuilder fourthSamplePickupMove;
    Action fourthSamplePickupFinalMove;
    Action parkAction;

    TranslationalVelConstraint approachVel;
    ProfileAccelConstraint approachAccel;

    SequentialAction deliverFourSamples;
    double samplePickupTimeout = 3;
    Gamepad currentGamepad1 = new Gamepad();
    Gamepad currentGamepad2 = new Gamepad();
    Gamepad previousGamepad1 = new Gamepad();
    Gamepad previousGamepad2 = new Gamepad();
    private MecanumDriveSubsystem drive;
    private ElevatorSubsystem elevator;
    private ExtendArmSubsystem arm;
    private RotateArmSubsystem rotate;
    private Elevator_Arm_RotateArm_Actions ears;
    private LimelightSubsystem limelight;
    private TelemetryPacket packet;
    private FieldConstantsSelect fcs;

    @Override
    public void initialize() {
        drive = new MecanumDriveSubsystem(this, new Pose2d(0, 0, 0));
        ;
        packet = new TelemetryPacket();
        fcs = new FieldConstantsSelect();
    }

    void createMotionActions() {

        drive.pose = fcs.basketSideStartPose;

        firstSampleDeliverMove = drive.actionBuilder(fcs.basketSideStartPose)
                .strafeToLinearHeading(fcs.basketDeliverPose.position, fcs.basketDeliverPose.heading);

        secondSamplePickupMove = drive.actionBuilder(fcs.basketDeliverPose)
                .strafeToLinearHeading(fcs.innerYellowPrePickupPose.position, fcs.innerYellowPrePickupPose.heading);

        secondSamplePickupFinalMove = secondSamplePickupMove.endTrajectory().fresh()
                .strafeToLinearHeading(fcs.innerYellowPickupPose.position, fcs.innerYellowPickupPose.heading,
                        approachVel, approachAccel).build();

        secondSampleDeliverMove = drive.actionBuilder(fcs.innerYellowPickupPose)
                .strafeToLinearHeading(fcs.basketDeliverPose.position, fcs.basketDeliverPose.heading);


        thirdSamplePickupMove = drive.actionBuilder(fcs.basketDeliverPose)
                .strafeToLinearHeading(fcs.midYellowPrePickupPose.position, fcs.midYellowPrePickupPose.heading);

        thirdSamplePickupFinalMove = thirdSamplePickupMove.endTrajectory().fresh()
                .strafeToLinearHeading(fcs.midYellowPickupPose.position, fcs.midYellowPickupPose.heading,
                        approachVel, approachAccel).build();

        thirdSampleDeliverMove = drive.actionBuilder(fcs.midYellowPickupPose)
                .strafeToLinearHeading(fcs.basketDeliverPose.position, fcs.basketDeliverPose.heading);

        fourthSamplePickupMove = drive.actionBuilder(fcs.basketDeliverPose)
                .strafeToLinearHeading(fcs.outerYellowPrePose.position, fcs.outerYellowPrePose.heading);

        fourthSamplePickupFinalMove = fourthSamplePickupMove.endTrajectory().fresh()
                .strafeToLinearHeading(fcs.outerYellowPickupPose.position, fcs.outerYellowPickupPose.heading,
                        approachVel, approachAccel).build();

        fourthSampleDeliverMove = drive.actionBuilder(fcs.outerYellowPickupPose)
                .strafeToLinearHeading(fcs.basketDeliverPose.position, fcs.basketDeliverPose.heading);

        parkAction = drive.actionBuilder(fcs.basketDeliverPose)
                .strafeToLinearHeading(fcs.ascentZoneParkPose.position, fcs.ascentZoneParkPose.heading)
                .build();

    }


//    private SequentialAction createMotionSequence() {
//
//        return
//                new SequentialAction(
//
//                        new ParallelAction(
//                                firstSampleDeliverMove.build(),
//                                elevator.elevatorToUpperBasket()),
//                        elevator.cycleBucket(),
//
//                        new ParallelAction(
//                                elevator.elevatorToHome(),
//                                secondSamplePickupMove.build(),
//                                ears.armOutTiltAboveSamplesOpenClaw(1)),
//
//                        secondSamplePickupFinalMove,
//                        rotate.tiltToPickupThenCloseIntakeClaw(.5),
//                        rotate.tiltBothAboveSamples(.5),
//
//                        new ParallelAction(
//                                secondSampleDeliverMove.build(),
//                                new SequentialAction(
//                                        ears.tiltClearArmToBucket(1),
//                                        rotate.openIntakeClaw())),
//                        elevator.elevatorToUpperBasket(),
//                        elevator.cycleBucket(),
//
//
//                        new ParallelAction(
//                                elevator.elevatorToHome(),
//                                thirdSamplePickupMove.build(),
//                                ears.armOutTiltAboveSamplesOpenClaw(1)),
//
//                        thirdSamplePickupFinalMove,
//                        rotate.tiltToPickupThenCloseIntakeClaw(.5),
//                        rotate.tiltBothAboveSamples(.5),
//
//                        new ParallelAction(
//                                thirdSampleDeliverMove.build(),
//                                new SequentialAction(
//                                        ears.tiltClearArmToBucket(1),
//                                        rotate.openIntakeClaw())),
//                        elevator.elevatorToUpperBasket(),
//                        elevator.cycleBucket(),
//
//
//                        new ParallelAction(
//                                elevator.elevatorToHome(),
//                                fourthSamplePickupMove.build(),
//                                ears.armOutTiltAboveSamplesOpenClaw(1)),
//
//                        fourthSamplePickupFinalMove,
//                        rotate.tiltToPickupThenCloseIntakeClaw(.5),
//                        rotate.tiltBothAboveSamples(.5),
//
//                        new ParallelAction(
//                                fourthSampleDeliverMove.build(),
//                                new SequentialAction(
//                                        ears.tiltClearArmToBucket(1),
//                                        rotate.openIntakeClaw())),
//                        elevator.elevatorToUpperBasket(),
//                        elevator.cycleBucket(),
//
//                        new ParallelAction(
//                                elevator.elevatorToHome(),
//                                parkAction));
//
//
//    }


    @Override
    public void runOpMode() throws InterruptedException {

        initialize();

        selectStartingPosition();

        //  deliverFourSamples = createMotionSequence();

        waitForStart();

        while (!isStopRequested() && opModeIsActive()) {

            run();

            if (currentGamepad1.left_bumper && !previousGamepad1.left_bumper)
                Actions.runBlocking(firstSampleDeliverMove.build());

            if (currentGamepad1.right_bumper && !previousGamepad1.right_bumper)
                Actions.runBlocking(new SequentialAction(
                        secondSamplePickupMove.build(),
                        secondSamplePickupFinalMove)
                );

            if (currentGamepad1.a && !previousGamepad1.a)
                Actions.runBlocking(secondSampleDeliverMove.build());

            if (currentGamepad1.b && !previousGamepad1.b)
                Actions.runBlocking(new SequentialAction(
                        thirdSamplePickupMove.build(),
                        thirdSamplePickupFinalMove)
                );

            if (currentGamepad1.x && !previousGamepad1.x)
                Actions.runBlocking(thirdSampleDeliverMove.build());

            if (currentGamepad1.y && !previousGamepad1.y)
                Actions.runBlocking(new SequentialAction(
                        fourthSamplePickupMove.build(),
                        fourthSamplePickupFinalMove)
                );

            if (currentGamepad1.dpad_up && !previousGamepad1.dpad_up)
                Actions.runBlocking(fourthSampleDeliverMove.build());


            if (currentGamepad1.dpad_left && !previousGamepad1.dpad_left)
                Actions.runBlocking(parkAction);


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
                drive.pose = fcs.basketSideStartPose;
                drive.startRadians = fcs.basketSideStartPose.heading.toDouble();
                createMotionActions();


            }

            if (gamepad1.x) {
                telemetry.clearAll();
                telemetry.addData("BLUE ", "Chosen");
                telemetry.addData("Restart OpMode ", "to Change");

                PoseStorage.currentTeam = PoseStorage.Team.BLUE;

                fcs.setBlue();

                drive.pose = fcs.basketSideStartPose;
                drive.startRadians = fcs.basketSideStartPose.heading.toDouble();
                createMotionActions();
            }


            telemetry.update();
        }


//        public void safeWaitSeconds ( double time){
//            ElapsedTime timer = new ElapsedTime(SECONDS);
//            timer.reset();
//            while (!isStopRequested() && timer.time() < time) {
//            }
//        }


    }

}