package org.firstinspires.ftc.teamcode;


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
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.commands_actions.combined.Elevator_Arm_RotateArm_Actions;
import org.firstinspires.ftc.teamcode.subsystems.ElevatorSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ExtendArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LimelightSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.RotateArmSubsystem;
import org.firstinspires.ftc.teamcode.utils.PoseStorage;


@Autonomous(name = "Basket", group = "Auto")
//@Disabled
public class BasketSideAutoWithApproachOpmode extends CommandOpMode {

    public String TEAM_NAME = "Gear Grinders"; // Enter team Name
    public int TEAM_NUMBER = 22526; //Enter team Number
    public Action firstSampleDeliverMoveAction;
    Action secondSampleDeliverMoveAction;
    Action thirdSampleDeliverMoveAction;
    Action fourthSampleDeliverMoveAction;
    Action fifthSampleDeliverMoveAction;

    Action secondSamplePickupMoveAction;
    Action thirdSamplePickupMoveAction;
    Action fourthSamplePickupMoveAction;
    Action fifthSamplePickupMoveAction;

    Action parkAction;

    SequentialAction deliverFourSamples;
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
        elevator = new ElevatorSubsystem(this);
        arm = new ExtendArmSubsystem(this);
        rotate = new RotateArmSubsystem(this);
        ears = new Elevator_Arm_RotateArm_Actions(elevator, arm, rotate, this);
        //   limelight = new LimelightSubsystem(this);
        packet = new TelemetryPacket();
        fcs = new FieldConstantsSelect();
    }

    void createMotionActions() {

        drive.pose = fcs.basketSideStartPose;

        firstSampleDeliverMoveAction = drive.actionBuilder(fcs.basketSideStartPose)
                .strafeToLinearHeading(fcs.basketDeliverPose.position, fcs.basketDeliverPose.heading)
                .build();//move to place first specimen

        secondSamplePickupMoveAction = drive.actionBuilder(fcs.basketDeliverPose)
                .strafeToLinearHeading(fcs.innerYellowPrePickupPose.position, fcs.innerYellowPrePickupPose.heading)
                .strafeToLinearHeading(fcs.innerYellowPickupPose.position, fcs.innerYellowPickupPose.heading)
                .build();


        secondSampleDeliverMoveAction = drive.actionBuilder(fcs.innerYellowPickupPose)
                .strafeToLinearHeading(fcs.basketDeliverPose.position, fcs.basketDeliverPose.heading)
                .build();//move to place first specimen

        thirdSamplePickupMoveAction = drive.actionBuilder(fcs.basketDeliverPose)
                .strafeToLinearHeading(fcs.midYellowPrePickupPose.position, fcs.midYellowPrePickupPose.heading)
                .strafeToLinearHeading(fcs.midYellowPickupPose.position, fcs.midYellowPickupPose.heading)
                .build();

        thirdSampleDeliverMoveAction = drive.actionBuilder(fcs.midYellowPickupPose)
                .strafeToLinearHeading(fcs.basketDeliverPose.position, fcs.basketDeliverPose.heading)
                .build();//move to place first specimen

        fourthSamplePickupMoveAction = drive.actionBuilder(fcs.basketDeliverPose)
                .strafeToLinearHeading(fcs.outerYellowApproachPose.position, fcs.outerYellowApproachPose.heading)
                .waitSeconds(.1)
                .lineToX(fcs.outerYellowPickupPose.position.x)

                .build();

        fourthSampleDeliverMoveAction = drive.actionBuilder(fcs.outerYellowPickupPose)
                .lineToX(fcs.outerYellowApproachPose.position.x)
                .strafeToLinearHeading(fcs.basketDeliverPose.position, fcs.basketDeliverPose.heading)
                .build();//

        fifthSamplePickupMoveAction = drive.actionBuilder(fcs.basketDeliverPose)
                .strafeToLinearHeading(fcs.ascentZonePickupPose.position, Math.toRadians(180))
                .build();

        fifthSampleDeliverMoveAction = drive.actionBuilder(fcs.ascentZonePickupPose)
                .strafeToLinearHeading(fcs.basketDeliverPose.position, fcs.basketDeliverPose.heading)
                .build();

        parkAction = drive.actionBuilder(fcs.basketDeliverPose)
                .strafeToLinearHeading(fcs.ascentZoneParkPose.position, Math.toRadians(180))
                .build();

    }

    double samplePickupTimeout = 3;

    private SequentialAction createMotionSequence() {
        return
                new SequentialAction(
                        //move, raise elevator to upper basket
                        new ParallelAction(
                                firstSampleDeliverMoveAction,
                                elevator.elevatorToUpperBasket()),
                        //deliver to basket
                        elevator.cycleBucket(1),

                        //go get second sample and place in bucket
                        new ParallelAction(
                                elevator.elevatorToHome(),
                                secondSamplePickupMoveAction,
                                ears.armOutTiltDownOpenClaw()),
                        rotate.closeIntakeClaw(),

                        //deliver second sample to basket
                        new ParallelAction(
                                secondSampleDeliverMoveAction,
                                elevator.elevatorToUpperBasket()),
                        rotate.openIntakeClaw(),
                        new SleepAction(.5),
                        elevator.cycleBucket(1),

                        //go get third sample and place in bucket
                        new ParallelAction(
                                elevator.elevatorToHome(),
                                thirdSamplePickupMoveAction,
                                ears.armOutTiltDownOpenClaw()),
                        rotate.closeIntakeClaw(),

                        //deliver third sample to basket
                        new ParallelAction(
                                thirdSampleDeliverMoveAction,
                                elevator.elevatorToUpperBasket()),
                        rotate.openIntakeClaw(),
                        new SleepAction(.5),
                        elevator.cycleBucket(1),

                        //go get fourth sample and place in bucket
                        new ParallelAction(
                                elevator.elevatorToHome(),
                                fourthSamplePickupMoveAction,
                                ears.armOutTiltDownOpenClaw()),
                        rotate.closeIntakeClaw(),

                        //deliver third sample to basket
                        new ParallelAction(
                                fourthSampleDeliverMoveAction,
                                elevator.elevatorToUpperBasket()),
                        rotate.openIntakeClaw(),
                        new SleepAction(.5),
                        elevator.cycleBucket(1),

                        new ParallelAction(
                                elevator.elevatorToHome(),
                                parkAction),
                        rotate.tiltToTouchSubmersibleAction());
    }


    @Override
    public void runOpMode() throws InterruptedException {

        initialize();

        selectStartingPosition();

        deliverFourSamples = createMotionSequence();

        waitForStart();

        while (!isStopRequested() && opModeIsActive()) {

            run();

            telemetry.update();

            Actions.runBlocking(deliverFourSamples);
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