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
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

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


@Autonomous(name = "Basket", group = "Auto")
//@Disabled
public class BasketSideAutoOpmode extends CommandOpMode {

    public String TEAM_NAME = "Gear Grinders"; // Enter team Name
    public int TEAM_NUMBER = 22526; //Enter team Number


    Action firstSampleStrafeMove;
    Action firstSampleDeliverMove;

    Action secondSampleDeliverMove;
    Action thirdSampleDeliverMove;
    Action fourthSampleDeliverMove;
    Action secondSamplePrePickupMove;
    Action secondSamplePickupMove;
    Action thirdSamplePrePickupMove;
    Action thirdSamplePickupMove;
    Action fourthSamplePrePickupMove;
    Action fourthSamplePickupMove;
    Action parkAction;


    boolean red = false;
    boolean blue = false;
    TranslationalVelConstraint finalVel;
    ProfileAccelConstraint finalAccel;
    private MecanumDriveSubsystem drive;
    private ElevatorSubsystem elevator;
    private ExtendArmSubsystem arm;
    private RotateArmSubsystem rotate;
    private Elevator_Arm_RotateArm_Actions ears;
    private TelemetryPacket packet;
    private FieldConstantsSelect fcs;
    private SequentialAction autoSequence;


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
        finalAccel = new ProfileAccelConstraint(-30, 30);
        finalVel = new TranslationalVelConstraint(30);
        register(arm, elevator);

        arm.setDefaultCommand(new PositionHoldArm(arm));

        elevator.setDefaultCommand(new PositionHoldElevator(elevator));

        drive.showSelect = Constants.ShowTelemetryConstants.showDrive1;
    }

    void createMotionActions(boolean red) {
        fcs.setBlue();
        PoseStorage.currentTeam = PoseStorage.Team.BLUE;

        if (red) {
            fcs.setRed();
            PoseStorage.currentTeam = PoseStorage.Team.RED;
        }

        drive.pose = fcs.basketSideStartPose;

        firstSampleStrafeMove = drive.actionBuilder(fcs.basketSideStartPose)
                .strafeTo(fcs.basketSideStrafePose.position).build();
        // .lineToX(fcs.basketSideStrafePose.position.x).build();

        firstSampleDeliverMove = drive.actionBuilder(fcs.basketSideStrafePose)
                .strafeToLinearHeading(fcs.basketDeliverPose.position, fcs.basketDeliverPose.heading).build();

        secondSamplePrePickupMove = drive.actionBuilder(fcs.basketDeliverPose)
                .strafeToLinearHeading(fcs.innerYellowPrePickupPose.position, fcs.innerYellowPrePickupPose.heading).build();


        secondSamplePickupMove = drive.actionBuilder(fcs.innerYellowPrePickupPose)
                .strafeToLinearHeading(fcs.innerYellowPickupPose.position, fcs.innerYellowPickupPose.heading).build();

        secondSampleDeliverMove = drive.actionBuilder(fcs.innerYellowPickupPose)
                .strafeToLinearHeading(fcs.basketDeliverPose.position, fcs.basketDeliverPose.heading).build();

        thirdSamplePrePickupMove = drive.actionBuilder(fcs.basketDeliverPose)
                .strafeToLinearHeading(fcs.midYellowPrePickupPose.position, fcs.midYellowPrePickupPose.heading).build();

        thirdSamplePickupMove = drive.actionBuilder(fcs.midYellowPrePickupPose)
                .strafeToLinearHeading(fcs.midYellowPickupPose.position, fcs.midYellowPickupPose.heading).build();

        thirdSampleDeliverMove = drive.actionBuilder(fcs.midYellowPickupPose)
                .strafeToLinearHeading(fcs.basketDeliverPose.position, fcs.basketDeliverPose.heading).build();

//        fourthSamplePrePickupMove = drive.actionBuilder(fcs.basketDeliverPose)
//                .strafeToLinearHeading(fcs.outerYellowPrePose.position, fcs.outerYellowPrePose.heading).build();
//
//        fourthSamplePickupMove = drive.actionBuilder(fcs.outerYellowPrePose)
//                .strafeToLinearHeading(fcs.outerYellowPickupPose.position, fcs.outerYellowPickupPose.heading)
//                .build();
//
//        fourthSampleDeliverMove = drive.actionBuilder(fcs.outerYellowPickupPose)
//                .strafeToLinearHeading(fcs.basketDeliverPose.position, fcs.basketDeliverPose.heading).build();

        parkAction = drive.actionBuilder(fcs.basketDeliverPose)
                .strafeToLinearHeading(fcs.ascentZoneParkPose.position, fcs.ascentZoneParkPose.heading).build();

    }

    private void buildSequence() {

        autoSequence = new SequentialAction(

                new ParallelAction(
                        new SequentialAction(
                                firstSampleStrafeMove,
                                firstSampleDeliverMove),
                        elevator.elevatorToUpperBasket()),

                elevator.cycleBucketToVertical(),

                new ParallelAction(
                        new SequentialAction(
                                secondSamplePrePickupMove,
                                secondSamplePickupMove),

                        elevator.elevatorToHome(),
                        new SequentialAction(
                                new SleepAction(1),
                                ears.autoArmOutTiltToPickup())),

                elevator.levelBucket(),

                ears.autoArmPickupThenBucketDrop(),

                new ParallelAction(
                        secondSampleDeliverMove,
                        elevator.elevatorToUpperBasket()),

                elevator.cycleBucketToVertical(),

                new ParallelAction(
                        new SequentialAction(
                                thirdSamplePrePickupMove,
                                thirdSamplePickupMove),

                        elevator.elevatorToHome(),
                        new SequentialAction(
                                new SleepAction(1),
                                ears.autoArmOutTiltToPickup())),

                elevator.levelBucket(),

                ears.autoArmPickupThenBucketDrop(),

                new ParallelAction(
                        thirdSampleDeliverMove,
                        elevator.elevatorToUpperBasket()),

                elevator.cycleBucketToVertical(),

//                new ParallelAction(
//                        fourthSamplePrePickupMove,
//                        elevator.elevatorToHome()),

//                new SequentialAction(
//                        fourthSamplePickupMove,
//                        ears.autoArmOutTiltToPickup()),
//
//                elevator.levelBucket(),
//
//                ears.autoArmPickupThenBucketDrop(),
//
//                new ParallelAction(
//                        fourthSampleDeliverMove,
//                        elevator.elevatorToUpperBasket()),


                new SequentialAction(
                        new ParallelAction(
                                rotate.tiltBothHome(),
                                elevator.elevatorToHome(),
                                parkAction),
                        elevator.levelBucket(),
                        arm.raiseParkArm()));
    }


    @Override
    public void runOpMode() throws InterruptedException {

        initialize();

        selectStartingPosition();

        createMotionActions(red);

        buildSequence();

        waitForStart();

        // elevator.showSelect = Constants.ShowTelemetryConstants.showElevatorCommon;

        elevator.travelBucket().run(packet);

        while (!isStopRequested() && opModeIsActive()) {

            run();

            autoSequence.run(packet);
        }

        PoseStorage.currentPose = drive.pose;
        PoseStorage.poseUpdatedTime = System.currentTimeMillis();

        reset();
    }

    //Method to select starting position using X, Y, A, B buttons on gamepad
    public void selectStartingPosition() {
        telemetry.setAutoClear(true);
        telemetry.clearAll();
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


//        public void safeWaitSeconds ( double time){
//            ElapsedTime timer = new ElapsedTime(SECONDS);
//            timer.reset();
//            while (!isStopRequested() && timer.time() < time) {
//            }
//        }


}