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
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.FieldConstantsBlue;
import org.firstinspires.ftc.teamcode.FieldConstantsRed;
import org.firstinspires.ftc.teamcode.subsystems.ElevatorSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ExtendArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;


@Autonomous(name = "Auto:Test", group = "Auto")
//@Disabled
public class BasketSideAutoOpmode extends CommandOpMode {

    public static String TEAM_NAME = "Gear Grinders"; // Enter team Name
    public static int TEAM_NUMBER = 22526; //Enter team Number
    public static Pose2d startPosition;
    private MecanumDriveSubsystem drive;
    private ExtendArmSubsystem arm;
    private ElevatorSubsystem elevator;
    Action firstSampleDeliverMoveAction;
    Action secondSampleDeliverMoveAction;
    Action thirdSampleDeliverMoveAction;
    Action fourthSampleDeliverMoveAction;

    Action secondSamplePickupMoveAction;
    Action thirdSamplePickupMoveAction;
    Action fourthSamplePickupMoveAction;

    Action placeSpecimenAction = new SleepAction(2);
    Action pickupSampleAction = new SleepAction(2);
    Action transferSampleToBucketAction = new SleepAction(2);
    Action dropSampleAction = new SleepAction(2);


    Action deliverFourSamples;

    private TelemetryPacket packet;

    @Override
    public void initialize() {
        drive = new MecanumDriveSubsystem(this, new Pose2d(0, 0, 0));
        arm = new ExtendArmSubsystem(this);
        elevator = new ElevatorSubsystem(this);
        packet = new TelemetryPacket();

    }

    @Override
    public void runOpMode() throws InterruptedException {

        initialize();

        waitForStart();

        while (!isStopRequested() && opModeIsActive()) {

            run();

            telemetry.update();


//            Actions.runBlocking(
//                    new SequentialAction(
//                            deliverMoveAction,
//                            elevator.deliverToTopBasket()));

//            Actions.runBlocking(
//                    new ParallelAction(
//                            firstPickupMoveAction,
//                          //  arm.goPickupSpecimen()));
//
//            Actions.runBlocking(
//                    new SequentialAction(
//                            deliverMoveAction,
//                            elevator.deliverToTopBasket()));
//
//            Actions.runBlocking(
//                    new ParallelAction(
//                            secondPickupMoveAction,
//                         //   arm.goPickupSpecimen()));
//
//            Actions.runBlocking(
//                    new SequentialAction(
//                            deliverMoveAction,
//                            elevator.deliverToTopBasket()));
//
//            Actions.runBlocking(
//                    new ParallelAction(
//                            thirdPickupMoveAction,
//                          //  arm.goPickupSpecimen()));
//
//            Actions.runBlocking(
//                    new SequentialAction(
//                            deliverMoveAction,
//                            elevator.deliverToTopBasket()));


            new SleepAction(2);


        }
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
            telemetry.addData("    Blue All Basket   ", "(X / ▢)");

            telemetry.addData("    Red All Basket    ", "(A / O)");

            if (gamepad1.a) {
                firstSampleDeliverMoveAction = drive.actionBuilder(FieldConstantsRed.basketSideStartPose)
                        .strafeTo(FieldConstantsRed.basketDeliverPose.position)
                        .build();//move to place first specimen

                secondSamplePickupMoveAction = drive.actionBuilder(FieldConstantsRed.basketDeliverPose)
                        .strafeToLinearHeading(FieldConstantsRed.innerYellowPickupPose.position, FieldConstantsRed.innerYellowPickupPose.heading)
                        .build();

                secondSampleDeliverMoveAction = drive.actionBuilder(FieldConstantsRed.innerYellowPickupPose)
                        .strafeToLinearHeading(FieldConstantsRed.basketDeliverPose.position, FieldConstantsRed.basketDeliverPose.heading)
                        .build();//move to place first specimen

                thirdSamplePickupMoveAction = drive.actionBuilder(FieldConstantsRed.basketDeliverPose)
                        .strafeToLinearHeading(FieldConstantsRed.midYellowPickupPose.position, FieldConstantsRed.midYellowPickupPose.heading)
                        .build();

                thirdSampleDeliverMoveAction = drive.actionBuilder(FieldConstantsRed.midYellowPickupPose)
                        .strafeToLinearHeading(FieldConstantsRed.basketDeliverPose.position, FieldConstantsRed.basketDeliverPose.heading)
                        .build();//move to place first specimen

                fourthSamplePickupMoveAction = drive.actionBuilder(FieldConstantsRed.basketDeliverPose)
                        .strafeToLinearHeading(FieldConstantsRed.outerYellowApproachPose.position, FieldConstantsRed.outerYellowApproachPose.heading)
                        .waitSeconds(.1)
                        .lineToX(FieldConstantsRed.outerYellowPickupPose.position.x)

                        .build();

                fourthSampleDeliverMoveAction = drive.actionBuilder(FieldConstantsRed.outerYellowPickupPose)
                        .lineToX(FieldConstantsRed.outerYellowApproachPose.position.x)
                        .strafeToLinearHeading(FieldConstantsRed.basketDeliverPose.position, FieldConstantsRed.basketDeliverPose.heading)
                        .build();//move to place first specimen


                deliverFourSamples = new SequentialAction(
                        firstSampleDeliverMoveAction,
                        dropSampleAction,
                        new ParallelAction(
                                secondSamplePickupMoveAction,
                                pickupSampleAction),
                        secondSampleDeliverMoveAction,
                        dropSampleAction,
                        new ParallelAction(
                                thirdSamplePickupMoveAction,
                                pickupSampleAction),
                        thirdSampleDeliverMoveAction,
                        dropSampleAction,
                        new ParallelAction(
                                fourthSamplePickupMoveAction,
                                pickupSampleAction),
                        fourthSampleDeliverMoveAction,
                        dropSampleAction


                );
                break;
            }
            if (gamepad1.x) {
                firstSampleDeliverMoveAction = drive.actionBuilder(FieldConstantsBlue.basketSideStartPose)
                        .strafeTo(FieldConstantsBlue.basketDeliverPose.position)
                        .build();//move to place first specimen

                secondSamplePickupMoveAction = drive.actionBuilder(FieldConstantsBlue.basketDeliverPose)
                        .strafeToLinearHeading(FieldConstantsBlue.innerYellowPickupPose.position, FieldConstantsBlue.innerYellowPickupPose.heading)
                        .build();

                secondSampleDeliverMoveAction = drive.actionBuilder(FieldConstantsBlue.innerYellowPickupPose)
                        .strafeToLinearHeading(FieldConstantsBlue.basketDeliverPose.position, FieldConstantsBlue.basketDeliverPose.heading)
                        .build();//move to place first specimen

                thirdSamplePickupMoveAction = drive.actionBuilder(FieldConstantsBlue.basketDeliverPose)
                        .strafeToLinearHeading(FieldConstantsBlue.midYellowPickupPose.position, FieldConstantsBlue.midYellowPickupPose.heading)
                        .build();

                thirdSampleDeliverMoveAction = drive.actionBuilder(FieldConstantsBlue.midYellowPickupPose)
                        .strafeToLinearHeading(FieldConstantsBlue.basketDeliverPose.position, FieldConstantsBlue.basketDeliverPose.heading)
                        .build();//move to place first specimen

                fourthSamplePickupMoveAction = drive.actionBuilder(FieldConstantsBlue.basketDeliverPose)
                        .strafeToLinearHeading(FieldConstantsBlue.outerYellowApproachPose.position, FieldConstantsBlue.outerYellowApproachPose.heading)
                        .waitSeconds(.1)
                        .lineToX(FieldConstantsBlue.outerYellowPickupPose.position.x)

                        .build();

                fourthSampleDeliverMoveAction = drive.actionBuilder(FieldConstantsBlue.outerYellowPickupPose)
                        .lineToX(FieldConstantsBlue.outerYellowApproachPose.position.x)
                        .strafeToLinearHeading(FieldConstantsBlue.basketDeliverPose.position, FieldConstantsBlue.basketDeliverPose.heading)
                        .build();//move to place first specimen


                deliverFourSamples = new SequentialAction(
                        new SleepAction(3),
                        firstSampleDeliverMoveAction,
                        dropSampleAction,
                        new ParallelAction(
                                secondSamplePickupMoveAction,
                                pickupSampleAction),
                        secondSampleDeliverMoveAction,
                        dropSampleAction,
                        new ParallelAction(
                                thirdSamplePickupMoveAction,
                                pickupSampleAction),
                        thirdSampleDeliverMoveAction,
                        dropSampleAction,
                        new ParallelAction(
                                fourthSamplePickupMoveAction,
                                pickupSampleAction),
                        fourthSampleDeliverMoveAction,
                        dropSampleAction


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
