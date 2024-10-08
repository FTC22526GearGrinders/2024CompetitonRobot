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
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.FieldConstantsBlue;
import org.firstinspires.ftc.teamcode.FieldConstantsRed;
import org.firstinspires.ftc.teamcode.commands.servo_actions.IntakeTiltServo;
import org.firstinspires.ftc.teamcode.subsystems.ExtendArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;


@Autonomous(name = "Auto:Test", group = "Auto")
//@Disabled
public class AutoOpmode extends CommandOpMode {

    public static String TEAM_NAME = "Gear Grinders"; // Enter team Name
    public static int TEAM_NUMBER = 22526; //Enter team Number
    public static Pose2d startPosition;
    public Pose2d deliverMove;
    public Pose2d secondMove;
    public Pose2d thirdMove;
    public Pose2d fourthMove;
    public Pose2d fifthMove;
    public Pose2d sixthMove;
    MecanumDriveSubsystem drive;
    ExtendArmSubsystem arm;
    private IntakeTiltServo tilt;
    private Action deliverAction;
    private Action firstPickupAction;
    private Action secondPickupAction;
    private Action thirdPickupAction;
    private TelemetryPacket packet;
    private HardwareMap hardwareMap;

    @Override
    public void initialize() {
        drive = new MecanumDriveSubsystem(this, new Pose2d(0, 0, 0));
        arm = new ExtendArmSubsystem(this);
        tilt = new IntakeTiltServo(hardwareMap);
        packet = new TelemetryPacket();
    }

    @Override
    public void runOpMode() throws InterruptedException {

        initialize();

        waitForStart();

        while (!isStopRequested() && opModeIsActive()) {

            run();

            telemetry.update();

            new ParallelAction(IntakeTiltServo.tiltClear()).run(packet);

            Actions.runBlocking(deliverAction);


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
            telemetry.addData("Initializing FTC Wires (ftcwires.org) Autonomous adopted for Team:",
                    TEAM_NAME, " ", TEAM_NUMBER);
            telemetry.addData("---------------------------------------", "");
            telemetry.addData("Select Starting Position using XYAB on Logitech (or ▢ΔOX on Playstayion) on gamepad 1:", "");
            telemetry.addData("    Blue Left   ", "(X / ▢)");
            telemetry.addData("    Blue Right ", "(Y / Δ)");
            telemetry.addData("    Red Left    ", "(B / O)");
            telemetry.addData("    Red Right  ", "(A / X)");
            if (gamepad1.x) {
                startPosition = FieldConstantsBlue.basketSideStartPose;
                deliverAction = drive.actionBuilder(drive.pose)
                        .strafeToLinearHeading(FieldConstantsBlue.basketDeliverPose.position,
                                FieldConstantsBlue.basketDeliverPose.heading)
                        .build();
                firstPickupAction = drive.actionBuilder(drive.pose)
                        .strafeToLinearHeading(FieldConstantsBlue.innerYellowPickupPose.position,
                                FieldConstantsBlue.innerYellowPickupPose.heading)
                        .build();
                secondPickupAction = drive.actionBuilder(drive.pose)
                        .strafeToLinearHeading(FieldConstantsBlue.midYellowPickupPose.position,
                                FieldConstantsBlue.midYellowPickupPose.heading)
                        .build();
                thirdPickupAction = drive.actionBuilder(drive.pose)
                        .strafeToLinearHeading(FieldConstantsBlue.midYellowPickupPose.position,
                                FieldConstantsBlue.midYellowPickupPose.heading)
                        .build();
                break;
            }
            if (gamepad1.y) {
                startPosition = FieldConstantsRed.basketSideStartPose;
                deliverAction = drive.actionBuilder(drive.pose)
                        .strafeToLinearHeading(FieldConstantsRed.basketDeliverPose.position,
                                FieldConstantsRed.basketDeliverPose.heading)
                        .build();
                firstPickupAction = drive.actionBuilder(drive.pose)
                        .strafeToLinearHeading(FieldConstantsRed.innerYellowPickupPose.position,
                                FieldConstantsRed.innerYellowPickupPose.heading)
                        .build();
                secondPickupAction = drive.actionBuilder(drive.pose)
                        .strafeToLinearHeading(FieldConstantsRed.midYellowPickupPose.position,
                                FieldConstantsRed.midYellowPickupPose.heading)
                        .build();
                thirdPickupAction = drive.actionBuilder(drive.pose)
                        .strafeToLinearHeading(FieldConstantsRed.midYellowPickupPose.position,
                                FieldConstantsRed.midYellowPickupPose.heading)
                        .build();
                break;
            }
            if (gamepad1.b) {
                startPosition = FieldConstantsRed.basketSideStartPose;
                break;
            }
            if (gamepad1.a) {
                startPosition = FieldConstantsRed.basketSideStartPose;
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
