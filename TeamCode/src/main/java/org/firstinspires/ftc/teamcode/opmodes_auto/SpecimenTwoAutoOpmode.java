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


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Drawing;
import org.firstinspires.ftc.teamcode.FieldConstantsSelect;
import org.firstinspires.ftc.teamcode.commands_actions.arm.PositionHoldArm;
import org.firstinspires.ftc.teamcode.commands_actions.combined.Elevator_Arm_RotateArm_Actions;
import org.firstinspires.ftc.teamcode.commands_actions.elevator.PositionHoldElevator;
import org.firstinspires.ftc.teamcode.subsystems.ElevatorSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ExtendArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.RotateArmSubsystem;
import org.firstinspires.ftc.teamcode.utils.PoseStorage;


@Autonomous(name = "Two Specimens", group = "Auto")
//@Disabled
public class SpecimenTwoAutoOpmode extends CommandOpMode {

    public static String TEAM_NAME = "Gear Grinders"; // Enter team Name
    public static int TEAM_NUMBER = 22526; //Enter team Number


    FieldConstantsSelect fcs;

    Action firstSpecimenPreDeliverMove;
    Action firstSpecimenDeliverMove;

    Action secondSpecimenPrePickupMove;
    Action secondSpecimenPickupMove;

    Action secondSpecimenPreDeliverMove;
    Action secondSpecimenDeliverMove;


    Action park;

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
    private SequentialAction autoSequence;
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
        finalAccel = new ProfileAccelConstraint(-15, 15);
        finalVel = new TranslationalVelConstraint(15);

        register(drive, arm, elevator, rotate);

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


        firstSpecimenPreDeliverMove = drive.actionBuilder(fcs.specimenSideStartPose)
                .strafeToLinearHeading(fcs.specimenDeliverApproachPose1.position, fcs.specimenDropAngle)
                .build();

        firstSpecimenDeliverMove = drive.actionBuilder(fcs.specimenDeliverApproachPose1)
                .strafeToLinearHeading(fcs.specimenDeliverPose1.position, fcs.specimenDropAngle,
                        finalVel, finalAccel).build();


        secondSpecimenPrePickupMove = drive.actionBuilder(fcs.specimenDeliverPose1)
                .splineToLinearHeading(fcs.specimenPickupApproachPose, fcs.specimenDropAngle).build();

        secondSpecimenPickupMove = drive.actionBuilder(fcs.specimenPickupApproachPose)
                .strafeToLinearHeading(fcs.specimenPickupPose.position, fcs.specimenPickupAngle,
                        finalVel, finalAccel).build();


        secondSpecimenPreDeliverMove = drive.actionBuilder(fcs.specimenPickupPose)
                .strafeToLinearHeading(fcs.specimenDeliverApproachPose2.position, fcs.specimenDropAngle).build();

        secondSpecimenDeliverMove = drive.actionBuilder(fcs.specimenDeliverApproachPose2)
                .strafeToLinearHeading(fcs.specimenDeliverPose2.position, fcs.specimenDropAngle,
                        finalVel, finalAccel).build();

        park = drive.actionBuilder(fcs.specimenDeliverPose2)
                .strafeToLinearHeading(fcs.specimenParkPose.position, fcs.specimenDropAngle).build();
    }

    private void buildSequence() {

        autoSequence = new SequentialAction(

                new ParallelAction(
                        firstSpecimenPreDeliverMove,
                        elevator.elevatorToAboveUpperSubmersible()),

                firstSpecimenDeliverMove,

                elevator.deliverSpecimenToNearestChamber(),


                new ParallelAction(
                        secondSpecimenPrePickupMove,
                        elevator.elevatorToHome()),

                secondSpecimenPickupMove,

                elevator.grabSpecimenAndClearWall(),


                new ParallelAction(
                        secondSpecimenPreDeliverMove,
                        elevator.elevatorToAboveUpperSubmersible()),

                secondSpecimenDeliverMove,

                elevator.deliverSpecimenToNearestChamber(),

                new ParallelAction(
                        elevator.elevatorToHome(),
                        park));
    }

    @Override
    public void runOpMode() throws InterruptedException {


        initialize();

        selectStartingPosition();

        createMotionActions(red);

        buildSequence();

        waitForStart();


        elevator.closeSpecimenClaw().run(packet);

        elevator.showSelect = Constants.ShowTelemetryConstants.showElevatorCommon;

        while (!isStopRequested() && opModeIsActive()) {

            run();

            autoSequence.run(packet);

            showField();


            if (gamepad2.start) incShowSelect();
            if (gamepad2.back) decShowSelect();


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

    void showField() {
        drive.updatePoseEstimate();
        TelemetryPacket packet = new TelemetryPacket();
        packet.fieldOverlay().setStroke("#3F51B5");
        Drawing.drawRobot(packet.fieldOverlay(), drive.pose);
        FtcDashboard.getInstance().sendTelemetryPacket(packet);
    }

    void incShowSelect() {
        showSelect++;
        if (showSelect > Constants.ShowTelemetryConstants.maxShowSelectCount)
            showSelect = Constants.ShowTelemetryConstants.minShowSelectCount;
        updateSubs();
    }

    void decShowSelect() {
        showSelect--;
        if (showSelect < Constants.ShowTelemetryConstants.minShowSelectCount)
            showSelect = Constants.ShowTelemetryConstants.maxShowSelectCount;
        updateSubs();
    }

    void updateSubs() {
        telemetry.clearAll();
        //   dashboard.clearTelemetry();
        drive.showSelect = showSelect;
        arm.showSelect = showSelect;
        elevator.showSelect = showSelect;
        //  rotateArm.showSelect = showSelect;
        // limelight.showSelect = showSelect;
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

