package org.firstinspires.ftc.teamcode.opmodes_teleop;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Drawing;
import org.firstinspires.ftc.teamcode.commands_actions.arm.JogArm;
import org.firstinspires.ftc.teamcode.commands_actions.arm.PositionHoldArm;
import org.firstinspires.ftc.teamcode.commands_actions.combined.Elevator_Arm_RotateArm_Actions;
import org.firstinspires.ftc.teamcode.commands_actions.drive.JogDrive;
import org.firstinspires.ftc.teamcode.commands_actions.elevator.JogElevator;
import org.firstinspires.ftc.teamcode.commands_actions.elevator.PositionHoldElevator;
import org.firstinspires.ftc.teamcode.subsystems.ElevatorSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ExtendArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.RotateArmSubsystem;

import java.util.ArrayList;
import java.util.List;


@TeleOp
public class TeleopOpMode extends CommandOpMode {

    private final Trigger gp2lsb = new Trigger(() -> gamepad2.left_stick_button);
    private final Trigger gp2rsb = new Trigger(() -> gamepad2.right_stick_button);
    public int showSelect;
    protected MecanumDriveSubsystem drive;
    protected ExtendArmSubsystem arm;
    protected RotateArmSubsystem rotateArm;
    protected ElevatorSubsystem elevator;
    TelemetryPacket packet;
    Gamepad currentGamepad1 = new Gamepad();
    Gamepad currentGamepad2 = new Gamepad();
    Gamepad previousGamepad1 = new Gamepad();
    Gamepad previousGamepad2 = new Gamepad();

    private Elevator_Arm_RotateArm_Actions eara;
    private List<Action> runningActions = new ArrayList<>();
    private int pickupStep;

    // @Override
    public void initialize() {

        drive = new MecanumDriveSubsystem(this, new Pose2d(0, 0, 0));

        arm = new ExtendArmSubsystem(this);

        rotateArm = new RotateArmSubsystem(this);

        elevator = new ElevatorSubsystem(this);

        eara = new Elevator_Arm_RotateArm_Actions(elevator, arm, rotateArm, this);

        packet = new TelemetryPacket();

        register(drive, arm, elevator, rotateArm);

        drive.setDefaultCommand(new JogDrive(this.drive, gamepad1, this));

        arm.setDefaultCommand(new PositionHoldArm(arm));//set in subsystem eventually since needed in auto and teleop

        elevator.setDefaultCommand(new PositionHoldElevator(elevator));

        arm.dropParkArm();
    }


    private void actionLoop() {
        TelemetryPacket packet = new TelemetryPacket();
        List<Action> newActions = new ArrayList<>();
        for (Action action : runningActions) {
            action.preview(packet.fieldOverlay());
            if (action.run(packet)) {
                newActions.add(action);
            }
        }
        runningActions = newActions;
        //following is in docs but crashes here
        //  if (!runningActions.isEmpty())
        //     dashboard.sendTelemetryPacket(packet);
    }

    void copyGamepads() {
        previousGamepad1.copy(currentGamepad1);
        previousGamepad2.copy(currentGamepad2);

        currentGamepad1.copy(gamepad1);
        currentGamepad2.copy(gamepad2);
    }

    public void runOpMode() throws InterruptedException {
        initialize();
        waitForStart();
        runningActions.add(rotateArm.tiltBothHome());
        runningActions.add(elevator.elevatorToHome());
        runningActions.add(elevator.levelBucket());
        while (!isStopRequested() && opModeIsActive()) {
            run();
            actionLoop();
            copyGamepads();
            showField();
            doDriverButtons();
            doCoDriverButtons();

//            if (!elevator.checkOK()) gamepad1.runRumbleEffect(RumbleDefs.fourStepRumbleEffect);
//            if (!arm.checkOK()) gamepad1.runRumbleEffect(RumbleDefs.threeStepRumbleEffect);

        }
        reset();
    }


    private void doDriverButtons() {

        /*
         *
         * Use dpad ri
         *
         *
         *
         * */

        if (currentGamepad1.dpad_right && !previousGamepad1.dpad_right)
            runningActions.add(eara.armOutTiltAboveSubmersible());

        if (currentGamepad1.dpad_down && !previousGamepad1.dpad_down)
            runningActions.add(rotateArm.toggleTiltPickupAboveSamples());

        if (currentGamepad1.left_bumper && !previousGamepad1.left_bumper)
            runningActions.add(rotateArm.toggleIntakeClaw());

        if (currentGamepad1.dpad_left && !previousGamepad1.dpad_left)
            runningActions.add(eara.armOutTiltAboveSamples());

        if (currentGamepad1.dpad_up && !previousGamepad1.dpad_up)
            runningActions.add(eara.armOutShortTiltAboveSubmersibleWithSample());

        if (currentGamepad1.left_trigger > .75 && !(previousGamepad1.left_trigger > .75))
            runningActions.add(eara.tiltAndArmMoveThenDeliverToBucket());


        if (currentGamepad1.right_bumper && !previousGamepad1.right_bumper)
            runningActions.add(elevator.grabSpecimenAndClearWall());

        if (currentGamepad1.right_trigger > .75 && !(previousGamepad1.right_trigger > .75))
            runningActions.add(elevator.deliverSpecimenToNearestChamber());

//        if (currentGamepad1.y && !previousGamepad1.y)
//            runningActions.add(

//        if (currentGamepad1.x && !previousGamepad1.x)
//            runningActions.add(

        if (currentGamepad1.a && !previousGamepad1.a)
            runningActions.add(elevator.openSpecimenClaw());

        if (currentGamepad1.b && !previousGamepad1.b)
            runningActions.add(elevator.closeSpecimenClaw());


    }

    public void doCoDriverButtons() {

        gp2rsb.whileActiveOnce(
                new JogArm(arm, gamepad2));//speed is right stick x

        gp2lsb.whileActiveContinuous(
                new JogElevator(elevator, gamepad2), false);//speed is left stick y

        if (currentGamepad2.left_bumper && !previousGamepad2.left_bumper)
            runningActions.add(elevator.elevatorToHome());

        if (currentGamepad2.left_trigger > .75 && !(previousGamepad2.left_trigger > .75))
            runningActions.add(elevator.elevatorToUpperBasket());

        if (currentGamepad2.right_bumper && !previousGamepad2.right_bumper)
            runningActions.add(elevator.elevatorToAboveUpperSubmersible());

        if (currentGamepad2.right_trigger > .75 && !(previousGamepad2.right_trigger > .75))
            runningActions.add(elevator.elevatorToAboveLowerSubmersible());

        if (currentGamepad2.y && !previousGamepad2.y)
            runningActions.add(elevator.elevatorToLowBasket());

        if (currentGamepad2.x && !previousGamepad2.x)
            runningActions.add(elevator.cycleBucket());

        if (currentGamepad2.a && !previousGamepad2.a)
            runningActions.add(elevator.deliverSpecimenToNearestChamber());

        //  if (currentGamepad2.b && !previousGamepad2.b)


        if (currentGamepad2.dpad_up && !previousGamepad1.dpad_up)
            runningActions.add(eara.testDeliver());

        if (currentGamepad2.dpad_down && !previousGamepad1.dpad_down)
            runningActions.add(eara.testPickup());


//        if (currentGamepad2.dpad_right && !previousGamepad1.dpad_right)
//
//
//        if (currentGamepad2.dpad_left && !previousGamepad1.dpad_left)
//

        if (currentGamepad2.start && !previousGamepad2.start) incShowSelect();
        if (currentGamepad2.back && !previousGamepad2.back) decShowSelect();

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
        rotateArm.showSelect = showSelect;
        // limelight.showSelect = showSelect;
    }

}




