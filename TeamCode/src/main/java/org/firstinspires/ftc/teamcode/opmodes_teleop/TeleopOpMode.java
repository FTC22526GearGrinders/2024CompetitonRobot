package org.firstinspires.ftc.teamcode.opmodes_teleop;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
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
import org.firstinspires.ftc.teamcode.subsystems.ElevatorSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ExtendArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.RotateArmSubsystem;

import java.util.ArrayList;
import java.util.List;


@TeleOp
public class TeleopOpMode extends CommandOpMode {

    private final Trigger gp2lb = new Trigger(() -> gamepad2.left_bumper);
    private final Trigger gp2rb = new Trigger(() -> gamepad2.right_bumper);
    public int showSelect;
    protected MecanumDriveSubsystem drive;
    protected ExtendArmSubsystem arm;
    protected RotateArmSubsystem rotateArm;
    protected ElevatorSubsystem elevator;
    FtcDashboard dashboard;
    TelemetryPacket packet;
    Gamepad currentGamepad1 = new Gamepad();
    Gamepad currentGamepad2 = new Gamepad();
    Gamepad previousGamepad1 = new Gamepad();
    Gamepad previousGamepad2 = new Gamepad();
    private Elevator_Arm_RotateArm_Actions eara;
    private List<Action> runningActions = new ArrayList<>();

    // @Override
    public void initialize() {

        drive = new MecanumDriveSubsystem(this, new Pose2d(0, 0, 0));

        arm = new ExtendArmSubsystem(this);

        rotateArm = new RotateArmSubsystem(this);

        elevator = new ElevatorSubsystem(this);

        eara = new Elevator_Arm_RotateArm_Actions(elevator, arm, rotateArm, this);

        packet = new TelemetryPacket();

        register(drive, arm);

        drive.setDefaultCommand(new JogDrive(this.drive, gamepad1, this));

        gp2lb.whileActiveOnce(
                new JogArm(arm, gamepad2));

        gp2rb.whileActiveOnce(
                new JogElevator(elevator, gamepad2));

        arm.setDefaultCommand(new PositionHoldArm(arm));//set in subsystem eventually since needed in auto and teleop

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
        // Store the gamepad values from the previous loop iteration in
        // previousGamepad1/2 to be used in this loop iteration.
        // This is equivalent to doing this at the end of the previous
        // loop iteration, as it will run in the same order except for
        // the first/last iteration of the loop.
        previousGamepad1.copy(currentGamepad1);
        previousGamepad2.copy(currentGamepad2);

        // Store the gamepad values from this loop iteration in
        // currentGamepad1/2 to be used for the entirety of this loop iteration.
        // This prevents the gamepad values from changing between being
        // used and stored in previousGamepad1/2.
        currentGamepad1.copy(gamepad1);
        currentGamepad2.copy(gamepad2);
    }

    public void runOpMode() throws InterruptedException {
        initialize();

        waitForStart();
        rotateArm.tiltBothClear(1).run(packet);
        while (!isStopRequested() && opModeIsActive()) {
            run();
            actionLoop();

            copyGamepads();

            arm.armTest = gamepad2.left_trigger > .75;

            elevator.elevatorTest = gamepad2.right_trigger > .75;

            if (!arm.armTest) // && !elevatorTest)
                doCoDriveButtons();

            if (!elevator.elevatorTest)
                doDriverButtons();

            if (arm.armTest)
                doArmTestCoDriverButtons();

            if (elevator.elevatorTest)
                doElevatorTestCoDriverButtons();


            //  telemetry.update();
            //    showField();

        }
        reset();
    }


    private void doDriverButtons() {


        // eara.pickupSample(5))

        //pickup specimen from wall

        //reset tilt and arm to clear and home

        //  eara.deliverSampleToBucket();

        // eara.deliverSampleToZone();

        // eara.deliverSampleToBasket();

        //deliver specimen to submersible


        // drive.setSlowMode(true);


    }

    public void doCoDriveButtons() {

        if (gamepad2.a && !previousGamepad2.a) runningActions.add(eara.getSampleAutoBasket());

        if (gamepad2.dpad_up && !previousGamepad2.dpad_up) incShowSelect();
        if (gamepad2.dpad_down && !previousGamepad2.dpad_down) decShowSelect();

    }

    void doArmTestCoDriverButtons() {

        if (gamepad2.a && !previousGamepad2.a)
            runningActions.add(rotateArm.runIntakeServos());

        if (gamepad2.b && !previousGamepad2.b)
            runningActions.add(rotateArm.stopIntakeServos());

        if (gamepad2.x && !previousGamepad2.x)
            runningActions.add(rotateArm.reverseIntakeServosTimed(3));

        if (gamepad2.y && !previousGamepad2.y)
            runningActions.add(new InstantAction(() -> arm.resetEncoders()));


        if (gamepad2.dpad_up && !previousGamepad2.dpad_up && arm.getLeftPositionInches() > Constants.RotateArmConstants.armDistanceOKTilt)
            runningActions.add(rotateArm.tiltBothClear(1));

        if (gamepad2.dpad_down && !previousGamepad2.dpad_down && arm.getLeftPositionInches() > Constants.RotateArmConstants.armDistanceOKTilt)
            runningActions.add(rotateArm.tiltBothDown());

        if (gamepad2.dpad_left && !previousGamepad2.dpad_left)
            runningActions.add(arm.armToBucketAction());

        if (gamepad2.dpad_right && !previousGamepad2.dpad_right)
            runningActions.add(arm.armToPickupAction());

        if (gamepad2.right_bumper && !previousGamepad2.right_bumper)
            runningActions.add(eara.deliverSpecimenToUpperSubmersible());


    }


    void doElevatorTestCoDriverButtons() {

        if (gamepad2.a && !previousGamepad2.a) runningActions.add(elevator.openSpecimenClaw());
        if (gamepad2.b && !previousGamepad2.b) runningActions.add(elevator.closeSpecimenClaw());
        if (gamepad2.x && !previousGamepad2.x) runningActions.add(elevator.tipBucket());
        if (gamepad2.y && !previousGamepad2.y) runningActions.add(elevator.levelBucket());

        if (gamepad2.dpad_up && !previousGamepad2.dpad_up)
            runningActions.add(elevator.elevatorToAboveUpperSubmersible());
        if (gamepad2.dpad_down && !previousGamepad2.dpad_down)
            runningActions.add(elevator.elevatorToUpperSubmersible());
        if (gamepad2.dpad_left && !previousGamepad2.dpad_left)
            runningActions.add(elevator.elevatorToHome());
        if (gamepad2.dpad_right && !previousGamepad2.dpad_right)
            runningActions.add(elevator.elevatorToUpperBasket());


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
        if (showSelect > Constants.MiscConstants.maxShowSelectCount)
            showSelect = Constants.MiscConstants.minShowSelectCount;
        updateSubs();
    }

    void decShowSelect() {
        showSelect--;
        if (showSelect < Constants.MiscConstants.minShowSelectCount)
            showSelect = Constants.MiscConstants.maxShowSelectCount;
        updateSubs();
    }

    void updateSubs() {
        drive.showSelect = showSelect;
        arm.showSelect = showSelect;
        elevator.showSelect = showSelect;
        rotateArm.showSelect = showSelect;
        // limelight.showSelect = showSelect;
    }

}




