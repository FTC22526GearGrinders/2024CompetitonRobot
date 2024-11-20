package org.firstinspires.ftc.teamcode.opmodes_teleop;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

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

    protected MecanumDriveSubsystem drive;
    protected ExtendArmSubsystem arm;
    protected RotateArmSubsystem rotateArm;
    protected ElevatorSubsystem elevator;
    FtcDashboard dashboard;
    TelemetryPacket packet;
    private Elevator_Arm_RotateArm_Actions eara;
    private List<Action> runningActions = new ArrayList<>();

    // @Override
    public void initialize() {


        drive = new MecanumDriveSubsystem(this, new Pose2d(0, 0, 0));

        arm = new ExtendArmSubsystem(this);

        rotateArm = new RotateArmSubsystem(this);

        elevator = new ElevatorSubsystem(this);

        eara = new Elevator_Arm_RotateArm_Actions(arm, rotateArm, this);

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


    public void runOpMode() throws InterruptedException {
        initialize();
        waitForStart();
        rotateArm.tiltBothClear(1).run(packet);
        while (!isStopRequested() && opModeIsActive()) {
            run();
            actionLoop();
            doDriverButtons();

            arm.armTest = gamepad2.left_trigger > .75;
            elevator.elevatorTest = gamepad2.right_trigger < -.75;

            if (!arm.armTest) // && !elevatorTest)
                doCoDriveButtons();

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

        if (gamepad2.a) runningActions.add(eara.getSampleAutoBasket());

    }

    void doArmTestCoDriverButtons() {

        if (gamepad2.a)
            runningActions.add(rotateArm.runIntakeServos());

        if (gamepad2.b)
            runningActions.add(rotateArm.stopIntakeServos());

        if (gamepad2.x)
            runningActions.add(rotateArm.reverseIntakeServosTimed(3));

        if (gamepad2.y)
            runningActions.add(new InstantAction(() -> arm.resetEncoders()));


        if (gamepad2.dpad_up && arm.getLeftPositionInches() > Constants.RotateArmConstants.armDistanceOKTilt)
            runningActions.add(rotateArm.tiltBothClear(1));

        if (gamepad2.dpad_down && arm.getLeftPositionInches() > Constants.RotateArmConstants.armDistanceOKTilt)
            runningActions.add(rotateArm.tiltBothDown());

        if (gamepad2.dpad_left)
            runningActions.add(arm.armToBucketAction());

        if (gamepad2.dpad_right)
            runningActions.add(arm.armToPickupAction());


    }


    void doElevatorTestCoDriverButtons() {






    }


    void showField() {


        drive.updatePoseEstimate();
        TelemetryPacket packet = new TelemetryPacket();
        packet.fieldOverlay().setStroke("#3F51B5");
        Drawing.drawRobot(packet.fieldOverlay(), drive.pose);
        FtcDashboard.getInstance().sendTelemetryPacket(packet);

    }


}




