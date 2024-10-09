package org.firstinspires.ftc.teamcode.opmodes_teleop;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.button.Button;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Drawing;
import org.firstinspires.ftc.teamcode.commands.arm.JogArm;
import org.firstinspires.ftc.teamcode.commands.drive.AlignToNote;
import org.firstinspires.ftc.teamcode.commands.drive.JogDrive;
import org.firstinspires.ftc.teamcode.commands.elevator.JogElevator;
import org.firstinspires.ftc.teamcode.subsystems.ElevatorSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ExtendArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LimelightSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;


@TeleOp
public class TeleopOpMode extends CommandOpMode {

    protected MecanumDriveSubsystem drive;
    protected LimelightSubsystem limelight;
    protected ElevatorSubsystem elevator;
    protected ExtendArmSubsystem arm;
    FtcDashboard dashboard;
    GamepadEx driver;
    GamepadEx coDriver;
    TelemetryPacket packet;

    private AlignToNote m_alignToNote;

    private Button alignbutton;
    private Button openIntakeClaw;
    private Button closeIntakeClaw;

    private Button jogArm;
    private Button jogElevator;


    private Button stepTelemetryUp;
    private Button stepTelemetryDown;

    @Override
    public void initialize() {

        driver = new GamepadEx(gamepad1);

        coDriver = new GamepadEx(gamepad2);

        drive = new MecanumDriveSubsystem(this, new Pose2d(0, 0, 0));

        limelight = new LimelightSubsystem(this);


        elevator = new ElevatorSubsystem(this);

        arm = new ExtendArmSubsystem(this);

        packet = new TelemetryPacket();


        jogArm = new GamepadButton(coDriver, GamepadKeys.Button.LEFT_BUMPER);

        jogElevator = new GamepadButton(coDriver, GamepadKeys.Button.LEFT_BUMPER);

        m_alignToNote = new AlignToNote(drive, limelight, driver, true, this);

        alignbutton = new GamepadButton(driver, GamepadKeys.Button.LEFT_BUMPER);

        openIntakeClaw = new GamepadButton(coDriver, GamepadKeys.Button.RIGHT_STICK_BUTTON);
        closeIntakeClaw = new GamepadButton(coDriver, GamepadKeys.Button.LEFT_STICK_BUTTON);


        register(drive, arm, elevator, limelight);

        drive.setDefaultCommand(new JogDrive(this.drive, driver, false, this));

        limelight.setAprilTagPipeline();


    }


    public void runOpMode() throws InterruptedException {
        initialize();
        waitForStart();
        while (!isStopRequested() && opModeIsActive()) {
            CommandScheduler.getInstance().run();
            checkTriggers();
            showField();
        }
        reset();
    }

    void checkTriggers() {

        driver.readButtons();

        if (driver.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) == 1) {
            new JogDrive(drive, driver, true, this).execute();
        }

        if (driver.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) == 1) {
            drive.resetEncoders();
        }
        alignbutton.whenHeld(m_alignToNote);


        jogArm.whenHeld(new JogArm(arm, coDriver));
        jogElevator.whenHeld(new JogElevator(elevator, coDriver));


        if (openIntakeClaw.get())
            arm.clawOpen().run(packet);
        if (closeIntakeClaw.get())
            arm.clawClose().run(packet);


//
//        positionElevatorZeroInches.whenPressed(new PositionElevator(elevator, 0));
//
//        positionElevatorTenInches.whenPressed(new PositionElevator(elevator, 10));
//
//        positionElevatorTwentyInches.whenPressed(new PositionElevator(elevator, 20));

    }


    void showField() {


        drive.updatePoseEstimate();
//
        telemetry.addData("x", drive.pose.position.x);
        telemetry.addData("y", drive.pose.position.y);
        telemetry.addData("heading (deg)", Math.toDegrees(drive.pose.heading.toDouble()));
        // telemetry.addData("BP3d",limelight.getBotPose2d().toString());

        telemetry.update();

        TelemetryPacket packet = new TelemetryPacket();
        packet.fieldOverlay().setStroke("#3F51B5");
        Drawing.drawRobot(packet.fieldOverlay(), drive.pose);
        // Drawing.drawRobot(packet.fieldOverlay(),limelight.getBotPose2d());
        FtcDashboard.getInstance().sendTelemetryPacket(packet);


    }


}


