package org.firstinspires.ftc.teamcode.opmodes_teleop;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.button.Button;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Drawing;
import org.firstinspires.ftc.teamcode.commands.arm.JogArm;
import org.firstinspires.ftc.teamcode.commands.drive.AlignToNote;
import org.firstinspires.ftc.teamcode.commands.drive.JogDrive;
import org.firstinspires.ftc.teamcode.subsystems.ExtendArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeRollerSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LimelightSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;


@TeleOp
public class TeleopOpMode extends CommandOpMode {

    protected MecanumDriveSubsystem drive;
    protected LimelightSubsystem limelight;
    protected IntakeRollerSubsystem intakeRoller;
    // protected ElevatorSubsystem elevator;
    protected ExtendArmSubsystem arm;
    FtcDashboard dashboard;
    GamepadEx driver;
    GamepadEx coDriver;
    private AlignToNote m_alignToNote;

    private Button alignbutton;

    private Button jogRollerIn;
    private Button jogRollerOut;
    private Button runRollerIn;
    private Button runRollerOut;
    private Button stopRoller;

    private Button tiltServoDown;
    private Button tiltServoUp;
    private Button jogTiltDown;
    private Button jogTiltUp;

    private Button jogElevator;
    private Button positionElevatorTenInches;
    private Button positionElevatorTwentyInches;
    private Button positionElevatorZeroInches;


    private Button jogArm;
    private Button positionArmTenInches;
    private Button positionArmTwentyInches;
    private Button positionArmZeroInches;


    private Button stepTelemetryUp;
    private Button stepTelemetryDown;

    @Override
    public void initialize() {

        driver = new GamepadEx(gamepad1);

        coDriver = new GamepadEx(gamepad2);

        drive = new MecanumDriveSubsystem(this, new Pose2d(0, 0, 0));

        limelight = new LimelightSubsystem(this);

        intakeRoller = new IntakeRollerSubsystem(this);

        //  elevator = new ElevatorSubsystem(this);

        arm = new ExtendArmSubsystem(this);

        intakeRoller.setTiltServoAngle(intakeRoller.tiltCurrent);

        jogArm = new GamepadButton(coDriver, GamepadKeys.Button.LEFT_BUMPER);

//        jogElevator = new GamepadButton(coDriver, GamepadKeys.Button.LEFT_BUMPER);
//
//        positionElevatorZeroInches = new GamepadButton(coDriver, GamepadKeys.Button.A);
//
//        positionElevatorTenInches = new GamepadButton(coDriver, GamepadKeys.Button.B);
//
//        positionElevatorTwentyInches = new GamepadButton(coDriver, GamepadKeys.Button.X);

        tiltServoUp = new GamepadButton(coDriver, GamepadKeys.Button.A);
        tiltServoDown = new GamepadButton(coDriver, GamepadKeys.Button.B);
        jogTiltUp = new GamepadButton(coDriver, GamepadKeys.Button.DPAD_UP);
        jogTiltDown = new GamepadButton(coDriver, GamepadKeys.Button.DPAD_DOWN);


        m_alignToNote = new AlignToNote(drive, limelight, driver, true, this);

        alignbutton = new GamepadButton(driver, GamepadKeys.Button.LEFT_BUMPER);

        jogRollerIn = new GamepadButton(coDriver, GamepadKeys.Button.X);

        jogRollerOut = new GamepadButton(coDriver, GamepadKeys.Button.Y);

        runRollerIn = new GamepadButton(driver, GamepadKeys.Button.A);

        runRollerOut = new GamepadButton(driver, GamepadKeys.Button.B);

        stopRoller = new GamepadButton(driver, GamepadKeys.Button.X);


        register(drive, arm, limelight);

        drive.setDefaultCommand(new JogDrive(this.drive, driver, false, this));

        limelight.setAprilTagPipeline();

//        TelemetryPacket packet = new TelemetryPacket();
//        packet.fieldOverlay().setStroke("#3F51B5");
//       Drawing.drawRobot(packet.fieldOverlay(), drive.pose);
//       Drawing.drawRobot(packet.fieldOverlay(),limelight.getBotPose2d());
//       FtcDashboard.getInstance().sendTelemetryPacket(packet);


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
//        TelemetryPacket packet = new TelemetryPacket();
//        packet.fieldOverlay().setStroke("#3F51B5");
//        Drawing.drawRobot(packet.fieldOverlay(), drive.pose);
        // Drawing.drawRobot(packet.fieldOverlay(),limelight.getBotPose2d());
//        FtcDashboard.getInstance().sendTelemetryPacket(packet);
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

        jogRollerIn.whenPressed(new InstantCommand(() -> intakeRoller.runRoller(1)))
                .whenReleased(new InstantCommand(() -> intakeRoller.stopRoller()));
        jogRollerOut.whenPressed(new InstantCommand(() -> intakeRoller.runRoller(-1)))
                .whenReleased(new InstantCommand(() -> intakeRoller.stopRoller()));

        runRollerIn.whenPressed(new InstantCommand(() -> intakeRoller.runRoller(1)));
        runRollerOut.whenPressed(new InstantCommand(() -> intakeRoller.runRoller(-1)));
        stopRoller.whenPressed(new InstantCommand(() -> intakeRoller.stopRoller()));

        tiltServoDown.whenPressed(new InstantCommand(() -> intakeRoller.setTiltServoAngle(0.25)));
        tiltServoUp.whenPressed(new InstantCommand(() -> intakeRoller.setTiltServoAngle(.4)));

        jogTiltDown.whenPressed(new InstantCommand(() -> intakeRoller.incTilt(-0.005)));

        jogTiltUp.whenPressed(new InstantCommand(() -> intakeRoller.incTilt(0.005)));

        jogArm.whenHeld(new JogArm(arm, coDriver));


//        jogElevator.whenHeld(new JogElevator(elevator, coDriver));
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


