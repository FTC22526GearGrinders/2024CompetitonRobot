package org.firstinspires.ftc.teamcode.commands.drive;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.gamepad.GamepadEx;

import org.firstinspires.ftc.teamcode.subsystems.LimelightSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;


public class AlignToNote extends CommandBase {
    public static double kP = .0009;
    public static double kD = .001;
    public static double kI = .001;
    private final MecanumDriveSubsystem drive;
    private final PIDController alignpid = new PIDController(kP, kI, kD);
    private final GamepadEx gamepad;
    private final boolean yellow;
    private final LimelightSubsystem limelight;
    private final CommandOpMode myOpmode;
    boolean redAlliance;

    public AlignToNote(MecanumDriveSubsystem drive, LimelightSubsystem limelight, GamepadEx gamepad, boolean yellow, CommandOpMode opMode) {
        this.drive = drive;
        this.limelight = limelight;
        this.gamepad = gamepad;
        this.yellow = yellow;
        myOpmode = opMode;
        addRequirements(this.drive);
    }

    public static double round2dp(double number, int dp) {
        double temp = Math.pow(10, dp);
        double temp1 = Math.round(number * temp);
        return temp1 / temp;
    }

    @Override
    public void initialize() {
        if (yellow)
            limelight.setPipeline(limelight.yellowPipeline);


        if (redAlliance) limelight.setPipeline(limelight.redPipeline);
        else limelight.setPipeline(limelight.bluePipeline);

        FtcDashboard dashboard = FtcDashboard.getInstance();
        myOpmode.telemetry = new MultipleTelemetry(myOpmode.telemetry, dashboard.getTelemetry());
    }

    @Override
    public void execute() {

        double rx = this.gamepad.getRightX();

        if (limelight.getLatestResult().isValid()) {
            rx = alignpid.calculate(limelight.getTX(), 0);
        }

        double y = this.gamepad.getLeftY();
        double x = this.gamepad.getLeftX();
        drive.jog(y, x, rx);

        myOpmode.telemetry.addData("LLISValid", limelight.getLatestResult().isValid());
        myOpmode.telemetry.addData("TX", round2dp(limelight.getTX(), 2));
        myOpmode.telemetry.addData("TY", round2dp(limelight.getTY(), 2));
        myOpmode.telemetry.addData("Area", round2dp(limelight.getTA(), 2));
        myOpmode.telemetry.addData("PIError", rx);
        myOpmode.telemetry.update();

    }

    @Override
    public void end(boolean interrupted) {
        // drive.show = true;
        drive.jog(0, 0, 0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

}