package org.firstinspires.ftc.teamcode.commandsandactions.drive;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.hardware.limelightvision.LLResult;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.subsystems.LimelightSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;


public class TrackAprilTags extends CommandBase {

    private final MecanumDriveSubsystem drive;

    private final LimelightSubsystem limelight;
    private final CommandOpMode myOpmode;
    private YawPitchRollAngles orientation;

    public TrackAprilTags(MecanumDriveSubsystem drive, LimelightSubsystem limelight, CommandOpMode opMode) {
        this.drive = drive;
        this.limelight = limelight;
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

        limelight.setPipeline(limelight.aprilTagPipeline);


        FtcDashboard dashboard = FtcDashboard.getInstance();
        myOpmode.telemetry = new MultipleTelemetry(myOpmode.telemetry, dashboard.getTelemetry());
    }

    @Override
    public void execute() {

        orientation = drive.lazyImu.get().getRobotYawPitchRollAngles();


        // limelight.updateRobotOrientation(orientation.getYaw(AngleUnit.DEGREES));
        LLResult result = limelight.getLatestResult();
        if (result != null) {
            if (result.isValid()) {
                Pose3D botpose = result.getBotpose_MT2();
                Pose2d bp2 = limelight.convertPose3dTo2d(botpose);
            }
        }
    }

    private void showTwlemetry() {
        myOpmode.telemetry.addData("LLISValid", limelight.getLatestResult().isValid());
        myOpmode.telemetry.addData("TX", round2dp(limelight.getTX(), 2));
        myOpmode.telemetry.addData("TY", round2dp(limelight.getTY(), 2));
        myOpmode.telemetry.addData("Area", round2dp(limelight.getTA(), 2));
        myOpmode.telemetry.addData("Yaw (Z)", "%.2f Deg. (Heading)", orientation.getYaw(AngleUnit.DEGREES));
        myOpmode.telemetry.update();
    }


    @Override
    public void end(boolean interrupted) {
        // drive.show = true;

    }

    @Override
    public boolean isFinished() {
        return false;
    }


}