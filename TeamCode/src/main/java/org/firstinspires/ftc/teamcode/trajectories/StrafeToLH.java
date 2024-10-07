package org.firstinspires.ftc.teamcode.trajectories;

import com.acmerobotics.roadrunner.Pose2d;
import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.FieldConstantsRed;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;


public class StrafeToLH extends CommandBase {
    private final MecanumDriveSubsystem drive;
    private final Pose2d endPose;

    public StrafeToLH(MecanumDriveSubsystem drive, Pose2d endPose) {
        this.drive = drive;
        this.endPose = endPose;
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {

        drive.actionBuilder(drive.pose)
                .strafeToLinearHeading(endPose.position, endPose.heading)
                .build();

    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {
        return Thread.currentThread().isInterrupted();
    }
}
