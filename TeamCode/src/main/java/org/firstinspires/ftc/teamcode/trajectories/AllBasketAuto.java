package org.firstinspires.ftc.teamcode.trajectories;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Trajectory;
import com.arcrobotics.ftclib.trajectory.constraint.TrajectoryConstraint;

import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;

public class AllBasketAuto {

    //
    Trajectory t1;
    TrajectoryConstraint constraint;


    private final MecanumDriveSubsystem drive;
    private final Pose2d startPose = new Pose2d(0, 0, 0);

    public AllBasketAuto(MecanumDriveSubsystem drive) {
        this.drive = drive;

    }


}
