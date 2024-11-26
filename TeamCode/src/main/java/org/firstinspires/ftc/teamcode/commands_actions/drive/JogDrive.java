package org.firstinspires.ftc.teamcode.commands_actions.drive;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.utils.PoseStorage;


public class JogDrive extends CommandBase {
    private final MecanumDriveSubsystem drive;

    private final Gamepad gamepad;
    private final double speedMultiplier = .25;

    CommandOpMode myOpmode;



    public JogDrive(MecanumDriveSubsystem drive, Gamepad gamepad, CommandOpMode opmode) {
        this.drive = drive;
        this.gamepad = gamepad;
        myOpmode = opmode;

        addRequirements(this.drive);

    }


    @Override
    public void initialize() {
    }


    @Override
    public void execute() {

        if (!drive.fieldCentric) {

            double y = -this.gamepad.left_stick_y;
            double x = this.gamepad.left_stick_x;
            double rx = this.gamepad.right_stick_x;


            if (drive.isSlowMode()) {
                y *= speedMultiplier;
                x *= speedMultiplier;
                rx *= speedMultiplier;
            }

            drive.jog(y, x, rx);

        }


        if (drive.fieldCentric) {
            /* Invert stick Y axis */
            double forward = -this.gamepad.left_stick_y;
            double strafe = this.gamepad.left_stick_x;
            double rcw = this.gamepad.right_stick_x;
            if (drive.isSlowMode()) {
                strafe *= speedMultiplier;
                ;
                forward *= speedMultiplier;
                rcw *= speedMultiplier;
            }
            /* Adjust Joystick X/Y inputs by navX MXP yaw angle */

            double gyro_radians = drive.startRadians - drive.pose.heading.toDouble();


            double temp = strafe * Math.sin(gyro_radians) + forward * (float) Math.cos(gyro_radians);

            strafe = strafe * Math.cos(gyro_radians) - forward * Math.sin(gyro_radians);

            forward = temp;


            if (PoseStorage.currentTeam == PoseStorage.Team.RED) {
                forward = -this.gamepad.left_stick_x;
                strafe = this.gamepad.left_stick_x; /* Invert stick Y axis */
                rcw = this.gamepad.left_stick_x;
            }




            /* Adjust Joystick X/Y inputs by navX MXP yaw angle */

            drive.jog(forward, strafe, rcw);
        }

    }

    @Override
    public void end(boolean interrupted) {
        drive.jog(0, 0, 0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }


}
