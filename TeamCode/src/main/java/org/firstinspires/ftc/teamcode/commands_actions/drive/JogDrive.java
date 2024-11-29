package org.firstinspires.ftc.teamcode.commands_actions.drive;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;


public class JogDrive extends CommandBase {
    private final MecanumDriveSubsystem drive;

    private final Gamepad gamepad;
    private final double speedMultiplier = .25;
    double denominator;
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

        double y = -this.gamepad.left_stick_y;
        double x = this.gamepad.left_stick_x * 1.1;
        double rx = this.gamepad.right_stick_x;


        if (drive.isSlowMode()) {
            y *= speedMultiplier;
            x *= speedMultiplier;
            rx *= speedMultiplier;
        }

        if (drive.fieldCentric) {

//shift the gyro to the starting heading and limit to +/- 180 degrees in radians
            double botHeading = Math.IEEEremainder(drive.startRadians - drive.getYawRads(), 2 * Math.PI);
            // Rotate the movement direction counter to the bot's rotation
            double rotX = x * Math.cos(botHeading) - y * Math.sin(botHeading);
            double rotY = x * Math.sin(botHeading) + y * Math.cos(botHeading);

            rotX = rotX * 1.1;  // Counteract imperfect strafing

            x = rotX;
            y = rotY;

        }

        denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);

        drive.leftFront.setPower((y + x + rx) / denominator);
        drive.leftBack.setPower((y - x + rx) / denominator);
        drive.rightFront.setPower((y - x - rx) / denominator);
        drive.rightBack.setPower((y + x - rx) / denominator);
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
