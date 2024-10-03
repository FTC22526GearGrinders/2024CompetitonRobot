package org.firstinspires.ftc.teamcode.commands.drive;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.gamepad.GamepadEx;

import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.utils.ActiveMotionValues;


public class JogDrive extends CommandBase {
    private final MecanumDriveSubsystem drive;

    private final GamepadEx gamepad;

    private final boolean slow;
    CommandOpMode myOpmode;
    private double startRadians;


    public JogDrive(MecanumDriveSubsystem drive, GamepadEx gamepad, boolean slow, CommandOpMode opmode) {
        this.drive = drive;
        this.gamepad = gamepad;
        this.slow = slow;
        myOpmode = opmode;

        addRequirements(this.drive);

    }


    @Override
    public void initialize() {
        drive.tempy = 911;
        double gyro_radians = startRadians - drive.getYaw();
    }


    @Override
    public void execute() {
        drive.tempy++;
        if (!drive.fieldCentric) {

            double y = this.gamepad.getLeftY();
            double x = this.gamepad.getLeftX();
            double rx = this.gamepad.getRightX();

//            myOpmode.telemetry.addData("X", x);
//            myOpmode.telemetry.update();

            if (slow) {
                y *= .2;
                x *= .2;
                rx *= .2;
            }

            drive.jog(y, x, rx);

        }

        if (drive.fieldCentric) {
            /* Invert stick Y axis */
            double forward = -this.gamepad.getLeftY();
            double strafe = this.gamepad.getLeftX();
            double rcw = this.gamepad.getRightX();
            if (slow) {
                strafe *= .2;
                forward *= .2;
                rcw *= .2;
            }


            if (!ActiveMotionValues.getRedAlliance()) {

                forward = -this.gamepad.getLeftY();
                strafe = this.gamepad.getLeftX(); /* Invert stick Y axis */
                rcw = this.gamepad.getRightX();
            }




            /* Adjust Joystick X/Y inputs by navX MXP yaw angle */

            double gyro_radians = startRadians;//- drive.drive.getRawExternalHeading();


            double temp = strafe * Math.sin(gyro_radians) + forward * (float) Math.cos(gyro_radians);

            strafe = strafe * Math.cos(gyro_radians) - forward * Math.sin(gyro_radians);

            forward = temp;

            drive.jog(forward, strafe, rcw);
        }

    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {
        return false;
    }


}
