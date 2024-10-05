package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class IntakeRollerSubsystem extends SubsystemBase {


    private final CommandOpMode myOpMode;
    private final FtcDashboard dashboard;
    public CRServo intakeRoller;
    public boolean show = false;
    public Servo tiltServo;
    public double tiltCurrent = 0;
    ElapsedTime et;
    private Telemetry telemetry;
    private double scanTime;
    private double val;

    public IntakeRollerSubsystem(CommandOpMode opMode) {

        myOpMode = opMode;

        intakeRoller = opMode.hardwareMap.get(CRServo.class, "intakeRoller");

        //  intakeRoller.setInverted((false));

        dashboard = FtcDashboard.getInstance();

        telemetry = new MultipleTelemetry(opMode.telemetry, dashboard.getTelemetry());

        tiltServo = opMode.hardwareMap.get(Servo.class, "tiltServo");

        et = new ElapsedTime();
    }


    @Override
    public void periodic() {
        //
        showTelemetry();
        // }
    }

    public void runRoller(double speed) {
        intakeRoller.setPower(speed);
    }

    public void stopRoller() {
        intakeRoller.setPower(0);
    }


    public void setTiltServoAngle(double angle) {
        tiltCurrent = angle;
        tiltServo.setPosition(angle);
    }

    public void incTilt(double val1) {
        if (val == 0) {
            tiltCurrent = tiltCurrent + val1;
            val = 1;
        }
        if (tiltCurrent > 1) tiltCurrent = 1;
        if (tiltCurrent < 0) tiltCurrent = 0;
        tiltServo.setPosition(tiltCurrent);
        //val = 0;
    }


    public void showTelemetry() {
        telemetry.addData("Intake Roller", show);
        telemetry.addData("TiltAngle", tiltCurrent);
        telemetry.addData("Val", val);


        telemetry.update();

    }
}
