package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class IntakeRollerSubsystem extends SubsystemBase {


    private final CommandOpMode myOpMode;
    private final FtcDashboard dashboard;
    public CRServo intakeRoller;
    public boolean show=false;
    ElapsedTime et;
    private Telemetry telemetry;
    private double scanTime;


    public IntakeRollerSubsystem(CommandOpMode opMode) {

        myOpMode = opMode;

        intakeRoller = opMode.hardwareMap.get(CRServo.class, "intakeRoller");

        //  intakeRoller.setInverted((false));

        dashboard = FtcDashboard.getInstance();

        telemetry = new MultipleTelemetry(opMode.telemetry, dashboard.getTelemetry());


        et = new ElapsedTime();
    }


    @Override
    public void periodic() {
        if (show) {
            showTelemetry();
        }
    }

    public void runRoller(double speed) {
        intakeRoller.setPower(speed);
    }

    public void stopRoller() {
        intakeRoller.setPower(0);
    }

    public void showTelemetry() {
        telemetry.addData("Intake Roller", show);
        // telemetry.addData("RollerSpeed", intakeRoller.get());


        telemetry.update();

    }
}
