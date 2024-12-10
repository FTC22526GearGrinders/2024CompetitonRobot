package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Constants;


@Config
public class RotateArmSubsystem extends SubsystemBase {
    public double intakeTiltHomeAngle = .1;
    public double intakeTiltClearAngle = 0.3;
    public double intakeTiltVerticalAngle = .5;
    public double touchSubmersibleAngle = .6;
    public double intakeTiltClearSubmersibleAngle = .85;
    public double intakeTiltAboveSampleAngle = .9;
    public double intakeTiltPickupAngle = .96;
    public double intakeClawOpenAngle = 0;
    public double intakeClawClosedAngle = 1;
    private double intakeClawTime = .5;
    public double currentTilt;
    public Servo intakeClawServo;

    public Servo leftTiltServo;
    public Servo rightTiltServo;

    public RevColorSensorV3 intakeSensor;
    public int showSelect = 0;

    private Telemetry telemetry;
    private boolean tiltPositionClear;


    public RotateArmSubsystem(CommandOpMode opMode) {

        intakeClawServo = opMode.hardwareMap.get(Servo.class, "intakeClaw");
        intakeClawServo.setDirection(Servo.Direction.FORWARD);

        leftTiltServo = opMode.hardwareMap.get(Servo.class, "leftTiltServo");
        rightTiltServo = opMode.hardwareMap.get(Servo.class, "rightTiltServo");

        leftTiltServo.setDirection(Servo.Direction.FORWARD);
        rightTiltServo.setDirection(Servo.Direction.REVERSE);

        intakeSensor = opMode.hardwareMap.get(RevColorSensorV3.class, "intakeSensor");


        FtcDashboard dashboard1 = FtcDashboard.getInstance();

        telemetry = new MultipleTelemetry(opMode.telemetry, dashboard1.getTelemetry());


        FtcDashboard dashboard = FtcDashboard.getInstance();

        telemetry = new MultipleTelemetry(opMode.telemetry, dashboard.getTelemetry());

    }

    @Override
    public void periodic() {
        if (showSelect == Constants.ShowTelemetryConstants.showRotateArm1)
            showTelemetry();
        if (showSelect == Constants.ShowTelemetryConstants.showColors)
            showTelemetryColors();

    }

    public Action openIntakeClaw() {
        return
                new SequentialAction(
                        new InstantAction(() -> intakeClawServo.setPosition(intakeClawOpenAngle)),
                        new SleepAction(intakeClawTime));
    }

    public Action closeIntakeClaw() {
        return
                new SequentialAction(
                        new InstantAction(() -> intakeClawServo.setPosition(intakeClawClosedAngle)),
                        new SleepAction(intakeClawTime));
    }

    public Action setTiltAngle(double angle) {
        return new ParallelAction(
                new InstantAction(() -> leftTiltServo.setPosition(angle)),
                new InstantAction(() -> rightTiltServo.setPosition(angle)),
                new InstantAction(() -> currentTilt = angle));
    }


    public Action tiltToPickup() {
        return setTiltAngle(intakeTiltPickupAngle);
    }

    public Action tiltBothAboveSamples(double timeout_secs) {
        return setTiltAngle(intakeTiltAboveSampleAngle);

    }

    public Action tiltBothClear(double timeout) {
        return setTiltAngle(intakeTiltClearAngle);
    }

    public Action tiltBothVertical(double timeout) {
        return setTiltAngle(intakeTiltVerticalAngle);
    }

    public Action tiltBothHome(double timeout) {
        return setTiltAngle(intakeTiltHomeAngle);
    }


    public Action tiltToTouchSubmersibleAction() {
        return setTiltAngle(touchSubmersibleAngle);
    }


    public void setTiltPositionClear() {
        tiltPositionClear = true;
    }

    public void setTiltPositionDown() {
        tiltPositionClear = false;
    }


    private void showTelemetry() {
        telemetry.addData("TiltAngle", currentTilt);
        telemetry.update();
    }

    public void showTelemetryColors() {
        telemetry.addData("Light", intakeSensor.getLightDetected());
        telemetry.addData("Light", intakeSensor.getDistance(DistanceUnit.INCH));
        telemetry.addData("Red", intakeSensor.red());
        telemetry.addData("Green", intakeSensor.green());
        telemetry.addData("Blue", intakeSensor.blue());
        telemetry.update();
    }

    public Action tiltToPickupThenCloseIntakeClaw(double timeout_secs) {
        return new SequentialAction(
                tiltToPickup(),
                new SleepAction(timeout_secs),
                closeIntakeClaw());
    }

    public Action pickupSample() {
        return new SequentialAction(
                tiltToPickup(),
                new SleepAction(1),
                closeIntakeClaw(),
                tiltBothAboveSamples(1));
    }

}









