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
    public static double intakeTiltHomeAngle = .1;
    public static double intakeTiltClearAngle = 0.3;
    public static double intakeTiltVerticalAngle = .5;
    public static double touchSubmersibleAngle = .6;
    public static double intakeTiltClearSubmersibleAngle = .85;
    public static double intakeTiltAboveSampleAngle = .9;
    public static double intakeTiltPickupAngle = .96;


    public static double intakeClawOpenAngle = 0;
    public static double intakeClawClosedAngle = 1;

    private static double intakeClawTime = .5;


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

    public Action tiltBothAboveFloor() {
        return
                new SequentialAction(
                        new ParallelAction(
                                new InstantAction(() -> leftTiltServo.setPosition(intakeTiltClearSubmersibleAngle)),
                                new InstantAction(() -> rightTiltServo.setPosition(intakeTiltClearSubmersibleAngle))),
                        new SleepAction(1),
                        new InstantAction(this::setTiltPositionDown));
    }


    public Action tiltToPickup() {
        return
                new SequentialAction(
                        new ParallelAction(
                                new InstantAction(() -> leftTiltServo.setPosition(intakeTiltPickupAngle)),
                                new InstantAction(() -> rightTiltServo.setPosition(intakeTiltPickupAngle))),
                        new InstantAction(this::setTiltPositionDown));
    }

    public Action tiltBothAboveSamples(double timeout_secs) {
        return
                new SequentialAction(
                        new ParallelAction(
                                new InstantAction(() -> leftTiltServo.setPosition(intakeTiltPickupAngle)),
                                new InstantAction(() -> rightTiltServo.setPosition(intakeTiltPickupAngle))),
                        new SleepAction(timeout_secs),
                        new InstantAction(this::setTiltPositionDown));
    }

    public Action tiltBothClear(double timeout) {
        return
                new SequentialAction(
                        new ParallelAction(
                                new InstantAction(() -> leftTiltServo.setPosition(intakeTiltClearAngle)),
                                new InstantAction(() -> rightTiltServo.setPosition(intakeTiltClearAngle))),
                        new SleepAction(timeout),
                        new InstantAction(this::setTiltPositionClear));
    }

    public Action tiltBothVertical(double timeout) {
        return
                new SequentialAction(
                        new ParallelAction(
                                new InstantAction(() -> leftTiltServo.setPosition(intakeTiltVerticalAngle)),
                                new InstantAction(() -> rightTiltServo.setPosition(intakeTiltVerticalAngle)),
                                new SleepAction(timeout)));

    }

    public Action tiltBothHome(double timeout) {
        return
                new SequentialAction(
                        new ParallelAction(
                                new InstantAction(() -> leftTiltServo.setPosition(intakeTiltHomeAngle)),
                                new InstantAction(() -> rightTiltServo.setPosition(intakeTiltHomeAngle)),
                                new SleepAction(timeout),
                                new InstantAction(this::setTiltPositionClear)));
    }


    public Action tiltToTouchSubmersibleAction() {
        return new ParallelAction(
                new InstantAction(() -> leftTiltServo.setPosition(touchSubmersibleAngle)),
                new InstantAction(() -> rightTiltServo.setPosition(touchSubmersibleAngle)));
    }


    public void setTiltPositionClear() {
        tiltPositionClear = true;
    }

    public void setTiltPositionDown() {
        tiltPositionClear = false;
    }


    private void showTelemetry() {
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
}









