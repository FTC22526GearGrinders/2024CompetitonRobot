package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.utils.ConditionalAction;


@Config
public class RotateArmSubsystem extends SubsystemBase {
    private final double intakeClawTime = .5;
    public double intakeTiltHomeAngle = .1;
    public double intakeTiltClearAngle = 0.3;
    public double intakeTiltVerticalAngle = .5;
    public double intakeTiltAboveSubmersibleAngleEmptyClaw = .87;
    public double intakeTiltAboveSubmersibleAngleWithSample = .85;
    public double intakeTiltAboveSampleAngle = .98;
    public double intakeTiltPickupAngle = .98;
    public double currentTilt;
    public double intakeClawOpenAngle = 0;
    public double intakeClawClosedAngle = 1;
    public Servo intakeClawServo;
    public double currentClaw;
    public Servo leftTiltServo;
    public Servo rightTiltServo;
    public RevColorSensorV3 intakeSensor;
    public int showSelect = 0;

    private Telemetry telemetry;


    public RotateArmSubsystem(CommandOpMode opMode) {

        intakeClawServo = opMode.hardwareMap.get(Servo.class, "intakeClaw");
        intakeClawServo.setDirection(Servo.Direction.FORWARD);

        leftTiltServo = opMode.hardwareMap.get(Servo.class, "leftTiltServo");
        rightTiltServo = opMode.hardwareMap.get(Servo.class, "rightTiltServo");

        leftTiltServo.setDirection(Servo.Direction.FORWARD);
        rightTiltServo.setDirection(Servo.Direction.REVERSE);

        //  intakeSensor = opMode.hardwareMap.get(RevColorSensorV3.class, "intakeSensor");


        FtcDashboard dashboard1 = FtcDashboard.getInstance();

        telemetry = new MultipleTelemetry(opMode.telemetry, dashboard1.getTelemetry());


        FtcDashboard dashboard = FtcDashboard.getInstance();

        telemetry = new MultipleTelemetry(opMode.telemetry, dashboard.getTelemetry());

        tiltBothHome();

    }

    @Override
    public void periodic() {
        if (showSelect == Constants.ShowTelemetryConstants.showRotateArm1)
            showTelemetry();
        if (showSelect == Constants.ShowTelemetryConstants.showColors)
            showTelemetryColors();

    }

    public Action openIntakeClaw() {
        return new ParallelAction(
                new InstantAction(() -> intakeClawServo.setPosition(intakeClawOpenAngle)),
                new InstantAction(() -> currentClaw = intakeClawOpenAngle));
    }

    public Action closeIntakeClaw() {
        return
                new ParallelAction(
                        new InstantAction(() -> intakeClawServo.setPosition(intakeClawClosedAngle)),
                        new InstantAction(() -> currentClaw = intakeClawClosedAngle));
    }

    public Action toggleIntakeClaw() {
        return
                new ConditionalAction(closeIntakeClaw(), openIntakeClaw(), currentClaw == intakeClawOpenAngle);
    }


    public Action setTiltAngle(double angle) {
        return
                new ParallelAction(
                        new InstantAction(() -> leftTiltServo.setPosition(angle)),
                        new InstantAction(() -> rightTiltServo.setPosition(angle)),
                        new InstantAction(() -> currentTilt = angle));
    }

    public Action tiltToPickup() {
        return setTiltAngle(intakeTiltPickupAngle);
    }

    public Action tiltAboveSamples() {
        return setTiltAngle(intakeTiltAboveSampleAngle);
    }

    public Action toggleTiltPickupAboveSamples() {
        return new ConditionalAction(tiltToPickup(), tiltAboveSamples(), currentTilt == intakeTiltAboveSampleAngle);
    }

    public Action tiltAboveSubmersible() {
        return setTiltAngle(intakeTiltAboveSubmersibleAngleEmptyClaw);
    }

    public Action tiltAboveSubmersibleWithSample() {
        return setTiltAngle(intakeTiltAboveSubmersibleAngleWithSample);
    }

    public Action tiltToBucketDeliver() {
        return setTiltAngle(intakeTiltClearAngle);
    }

    public Action tiltBothVertical() {
        return setTiltAngle(intakeTiltVerticalAngle);
    }

    public Action tiltBothHome() {
        return setTiltAngle(intakeTiltHomeAngle);
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


}









