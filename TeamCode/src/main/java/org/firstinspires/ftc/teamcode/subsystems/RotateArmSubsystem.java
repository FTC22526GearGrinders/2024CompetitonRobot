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
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants;


@Config
public class RotateArmSubsystem extends SubsystemBase {

    public static double leftIntakeTiltClearAngle = 0.46;
    public static double leftIntakeTiltDownAngle = 1;
    public static double leftIntakeBucketClearAngle = 0.25;
    public static double rightIntakeTiltClearAngle = .46;
    public static double rightIntakeTiltDownAngle = 1;
    public static double rightIntakeBucketClearAngle = 0.25;
    public static double touchSubmersibleAngle = .3;

    public static double intakeClawOpenAngle = 0;
    public static double intakeClawClosedAngle = 1;


    public Servo intakeClawServo;

    public Servo leftTiltServo;
    public Servo rightTiltServo;
    public int showSelect = 0;

    private Telemetry telemetry;
    private boolean tiltPositionClear;


    public RotateArmSubsystem(CommandOpMode opMode) {

        intakeClawServo = opMode.hardwareMap.get(Servo.class, "intakeClaw");
        intakeClawServo.setDirection(Servo.Direction.FORWARD);


        leftTiltServo = opMode.hardwareMap.get(Servo.class, "leftTiltServo");//.4
        rightTiltServo = opMode.hardwareMap.get(Servo.class, "rightTiltServo");

        leftTiltServo.setDirection(Servo.Direction.FORWARD);
        rightTiltServo.setDirection(Servo.Direction.REVERSE);

        FtcDashboard dashboard1 = FtcDashboard.getInstance();

        telemetry = new MultipleTelemetry(opMode.telemetry, dashboard1.getTelemetry());


        FtcDashboard dashboard = FtcDashboard.getInstance();

        telemetry = new MultipleTelemetry(opMode.telemetry, dashboard.getTelemetry());

    }

    @Override
    public void periodic() {
        if (showSelect == Constants.ShowTelemtryConstants.showRotateArm1)
            showTelemetry();
    }

    public Action openIntakeClaw() {
        return
                new InstantAction(() -> intakeClawServo.setPosition(intakeClawOpenAngle));
    }

    public Action closeIntakeClaw() {
        return
                new InstantAction(() -> intakeClawServo.setPosition(intakeClawClosedAngle));
    }

    public Action tiltBothDown() {
        return
                new SequentialAction(
                        new ParallelAction(
                                new InstantAction(() -> leftTiltServo.setPosition(leftIntakeTiltDownAngle)),
                                new InstantAction(() -> rightTiltServo.setPosition(rightIntakeTiltDownAngle))),
                        new SleepAction(1),
                        new InstantAction(this::setTiltPositionDown));
    }

    public Action tiltBothClear(double timeout) {
        return
                new SequentialAction(
                        new ParallelAction(
                                new InstantAction(() -> leftTiltServo.setPosition(leftIntakeTiltClearAngle)),
                                new InstantAction(() -> rightTiltServo.setPosition(rightIntakeTiltClearAngle))),
                        new SleepAction(timeout),
                        new InstantAction(this::setTiltPositionClear));
    }

    public Action tiltBothToBucket(double timeout) {
        return
                new SequentialAction(
                        new ParallelAction(
                                new InstantAction(() -> leftTiltServo.setPosition(leftIntakeBucketClearAngle)),
                                new InstantAction(() -> rightTiltServo.setPosition(rightIntakeBucketClearAngle))),
                        new SleepAction(timeout),
                        new InstantAction(this::setTiltPositionClear));
    }

    public Action tiltToSubmersibleAction() {
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
}









