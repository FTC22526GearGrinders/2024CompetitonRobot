package org.firstinspires.ftc.teamcode.subsystems;

import android.graphics.Color;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.utils.PoseStorage;


@Config
public class RotateArmSubsystem extends SubsystemBase {

    public static double leftIntakeTiltClearAngle = 0.46;
    public static double leftIntakeTiltDownAngle = 1;
    public static double leftIntakeBucketClearAngle = 0.25;
    public static double rightIntakeTiltClearAngle = .46;
    public static double rightIntakeTiltDownAngle = 1;
    public static double rightIntakeBucketClearAngle = 0.25;
    public static double touchSubmersibleAngle = .3;
    public static double intakeServoInPower = 1;
    public static double intakeServoOutPower = -1;
    public CRServo leftIntakeServo;
    public CRServo rightIntakeServo;
    public RevColorSensorV3 intakeSensor;
    public Servo leftTiltServo;
    public Servo rightTiltServo;
    public int showSelect = 0;
    public boolean intaking;
    public boolean reversing;
    public boolean colorDetected;
    public boolean sampleInIntake;
    private Telemetry telemetry;
    private boolean tiltPositionClear;
    private boolean redSeen;
    private boolean blueSeen;
    private boolean yellowSeen;

    public RotateArmSubsystem(CommandOpMode opMode) {

        leftIntakeServo = opMode.hardwareMap.get(CRServo.class, "leftInServo");
        leftIntakeServo.setDirection(DcMotorSimple.Direction.REVERSE);
        rightIntakeServo = opMode.hardwareMap.get(CRServo.class, "rightInServo");
        intakeSensor = opMode.hardwareMap.get(RevColorSensorV3.class, "intakeSensor");

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
        if (showSelect == Constants.ShowTelemtryConstants.showRotateArm2)
            showTelemetryColors();
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

    public Action runLeftIntake() {
        return new InstantAction(() -> leftIntakeServo.setPower(intakeServoInPower));
    }

    public Action runRightIntake() {
        return new InstantAction(() -> rightIntakeServo.setPower(intakeServoInPower));
    }

    public Action runIntakeServos() {
        return
                new SequentialAction(
                        runLeftIntake(),
                        runRightIntake(),
                        new InstantAction(() -> intaking = true));
    }

    public Action runUntilSampleOrTimeout(double timeout_secs) {
        return new Action() {
            long startTime_ms;
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    leftIntakeServo.setPower(intakeServoInPower);
                    rightIntakeServo.setPower(intakeServoInPower);
                    startTime_ms = System.currentTimeMillis();
                    initialized = true;
                }
                packet.put("waiting", sampleAtIntake());
                sampleInIntake = sampleAtIntake();
                return !sampleAtIntake() && System.currentTimeMillis() < (startTime_ms + timeout_secs * 1000);
            }
        };
    }

    public Action rejectSampleAction() {
        return new SequentialAction(
                reverseIntakeServosTimed(3));
    }

    public Action reverseIntakeServosTimed(double secs) {
        return
                new SequentialAction(
                        reverseIntakeServos(),
                        new SleepAction(secs),
                        stopIntakeServos(),
                        new InstantAction(() -> reversing = false));
    }

    public Action reverseIntakeServos() {
        return new ParallelAction(

                reverseLeftServo(), reverseRightServo());

    }

    public Action reverseLeftServo() {
        return new InstantAction(() -> leftIntakeServo.setPower(intakeServoOutPower));
    }

    public Action reverseRightServo() {
        return new InstantAction(() -> rightIntakeServo.setPower(intakeServoOutPower));
    }

    public Action stopLeftServo() {
        return new InstantAction(() -> leftIntakeServo.setPower(0));
    }

    public Action stopRightServo() {
        return new InstantAction(() -> rightIntakeServo.setPower(0));
    }

    public Action stopIntakeServos() {
        return
                new ParallelAction(
                        stopLeftServo(),
                        stopRightServo());
    }

    public boolean wrongAllianceSampleSeen() {
        return sampleAtIntake() && (PoseStorage.currentTeam == PoseStorage.Team.BLUE && getRedSeen() || PoseStorage.currentTeam == PoseStorage.Team.RED && getBlueSeen());
    }

    public Action colorDetectAction() {
        return new Action() {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    clearColors();
                    initialized = true;
                }
                if (sampleAtIntake()) colorDetect();

                packet.put("checking", sampleAtIntake());
                return colorDetected || !sampleAtIntake();
            }
        };
    }


    public Action clearColors() {
        return new SequentialAction(
                new InstantAction(this::resetRedSeen),
                new InstantAction(this::resetBlueSeen),
                new InstantAction(this::resetYellowSeen));
    }

    public void colorDetect() {
        float hsvValues[] = {0F, 0F, 0F};
        // values is a reference to the hsvValues array.
        final float values[] = hsvValues;

        double[] redLowerHue = {330, 0};
        double[] redUpperHue = {0, 40};
        double[] blueHue = {220, 260};
        double[] yellowHue = {45, 100};

        Color.RGBToHSV(intakeSensor.red() * 8, intakeSensor.green() * 8, intakeSensor.blue() * 8, hsvValues);

        if (hsvValues[0] > redLowerHue[0] && hsvValues[0] < redLowerHue[1]
                || hsvValues[0] > redUpperHue[0] && hsvValues[0] < redUpperHue[1])
            setRedSeen();
        if (hsvValues[0] > blueHue[0] && hsvValues[0] < blueHue[1])
            setBlueSeen();
        if (hsvValues[0] > yellowHue[0] && hsvValues[0] < yellowHue[1])
            setYellowSeen();

        colorDetected = getRedSeen() || getBlueSeen() || getYellowSeen();
    }


    public void setRedSeen() {
        redSeen = true;
    }

    public boolean getRedSeen() {
        return redSeen;
    }

    public void resetRedSeen() {
        redSeen = false;
    }

    public void setBlueSeen() {
        blueSeen = true;
    }

    public boolean getBlueSeen() {
        return blueSeen;
    }

    public void resetBlueSeen() {
        blueSeen = false;
    }

    public void setYellowSeen() {
        yellowSeen = true;
    }

    public boolean getYellowSeen() {
        return yellowSeen;
    }

    public void resetYellowSeen() {
        yellowSeen = false;
    }

    public double getSensorDistance() {
        return intakeSensor.getDistance(DistanceUnit.INCH);
    }

    public boolean sampleAtIntake() {
        return getSensorDistance() < Constants.RotateArmConstants.sampleDetectedDistance;
    }

    public boolean sampleClearOfIntake() {
        return getSensorDistance() > Constants.RotateArmConstants.sampleClearDistance;
    }

    public void showTelemetry() {
        telemetry.addData("Intaking", intaking);
        telemetry.addData("Reversing", reversing);
        telemetry.addData("Inches", intakeSensor.getDistance(DistanceUnit.INCH));
        telemetry.addData("Team", PoseStorage.currentTeam);
        telemetry.update();
    }

    public void showTelemetryColors() {
        telemetry.addData("TiltClear", tiltPositionClear);
        telemetry.addData("Light", intakeSensor.getLightDetected());
        telemetry.addData("Light", intakeSensor.getDistance(DistanceUnit.INCH));
        telemetry.addData("Red", intakeSensor.red());
        telemetry.addData("Green", intakeSensor.green());
        telemetry.addData("Blue", intakeSensor.blue());
        telemetry.addData("RedSeen", getRedSeen());
        telemetry.addData("BlueSeen", getBlueSeen());
        telemetry.addData("YellSeen", getYellowSeen());
        telemetry.addData("AtIntake", sampleAtIntake());
        telemetry.addData("ClearIntake", sampleClearOfIntake());
        telemetry.update();
    }


}
