package org.firstinspires.ftc.teamcode.subsystems;

import static com.arcrobotics.ftclib.util.MathUtils.clamp;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.wpilibcontroller.ProfiledPIDController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.trajectory.TrapezoidProfile;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants;


@Config
public class ExtendArmSubsystem extends SubsystemBase {

    public static double TARGET;

    public static double kp = 0.8;
    public static double ki = 0;
    public static double kd = 0;

    public static double parkArmDown = .25;
    public static double parkArmUp = .0;


    public static boolean TUNE = false;

    public final double OUT_POSITION_LIMIT = 17.00;
    public final double IN_POSITION_LIMIT = .5;

    /*
     * Motion is 12 inches in say 1 second
     * 1/3 time each in acc, constant speed and dec gives 1/2 distance = 6" in 1/3 sec or 18 IPs
     * accel is 0 to 18 in 1/3 = 54 ipsps
     *
     *
     * */
    private final double lrDiffMaxInches = 4;
    public double TRAJECTORY_VEL = 20;
    public double TRAJECTORY_ACCEL = 10;
    public double lastTargetInches;
    public Motor leftArmMotor;
    public Motor rightArmMotor;
    public Motor.Encoder leftArmEncoder;
    public Motor.Encoder rightArmEncoder;
    public double power;
    public ProfiledPIDController leftArmController;
    public ProfiledPIDController rightArmController;
    public TrapezoidProfile.State leftArmSetPoint = new TrapezoidProfile.State();
    public TrapezoidProfile.State rightArmSetPoint = new TrapezoidProfile.State();
    public TrapezoidProfile.Constraints constraints;
    public int holdCtr;
    public int showSelect = 0;
    public boolean shutDownArmPositioning;
    public Servo parkArmServo;
    double leftPidOut;
    double rightPidOut;
    private ElapsedTime et;
    private int inPositionCtr;
    private Telemetry telemetry;
    private double scanTime;

    public ExtendArmSubsystem(CommandOpMode opMode) {

        parkArmServo = opMode.hardwareMap.get(Servo.class, "parkArm");

        parkArmServo.setDirection(Servo.Direction.FORWARD);

        constraints = new TrapezoidProfile.Constraints(TRAJECTORY_VEL, TRAJECTORY_ACCEL);

        leftArmMotor = new Motor(opMode.hardwareMap, "leftArmMotor");
        rightArmMotor = new Motor(opMode.hardwareMap, "rightArmMotor");


        leftArmMotor.setInverted(false);
        leftArmMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        leftArmEncoder = leftArmMotor.encoder;
        leftArmEncoder.setDirection(Motor.Direction.FORWARD);
        leftArmEncoder.setDistancePerPulse(1 / Constants.ExtendArmConstants.ENCODER_COUNTS_PER_INCH);//ENCODER_COUNTS_PER_INCH);
        leftArmController = new ProfiledPIDController(kp, ki, kd, constraints);
        leftArmController.setTolerance(Constants.ExtendArmConstants.POSITION_TOLERANCE_INCHES);
        leftArmController.reset(0);


        rightArmMotor.setInverted(true);
        rightArmMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        rightArmEncoder = rightArmMotor.encoder;
        rightArmEncoder.setDirection(Motor.Direction.FORWARD);
        rightArmEncoder.setDistancePerPulse(1 / Constants.ExtendArmConstants.ENCODER_COUNTS_PER_INCH);//ENCODER_COUNTS_PER_INCH);
        rightArmController = new ProfiledPIDController(kp, ki, kd, constraints);
        rightArmController.setTolerance(Constants.ExtendArmConstants.POSITION_TOLERANCE_INCHES);
        rightArmController.reset(0);

        resetEncoders();

        FtcDashboard dashboard1 = FtcDashboard.getInstance();

        telemetry = new MultipleTelemetry(opMode.telemetry, dashboard1.getTelemetry());


        FtcDashboard dashboard = FtcDashboard.getInstance();

        telemetry = new MultipleTelemetry(opMode.telemetry, dashboard.getTelemetry());

        et = new ElapsedTime();

        dropParkArm();
    }

    public static double round2dp(double number, int dp) {
        double temp = Math.pow(10, dp);
        double temp1 = Math.round(number * temp);
        return temp1 / temp;
    }


    public void setTargetInches(double targetInches) {
        leftArmController.reset(getLeftPositionInches());
        rightArmController.reset(getRightPositionInches());
        leftArmController.setGoal(targetInches);
        rightArmController.setGoal(targetInches);
    }

    public Action positionArm(double target) {
        return new Action() {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    inPositionCtr = 0;
                    setTargetInches(target);
                    initialized = true;
                }
                packet.put("moving", atGoal());
                return !atGoal();
            }
        };
    }

    public boolean checkOK() {
        shutDownArmPositioning = Math.abs(getLeftPositionInches() - getRightPositionInches()) > lrDiffMaxInches;
        return !shutDownArmPositioning;
    }

    @Override
    public void periodic() {
        if (showSelect == Constants.ShowTelemetryConstants.showArmCommon)
            showTelemetryCommon();
        if (showSelect == Constants.ShowTelemetryConstants.showArm1)
            showTelemetryLeft();
        if (showSelect == Constants.ShowTelemetryConstants.showArm2)
            showTelemetryRight();

        shutDownArmPositioning = Math.abs(getLeftPositionInches() - getRightPositionInches()) > lrDiffMaxInches;


        if (holdCtr >= 100) {
            scanTime = et.milliseconds() / holdCtr;
            holdCtr = 0;
            et.reset();
        }
        if (TUNE) tuning();
    }

    public void tuning() {
        if (lastTargetInches != TARGET) {
            setTargetInches(TARGET);
            lastTargetInches = TARGET;
        }

        if (leftArmController.getP() != kp) {
            leftArmController.setP(kp);
            rightArmController.setP(kp);
        }
        if (leftArmController.getI() != ki) {
            leftArmController.setI(ki);
            rightArmController.setI(ki);
        }
        if (leftArmController.getD() != kd) {
            leftArmController.setD(kd);
            rightArmController.setD(kd);
        }

    }

    public void position() {

        if (inPositionCtr != 3) inPositionCtr++;

        boolean armOutLimit = getLeftPositionInches() >= OUT_POSITION_LIMIT || getRightPositionInches() >= OUT_POSITION_LIMIT;
        boolean armInLimit = getLeftPositionInches() <= IN_POSITION_LIMIT || getRightPositionInches() >= OUT_POSITION_LIMIT;


        leftPidOut = leftArmController.calculate(getLeftPositionInches());


        double leftPowerVal = clamp(leftPidOut, -1, 1);

        if (!shutDownArmPositioning && (leftPowerVal > 0 && !armOutLimit || leftPowerVal < 0 && !armInLimit))
            leftArmMotor.set(leftPowerVal);
        else leftArmMotor.set(0);


        rightPidOut = rightArmController.calculate(getRightPositionInches());

        double rightPowerVal = clamp(rightPidOut, -1, 1);

        if (!shutDownArmPositioning && (rightPowerVal > 0 && !armOutLimit || rightPowerVal < 0 && !armInLimit))
            rightArmMotor.set(rightPowerVal);
        else rightArmMotor.set(0);
    }

    public void resetEncoders() {
        leftArmEncoder.reset();
        rightArmEncoder.reset();
        setTargetInches(0);
    }

    public Action dropParkArm() {
        return new InstantAction(() -> parkArmServo.setPosition(parkArmDown));
    }

    public Action raiseParkArm() {
        return new InstantAction(() -> parkArmServo.setPosition(parkArmUp));
    }

    public double getLeftPositionInches() {
        return leftArmEncoder.getDistance();
    }

    public double getRightPositionInches() {
        return rightArmEncoder.getDistance();
    }

    public double getLeftInchesPerSec() {
        return leftArmEncoder.getRate();
    }

    public double getRightInchesPerSec() {
        return rightArmEncoder.getRate();
    }

    public void setLeftPower(double power) {
        leftArmMotor.set(power);
    }

    public void setRightPower(double power) {
        rightArmMotor.set(power);
    }

    public boolean atLeftGoal() {
        return inPositionCtr == 3 && leftArmController.atGoal();
    }

    public boolean atRightGoal() {
        return inPositionCtr == 3 && rightArmController.atGoal();
    }

    public boolean atGoal() {
        return atLeftGoal() && atRightGoal();
    }

    public Action armToPickupAction() {
        return positionArm(Constants.ExtendArmConstants.pickupDistance);
    }

    public Action armToAutoPickupAction() {
        return positionArm(Constants.ExtendArmConstants.autoPickupDistance);
    }

    public Action armToHome() {
        return positionArm(Constants.ExtendArmConstants.home);
    }

    public void showTelemetryCommon() {
        telemetry.addData("ArmCommon", showSelect);
        telemetry.addData("ArmLeftInches", round2dp(getLeftPositionInches(), 2));
        telemetry.addData("ArmRightInches", round2dp(getRightPositionInches(), 2));
        telemetry.addData("ArmLeftPower", round2dp(leftArmMotor.get(), 2));
        telemetry.addData("ArmRightInches", round2dp(getRightPositionInches(), 2));
        telemetry.addData("ArmRightPower", round2dp(rightArmMotor.get(), 2));
        telemetry.addData("ArmShutdownPositioning", shutDownArmPositioning);
        telemetry.update();
    }

    public void showTelemetryLeft() {
        telemetry.addData("ArmLeft", showSelect);
        telemetry.addData("Arm Left at Goal", atLeftGoal());
        telemetry.addData("ArmLeftTargetInches", TARGET);
        telemetry.addData("ArmLeftInches", round2dp(getLeftPositionInches(), 2));
        telemetry.addData("ArmLeftSetPointPos", round2dp(leftArmSetPoint.position, 2));
        telemetry.addData("ArmLeftSetPointVel", round2dp(leftArmSetPoint.velocity, 2));
        telemetry.addData("ArmLeftPosError", round2dp(leftArmController.getPositionError(), 2));
        telemetry.addData("ArmLeftVelError", round2dp(leftArmController.getVelocityError(), 2));
        telemetry.addData("ArmLeftVel", round2dp(getLeftInchesPerSec(), 2));
        telemetry.addData("ArmLeftPID", round2dp(leftPidOut, 2));
        telemetry.addData("ArmLeftPower", round2dp(leftArmMotor.get(), 2));
        telemetry.update();
    }

    public void showTelemetryRight() {
        telemetry.addData("ArmRight", showSelect);
        telemetry.addData("Arm Right at Goal", atRightGoal());
        telemetry.addData("ArmRightTargetInches", TARGET);
        telemetry.addData("ArmRightInches", round2dp(getRightPositionInches(), 2));
        telemetry.addData("ArmRightSetPointPos", round2dp(rightArmSetPoint.position, 2));
        telemetry.addData("ArmRightSetPointV", round2dp(rightArmSetPoint.velocity, 2));
        telemetry.addData("ArmRightPosError", round2dp(rightArmController.getPositionError(), 2));
        telemetry.addData("ArmRightVelError", round2dp(rightArmController.getVelocityError(), 2));
        telemetry.addData("ArmRightVel", round2dp(getRightInchesPerSec(), 2));
        telemetry.addData("ArmRightPID", round2dp(rightPidOut, 2));
        telemetry.addData("ArmRightPower", round2dp(rightArmMotor.get(), 2));
        telemetry.update();
    }


}
