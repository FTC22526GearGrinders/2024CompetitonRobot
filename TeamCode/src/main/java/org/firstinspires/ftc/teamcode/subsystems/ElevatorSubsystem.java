package org.firstinspires.ftc.teamcode.subsystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.wpilibcontroller.ElevatorFeedforward;
import com.arcrobotics.ftclib.controller.wpilibcontroller.ProfiledPIDController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.trajectory.TrapezoidProfile;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants;


@Config
public class ElevatorSubsystem extends SubsystemBase {
    //units used are per unit motor setting since motor setVolts isn't available
    public static double lks = 0.;//1% motor power
    public static double lkg = 0;
    public static double lkv = .04;//per inch per second (max 17 ips )
    public static double lka = 0;
    public static double lkp = 0.01;
    public static double lki = 0;
    public static double lkd = 0;

    public static double rks = 0.;//1% motor power
    public static double rkg = 0;
    public static double rkv = .04;//per inch per second (max 17 ips )
    public static double rka = 0;
    public static double rkp = 0.01;
    public static double rki = 0;
    public static double rkd = 0;


    public static double lkP = 1.5;
    public static double lkI = 0;
    public static double lkD = 0;

    public static double rkP = 1.5;
    public static double rkI = 0;
    public static double rkD = 0;


    public static double targetInches;
    private final Telemetry telemetry;
    public TrapezoidProfile.Constraints constraints;
    public Motor leftElevatorMotor;
    public Motor.Encoder leftElevatorEncoder;
    public ProfiledPIDController leftPidController;
    public TrapezoidProfile.State leftGoal = new TrapezoidProfile.State();
    public TrapezoidProfile.State leftSetpoint = new TrapezoidProfile.State();
    public ElevatorFeedforward leftFeedForward;
    public Motor rightElevatorMotor;
    public Motor.Encoder rightElevatorEncoder;
    public ProfiledPIDController rightPidController;
    public TrapezoidProfile.State rightGoal = new TrapezoidProfile.State();
    public TrapezoidProfile.State rightSetpoint = new TrapezoidProfile.State();
    public ElevatorFeedforward rightFeedForward;
    public Servo bucketServo;
    public Servo sampleClawServo;

    public int holdCtr;
    public int show = 0;

    public double currentSampleClawAngle;
    public int posrng;
    public double leftPower;
    public double rightPower;
    ElapsedTime et;
    CommandOpMode myOpmode;
    double leftSetVel;
    double leftSetPos;
    double leftFf;
    double leftPidout;
    double rightSetVel;
    double rightSetPos;
    double rightFf;
    double rightPidout;
    private double leftAccel;
    private double leftLastVel;
    private double rightAccel;
    private double rightLastVel;
    private double scanTime;

    public ElevatorSubsystem(CommandOpMode opMode) {

        myOpmode = opMode;

        constraints = new TrapezoidProfile.Constraints(Constants.ElevatorConstants.TRAJ_VEL, Constants.ElevatorConstants.TRAJ_ACCEL);


        leftElevatorMotor = new Motor(opMode.hardwareMap, "leftElevatorMotor", Motor.GoBILDA.RPM_312);
        leftElevatorMotor.setInverted(true);
        leftElevatorMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
        leftElevatorEncoder = leftElevatorMotor.encoder;
        leftElevatorEncoder.setDirection(Motor.Direction.FORWARD);
        leftElevatorEncoder.setDistancePerPulse(1 / Constants.ElevatorConstants.ENCODER_COUNTS_PER_INCH);
        leftFeedForward = new ElevatorFeedforward(lks, lkg, lkv, lka);
        leftPidController = new ProfiledPIDController(lkp, lki, lkd, constraints);
        leftPidController.setTolerance(Constants.ElevatorConstants.POSITION_TOLERANCE_INCHES);
        leftPidController.reset();


        rightElevatorMotor = new Motor(opMode.hardwareMap, "rightElevatorMotor", Motor.GoBILDA.RPM_312);
        rightElevatorMotor.setInverted(false);
        rightElevatorMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
        rightElevatorEncoder = rightElevatorMotor.encoder;
        rightElevatorEncoder.setDirection(Motor.Direction.FORWARD);
        rightElevatorEncoder.setDistancePerPulse(1 / Constants.ElevatorConstants.ENCODER_COUNTS_PER_INCH);
        rightFeedForward = new ElevatorFeedforward(rks, rkg, rkv, rka);
        rightPidController = new ProfiledPIDController(rkp, rki, rkd, constraints);
        rightPidController.setTolerance(Constants.ElevatorConstants.POSITION_TOLERANCE_INCHES);
        rightPidController.reset();


        bucketServo = opMode.hardwareMap.get(Servo.class, "bucketServo");
        sampleClawServo = opMode.hardwareMap.get(Servo.class, "sampleClawServo");

        bucketServo.setDirection(Servo.Direction.FORWARD);
        sampleClawServo.setDirection(Servo.Direction.FORWARD);

        resetElevatorEncoders();

        setTargetInches(Constants.ElevatorConstants.HOME_POSITION);

        FtcDashboard dashboard = FtcDashboard.getInstance();

        telemetry = new MultipleTelemetry(opMode.telemetry, dashboard.getTelemetry());


        et = new ElapsedTime();
    }

    public static double round2dp(double number, int dp) {
        double temp = Math.pow(10, dp);
        double temp1 = Math.round(number * temp);
        return temp1 / temp;
    }

    public double getTargetInches() {
        return targetInches;
    }

    public void setTargetInches(double inches) {
        targetInches = inches;
        leftPidController.setGoal(targetInches);
        rightPidController.setGoal(targetInches);
    }

    public Action setWaitAtTarget(double target) {
        return new Action() {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    setTargetInches(target);
                    initialized = true;
                }
                packet.put("target", target);
                packet.put("actual", getPositionInches());
                return atGoal();
            }
        };
    }

    public double getPositionInches() {
        return leftElevatorEncoder.getPosition();
    }

    public boolean atGoal() {
        return leftPidController.atGoal() && rightPidController.atGoal();
    }

    public Action setTarget(double targetInches) {
        return new Action() {
            private boolean initialized = false;


            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    setTargetInches(targetInches);
                    initialized = true;
                }
                packet.put("targetInches", getTargetInches());
                packet.put("actualInches", getPositionInches());

                return atGoal();
            }
        };
    }

    public Action tipBucket() {
        return new InstantAction(() -> bucketServo.setPosition(Constants.ElevatorConstants.bucketTippedAngle));
    }

    public Action levelBucket() {
        return new InstantAction(() -> bucketServo.setPosition(Constants.ElevatorConstants.bucketUprightAngle));
    }

    public Action closeSampleClaw() {
        return new InstantAction(() -> sampleClawServo.setPosition(Constants.ElevatorConstants.sampleClawClosedAngle));
    }

    public Action openSampleClaw() {
        return new InstantAction(() -> bucketServo.setPosition(Constants.ElevatorConstants.sampleClawOpenAngle));
    }

    public Action deliverToTopBasket() {
        return new SequentialAction(
                setTarget(Constants.ElevatorConstants.upperBasketDeliverPosition),
                tipBucket(),
                new SleepAction(1),
                setTarget(Constants.ElevatorConstants.homePosition));
    }

    public Action deliverToLowerBasket() {
        return new SequentialAction(
                setTarget(Constants.ElevatorConstants.lowerBasketDeliverPosition),
                tipBucket(),
                new SleepAction(1),
                setTarget(Constants.ElevatorConstants.homePosition));
    }


    @Override

    public void periodic() {

        //    if (show == 0) {// Constants.TelemetryConstants.showRotateArm) {
        showLeftTelemetry();
        //   }
        if (holdCtr >= 100) {
            scanTime = et.milliseconds() / holdCtr;
            holdCtr = 0;
            et.reset();
        }
    }

    public double getLeftVoltsPerIPS() {
        return (leftPower - .4) / getLeftVelocityInPerSec();
    }

    public void position() {
        posrng++;

        leftPidout = leftPidController.calculate(getLeftPositionInches());
        leftSetpoint = leftPidController.getSetpoint();
        leftSetVel = leftSetpoint.velocity;
        leftSetPos = leftSetpoint.position;
        leftAccel = (leftSetVel - leftLastVel) * 50;
        leftFf = leftFeedForward.calculate(leftSetVel, leftAccel);
        leftElevatorMotor.set(leftFf + leftPidout);
        leftLastVel = leftSetVel;

        rightPidout = rightPidController.calculate(getRightPositionInches());
        rightSetpoint = rightPidController.getSetpoint();
        rightSetVel = rightSetpoint.velocity;
        rightSetPos = rightSetpoint.position;
        rightAccel = (rightSetVel - rightLastVel) * 50;
        rightFf = rightFeedForward.calculate(rightSetVel, rightAccel);
        rightElevatorMotor.set(rightFf + rightPidout);
        rightLastVel = rightSetVel;
    }

    public void setLeftPositionKp() {
        leftPidController.setP(lkp);
    }

    public void setLefPositionKi() {
        leftPidController.setI(lki);
    }

    public void setLeftPositionKd() {
        leftPidController.setD(lkd);
    }

    public void setRightPositionKp() {
        rightPidController.setP(rkp);
    }

    public void setRightPositionKi() {
        rightPidController.setI(rki);
    }

    public void setRightPositionKd() {
        rightPidController.setD(rkd);
    }

    public void setNewFFValues() {
        leftFeedForward = new ElevatorFeedforward(lks, lkg, lkv, lka);
        rightFeedForward = new ElevatorFeedforward(rks, rkg, rkv, rka);
    }

    public void resetElevatorEncoders() {
        leftElevatorEncoder.reset();
        rightElevatorEncoder.reset();
    }

    public double getLeftPositionInches() {
        return round2dp(leftElevatorEncoder.getDistance(), 2);
    }

    public double getLeftVelocityInPerSec() {
        return round2dp(leftElevatorEncoder.getCorrectedVelocity() / 60, 2);
    }

    public double getRightPositionInches() {
        return round2dp(leftElevatorEncoder.getDistance(), 2);
    }

    public double getRightVelocityInPerSec() {
        return round2dp(rightElevatorEncoder.getCorrectedVelocity() / 60, 2);
    }

    public boolean leftInPosition() {
        return leftPidController.atGoal();
    }

    public double getLeftPower() {
        return leftElevatorMotor.get();
    }

    public void setLeftMotorPower(double leftPower) {
        leftElevatorMotor.set(leftPower);
    }

    public boolean rightInPosition() {
        return rightPidController.atGoal();
    }

    public double getRightPower() {
        return rightElevatorMotor.get();
    }

    public void setRightMotorPower(double leftPower) {
        rightElevatorMotor.set(leftPower);
    }


    public void showLeftTelemetry() {
        telemetry.addData("ElevatorLeft", show);

        telemetry.addData("HoldRng", posrng);

        telemetry.addData("LeftPositionInches", getLeftPositionInches());
        telemetry.addData("RightPositionInches", getRightPositionInches());
        telemetry.addData("ElevatorGoal", leftPidController.getGoal().position);
        telemetry.addData("LeftPower", getLeftPower());
        telemetry.addData("LeftPosErr", leftPidController.getPositionError());
        telemetry.addData("LeftFF", leftFf);
        telemetry.addData("LeftPIDout", leftPidout);
        telemetry.addData("LeftSetVel", leftSetVel);
        telemetry.addData("LeftSetPos", leftSetPos);

        telemetry.update();

    }

    public void showRightTelemetry() {
        telemetry.addData("ElevatorRight", show);

        telemetry.addData("HoldRng", posrng);

        telemetry.addData("RightPositionInches", getRightPositionInches());
        telemetry.addData("RightPositionInches", getRightPositionInches());
        telemetry.addData("ElevatorGoal", rightPidController.getGoal().position);
        telemetry.addData("RightPower", getRightPower());
        telemetry.addData("RightPosErr", rightPidController.getPositionError());
        telemetry.addData("RightFF", rightFf);
        telemetry.addData("RightPIDout", rightPidout);
        telemetry.addData("RightSetVel", rightSetVel);
        telemetry.addData("RightSetPos", rightSetPos);

        telemetry.update();

    }
}
