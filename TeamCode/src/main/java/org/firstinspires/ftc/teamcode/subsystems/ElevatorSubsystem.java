package org.firstinspires.ftc.teamcode.subsystems;

import android.hardware.Sensor;

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
    public static double eks = 0.08;//1% motor power
    public static double ekg = 0.04;
    public static double ekv = .05;//per inch per second (max 17 ips )
    public static double eka = 0;

    public static double lkp = 0.05;
    public static double lki = 0;
    public static double lkd = 0;


    public static double rkp = 0.05;
    public static double rki = 0;
    public static double rkd = 0;

    public static boolean TUNING = false;

    public static double targetInches;


    private final Telemetry telemetry;
    public TrapezoidProfile.Constraints constraints;
    public Motor leftElevatorMotor;
    public Motor.Encoder leftElevatorEncoder;
    public ProfiledPIDController leftPidController;
    public TrapezoidProfile.State leftGoal = new TrapezoidProfile.State();
    public TrapezoidProfile.State leftSetpoint = new TrapezoidProfile.State();
    public ElevatorFeedforward elevatorFeedForward;
    public Motor rightElevatorMotor;
    public Motor.Encoder rightElevatorEncoder;
    public ProfiledPIDController rightPidController;
    public TrapezoidProfile.State rightGoal = new TrapezoidProfile.State();
    public TrapezoidProfile.State rightSetpoint = new TrapezoidProfile.State();
    public Servo bucketServo;
    public Servo specimenClawServo;
    public Sensor specimenClawSwitch;
    public int holdCtr;
    public int showSelect = 0;

    public double currentSpecimenClawAngle;
    public int posrng;
    public double leftPower;
    public double rightPower;
    public boolean elevatorTest;
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
    private int targetSetCounter;
    private int inPositionCtr;

    public ElevatorSubsystem(CommandOpMode opMode) {

        myOpmode = opMode;

        constraints = new TrapezoidProfile.Constraints(Constants.ElevatorConstants.TRAJ_VEL, Constants.ElevatorConstants.TRAJ_ACCEL);


        leftElevatorMotor = new Motor(opMode.hardwareMap, "leftElevator");
        leftElevatorMotor.setInverted(true);
        leftElevatorMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);

        leftElevatorEncoder = leftElevatorMotor.encoder;
        leftElevatorEncoder.setDirection(Motor.Direction.FORWARD);
        leftElevatorEncoder.setDistancePerPulse(1 / Constants.ElevatorConstants.ENCODER_COUNTS_PER_INCH);

        elevatorFeedForward = new ElevatorFeedforward(eks, ekg, ekv, eka);
        leftPidController = new ProfiledPIDController(lkp, lki, lkd, constraints);
        leftPidController.setTolerance(Constants.ElevatorConstants.POSITION_TOLERANCE_INCHES);
        leftPidController.reset();


        rightElevatorMotor = new Motor(opMode.hardwareMap, "rightElevator");
        rightElevatorMotor.setInverted(true);
        rightElevatorMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);

        rightElevatorEncoder = rightElevatorMotor.encoder;
        rightElevatorEncoder.setDirection(Motor.Direction.FORWARD);
        rightElevatorEncoder.setDistancePerPulse(1 / Constants.ElevatorConstants.ENCODER_COUNTS_PER_INCH);

        rightPidController = new ProfiledPIDController(rkp, rki, rkd, constraints);
        rightPidController.setTolerance(Constants.ElevatorConstants.POSITION_TOLERANCE_INCHES);
        rightPidController.reset();


        bucketServo = opMode.hardwareMap.get(Servo.class, "bucketServo");
        specimenClawServo = opMode.hardwareMap.get(Servo.class, "specimenClawServo");

        bucketServo.setDirection(Servo.Direction.REVERSE);
        specimenClawServo.setDirection(Servo.Direction.FORWARD);

        // specimenClawSwitch = new Sensor()

        resetElevatorEncoders();

        setTargetInches(Constants.ElevatorConstants.HOME_POSITION);

        FtcDashboard dashboard = FtcDashboard.getInstance();

        telemetry = new MultipleTelemetry(opMode.telemetry, dashboard.getTelemetry());


        et = new ElapsedTime();

        levelBucket();
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

    public Action positionElevator(double target) {
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

    public Action elevatorToHome() {
        return positionElevator(Constants.ElevatorConstants.HOME_POSITION);
    }

    public Action elevatorToSpecimenOnWall() {
        return positionElevator(Constants.ElevatorConstants.elevatorSpecimenWallHeight);
    }

    public Action elevatorToAboveLowSubmersible() {
        return positionElevator(Constants.ElevatorConstants.elevatorSpecimenAboveLowPlaceHeight);
    }

    public Action elevatorToLowSubmersible() {
        return positionElevator(Constants.ElevatorConstants.elevatorSpecimenLowPlaceHeight);
    }

    public Action elevatorToAboveUpperSubmersible() {
        return positionElevator(Constants.ElevatorConstants.elevatorSpecimenAboveHighPlaceHeight);
    }

    public Action elevatorToUpperSubmersible() {
        return positionElevator(Constants.ElevatorConstants.elevatorSpecimenAtHighPlaceHeight);
    }

    public Action elevatorToLowBasket() {
        return positionElevator(Constants.ElevatorConstants.elevatorLowerBasketHeight);
    }

    public Action elevatorToUpperBasket() {
        return positionElevator(Constants.ElevatorConstants.elevatorUpperBasketHeight);
    }


    public double getPositionInches() {
        return leftElevatorEncoder.getPosition();
    }

    public boolean atGoal() {
        return atLeftGoal() && atRightGoal();
    }

    public boolean atLeftGoal() {
        return inPositionCtr == 3 && leftPidController.atGoal();
    }

    public boolean atRightGoal() {
        return inPositionCtr == 3 && rightPidController.atGoal();
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

    public Action tipBucketDelayed(double time) {
        return new SequentialAction(
                new SleepAction(time),
                new InstantAction(() -> bucketServo.setPosition(Constants.ElevatorConstants.bucketTippedAngle)));
    }

    public Action levelBucket() {
        return new InstantAction(() -> bucketServo.setPosition(Constants.ElevatorConstants.bucketUprightAngle));
    }

    public Action cycleBucket(double timeoutsecs) {
        return new SequentialAction(
                tipBucket(),
                new SleepAction(timeoutsecs),
                levelBucket());
    }

    public Action closeSpecimenClaw() {
        return new InstantAction(() -> specimenClawServo.setPosition(Constants.ElevatorConstants.specimenClawClosedAngle));//.5
    }

    public Action openSpecimenClaw() {
        return new InstantAction(() -> specimenClawServo.setPosition(Constants.ElevatorConstants.specimenClawOpenAngle));//.3
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

        if (showSelect == Constants.MiscConstants.showRotateArm1) {
            showLeftTelemetry();
        }
        if (showSelect == Constants.MiscConstants.showRotateArm1) {
            showRightTelemetry();
        }


        if (holdCtr >= 100) {
            scanTime = et.milliseconds() / holdCtr;
            holdCtr = 0;
            et.reset();
        }

        if (TUNING) {
            setTargetInches(targetInches);
            setNewFFValues();
            setGains();
        }
    }

    public double getLeftVoltsPerIPS() {
        return (leftPower - .4) / getLeftVelocityInPerSec();
    }

    public void position() {
        posrng++;

        if (inPositionCtr != 3) inPositionCtr++;
        leftPidout = leftPidController.calculate(getLeftPositionInches());
        leftSetpoint = leftPidController.getSetpoint();
        leftSetVel = leftSetpoint.velocity;
        leftSetPos = leftSetpoint.position;
        leftAccel = (leftSetVel - leftLastVel) * 50;
        leftFf = elevatorFeedForward.calculate(leftSetVel, leftAccel);
        leftElevatorMotor.set(leftFf + leftPidout);
        leftLastVel = leftSetVel;

        rightPidout = rightPidController.calculate(getRightPositionInches());
        rightSetpoint = rightPidController.getSetpoint();
        rightSetVel = rightSetpoint.velocity;
        rightSetPos = rightSetpoint.position;
        rightAccel = (rightSetVel - rightLastVel) * 50;
        rightFf = elevatorFeedForward.calculate(rightSetVel, rightAccel);
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
        elevatorFeedForward = new ElevatorFeedforward(eks, ekg, ekv, eka);

    }

    public boolean getSpecimenClawPressed() {
        return true;
    }

    public void setGains() {
        if (lkp != leftPidController.getP())
            setLeftPositionKp();
        if (lki != leftPidController.getI())
            setLefPositionKi();
        if (lkd != leftPidController.getD())
            setLeftPositionKd();
        if (rkp != rightPidController.getP())
            setRightPositionKp();
        if (rki != rightPidController.getI())
            setRightPositionKi();
        if (rkd != rightPidController.getD())
            setRightPositionKd();
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
        return round2dp(rightElevatorEncoder.getDistance(), 2);
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
        telemetry.addData("ElevatorLeft", showSelect);

        telemetry.addData("HoldRng", posrng);

        telemetry.addData("LeftPositionInches", getLeftPositionInches());
        telemetry.addData("RightPositionInches", getRightPositionInches());
        telemetry.addData("ElevatorGoal", leftPidController.getGoal().position);
        telemetry.addData("LeftPower", getLeftPower());
        telemetry.addData("RightPower", getRightPower());
        telemetry.addData("LeftPosErr", leftPidController.getPositionError());
        telemetry.addData("RightPosErr", rightPidController.getPositionError());
//        telemetry.addData("LeftFF", leftFf);
//        telemetry.addData("LeftPIDout", leftPidout);
//        telemetry.addData("LeftSetVel", leftSetVel);
//        telemetry.addData("LeftSetPos", leftSetPos);

        telemetry.update();

    }

    public void showRightTelemetry() {
        telemetry.addData("ElevatorRight", showSelect);

        telemetry.addData("HoldRng", posrng);

        telemetry.addData("RightPositionInches", getRightPositionInches());
        telemetry.addData("LeftPositionInches", getLeftPositionInches());
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
