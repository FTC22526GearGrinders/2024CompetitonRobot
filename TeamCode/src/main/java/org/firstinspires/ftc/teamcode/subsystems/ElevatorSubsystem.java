package org.firstinspires.ftc.teamcode.subsystems;

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
import com.arcrobotics.ftclib.controller.wpilibcontroller.ProfiledPIDController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.trajectory.TrapezoidProfile;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.utils.ConditionalAction;


@Config
public class ElevatorSubsystem extends SubsystemBase {


    public static double ekp = 0.5;
    public static double eki = 0;
    public static double ekd = 0;
    public static boolean TUNE = false;
    public static double TARGET;
    public final double UPPER_POSITION_LIMIT = 28;
    public final double LOWER_POSITION_LIMIT = 0;
    public final double TRAJECTORY_VEL = 15;
    public final double TRAJECTORY_ACCEL = 10;
    public final double minimumHoldHeight = 0;//1.2;
    private final Telemetry telemetry;
    private final double lrDiffMaxInches = 5;
    private final double bucketCycleTime = .75;
    public double releaseDelay = .5;
    public double specimenClawOpenAngle = 0.0;
    public double specimenClawClosedAngle = .4;
    public double bucketUprightAngle = .5;
    public double bucketTravelAngle = .4;
    public double bucketTippedAngle = 0;
    public ElapsedTime holdTime;
    public TrapezoidProfile.Constraints constraints;
    public Motor leftElevatorMotor;
    public Motor.Encoder leftElevatorEncoder;
    public ProfiledPIDController leftPidController;
    public TrapezoidProfile.State leftGoal = new TrapezoidProfile.State();
    public TrapezoidProfile.State leftSetpoint = new TrapezoidProfile.State();
    public Motor rightElevatorMotor;
    public Motor.Encoder rightElevatorEncoder;
    public ProfiledPIDController rightPidController;
    public TrapezoidProfile.State rightGoal = new TrapezoidProfile.State();
    public TrapezoidProfile.State rightSetpoint = new TrapezoidProfile.State();
    public Servo bucketServo;
    public Servo specimenClawServo;
    public int holdCtr;
    public int showSelect = 0;
    public int posrng;
    public double leftPower;
    public double rightPower;

    public boolean shutDownElevatorPositioning;
    public double leftTotalPower;
    public double rightTotalPower;
    CommandOpMode myOpmode;
    double leftSetVel;
    double leftSetPos;
    double leftPidOut;
    double rightSetVel;
    double rightSetPos;

    double rightPidOut;

    private int inPositionCtr;

    public ElevatorSubsystem(CommandOpMode opMode) {

        myOpmode = opMode;

        constraints = new TrapezoidProfile.Constraints(TRAJECTORY_VEL, TRAJECTORY_ACCEL);


        leftElevatorMotor = new Motor(opMode.hardwareMap, "leftElevator", Motor.GoBILDA.RPM_1150);
        leftElevatorMotor.setInverted(true);
        leftElevatorMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);

        leftElevatorEncoder = leftElevatorMotor.encoder;
        leftElevatorEncoder.setDirection(Motor.Direction.FORWARD);
        leftElevatorEncoder.setDistancePerPulse(1 / Constants.ElevatorConstants.ENCODER_COUNTS_PER_INCH);

        leftPidController = new ProfiledPIDController(ekp, eki, ekd, constraints);
        leftPidController.setTolerance(Constants.ElevatorConstants.POSITION_TOLERANCE_INCHES);
        leftPidController.reset();


        rightElevatorMotor = new Motor(opMode.hardwareMap, "rightElevator", Motor.GoBILDA.RPM_1150);
        rightElevatorMotor.setInverted(false);
        rightElevatorMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);

        rightElevatorEncoder = rightElevatorMotor.encoder;
        rightElevatorEncoder.setDirection(Motor.Direction.FORWARD);
        rightElevatorEncoder.setDistancePerPulse(1 / Constants.ElevatorConstants.ENCODER_COUNTS_PER_INCH);

        rightPidController = new ProfiledPIDController(ekp, eki, ekd, constraints);
        rightPidController.setTolerance(Constants.ElevatorConstants.POSITION_TOLERANCE_INCHES);
        rightPidController.reset();


        bucketServo = opMode.hardwareMap.get(Servo.class, "bucketServo");
        specimenClawServo = opMode.hardwareMap.get(Servo.class, "specimenClawServo");

        bucketServo.setDirection(Servo.Direction.FORWARD);
        specimenClawServo.setDirection(Servo.Direction.FORWARD);

        // specimenClawSwitch = new Sensor()

        resetElevatorEncoders();

        setTargetInches(Constants.ElevatorConstants.HOME_POSITION);

        FtcDashboard dashboard = FtcDashboard.getInstance();

        telemetry = new MultipleTelemetry(opMode.telemetry, dashboard.getTelemetry());

        holdTime = new ElapsedTime();

        levelBucket();

        //setDefaultCommand(new PositionHoldElevator(this));
    }

    public static double round2dp(double number, int dp) {
        double temp = Math.pow(10, dp);
        double temp1 = Math.round(number * temp);
        return temp1 / temp;
    }

    public double getTargetInches() {
        return leftPidController.getGoal().position;
    }

    public void setTargetInches(double targetInches) {
        //leftPidController.reset(getLeftPositionInches());
        leftPidController.setGoal(targetInches);
        //rightPidController.reset(getRightPositionInches());
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

    public Action elevatorToClearWall() {
        return positionElevator(Constants.ElevatorConstants.elevatorClearOfWall);
    }


    public Action elevatorToAboveLowerSubmersible() {
        return positionElevator(Constants.ElevatorConstants.elevatorSpecimenAboveLowPlaceHeight);
    }

    public Action elevatorToLowerSubmersible() {
        return positionElevator(Constants.ElevatorConstants.elevatorSpecimenLowPlaceHeight);
    }

    public Action elevatorToNearestChamber() {
        return new ConditionalAction(elevatorToUpperSubmersible(), elevatorToLowerSubmersible(),
                getLeftPositionInches() > Constants.ElevatorConstants.elevatorSpecimenAboveDecisionHeight);
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
                packet.put("leftInches", getLeftPositionInches());
                packet.put("rightInches", getRightPositionInches());

                return atGoal();
            }
        };
    }


    public Action tipBucket() {
        return new InstantAction(() -> bucketServo.setPosition(bucketTippedAngle));
    }

    public Action travelBucket() {
        return new InstantAction(() -> bucketServo.setPosition(bucketTravelAngle));
    }

    public Action levelBucket() {
        return new InstantAction(() -> bucketServo.setPosition(bucketUprightAngle));
    }

    public Action cycleBucket() {
        return new SequentialAction(
                tipBucket(),
                new SleepAction(bucketCycleTime),
                levelBucket());
    }

    public Action closeSpecimenClaw() {
        return new InstantAction(() -> specimenClawServo.setPosition(specimenClawClosedAngle));//.5
    }

    public Action openSpecimenClaw() {
        return new InstantAction(() -> specimenClawServo.setPosition(specimenClawOpenAngle));//.3
    }

    public Action grabSpecimenAndClearWall() {
        return new SequentialAction(
                closeSpecimenClaw(),
                new SleepAction(1),
                elevatorToClearWall());
    }

    public Action deliverSpecimenToNearestChamber() {
        return
                new ParallelAction(
                        elevatorToNearestChamber(),
                        new SequentialAction(
                                new SleepAction(releaseDelay),
                                openSpecimenClaw()));
    }


    @Override

    public void periodic() {
        if (showSelect == Constants.ShowTelemetryConstants.showElevatorCommon) {
            showCommonTelemetry();
        }
        if (showSelect == Constants.ShowTelemetryConstants.showElevatorLeft) {
            showLeftTelemetry();
        }
        if (showSelect == Constants.ShowTelemetryConstants.showElevatorRight) {
            showRightTelemetry();
        }

        holdCtr++;

        if (holdCtr >= 50) {
            leftTotalPower += Math.abs(leftElevatorMotor.get());
            rightTotalPower += Math.abs(rightElevatorMotor.get());
            holdCtr = 0;
        }


        if (TUNE) {
            if (TARGET > UPPER_POSITION_LIMIT) TARGET = UPPER_POSITION_LIMIT;
            if (TARGET < LOWER_POSITION_LIMIT) TARGET = LOWER_POSITION_LIMIT;
            setTargetInches(TARGET);
            setGains();
        }
    }

    public boolean checkOK() {
        shutDownElevatorPositioning = Math.abs(getLeftPositionInches() - getRightPositionInches()) > lrDiffMaxInches;
        return !shutDownElevatorPositioning;
    }

    public void position() {
        posrng++;

        if (inPositionCtr != 3) inPositionCtr++;

        boolean elevatorHigh = false;// getLeftPositionInches() >= UPPER_POSITION_LIMIT || getRightPositionInches() >= UPPER_POSITION_LIMIT;
        boolean elevatorLow = false;//getLeftPositionInches() <= LOWER_POSITION_LIMIT || getRightPositionInches() <= LOWER_POSITION_LIMIT;


        leftPidOut = leftPidController.calculate(getLeftPositionInches());


        double leftPowerVal = leftPidOut;

        if (!shutDownElevatorPositioning && (leftPowerVal > 0 && !elevatorHigh || leftPowerVal < 0 && !elevatorLow))
            leftElevatorMotor.set(leftPowerVal);
        else leftElevatorMotor.set(0);


        rightPidOut = rightPidController.calculate(getRightPositionInches());

        double rightPowerVal = rightPidOut;

        if (!shutDownElevatorPositioning && (rightPowerVal > 0 && !elevatorHigh || rightPowerVal < 0 && !elevatorLow))
            rightElevatorMotor.set(rightPowerVal);
        else rightElevatorMotor.set(0);
    }


    public void setGains() {
        if (ekp != leftPidController.getP()) {
            leftPidController.setP(ekp);
            rightPidController.setP(ekp);
        }
        if (eki != leftPidController.getI()) {
            leftPidController.setP(eki);
            rightPidController.setP(eki);
        }

        if (ekd != leftPidController.getD()) {
            leftPidController.setP(ekd);
            rightPidController.setP(ekd);
        }

    }

    public void resetElevatorEncoders() {
        leftElevatorEncoder.reset();
        rightElevatorEncoder.reset();
    }

    public double getLeftPositionInches() {
        return leftElevatorEncoder.getDistance();
    }

    public double getLeftVelocityInPerSec() {
        return leftElevatorEncoder.getRate();
    }

    public double getRightPositionInches() {
        return rightElevatorEncoder.getDistance();
    }

    public double getRightVelocityInPerSec() {
        return rightElevatorEncoder.getRate();
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


    public void showCommonTelemetry() {
        telemetry.addData("ElevatorCommon", showSelect);
        telemetry.addData("Goal", leftPidController.getGoal().position);
        telemetry.addData("LeftPositionInches", round2dp(getLeftPositionInches(), 2));
        telemetry.addData("RightPositionInches", round2dp(getRightPositionInches(), 2));
        telemetry.addData("LeftPower", round2dp(getLeftPower(), 2));
        telemetry.addData("RightPower", round2dp(getRightPower(), 2));
        telemetry.addData("LeftTotalPower", round2dp(leftTotalPower, 2));
        telemetry.addData("RightTotalPower", round2dp(rightTotalPower, 2));
        telemetry.addData("Shutdown", shutDownElevatorPositioning);
        telemetry.addData("Target", TARGET);
        telemetry.update();

    }


    public void showLeftTelemetry() {
        telemetry.addData("ElevatorLeft", showSelect);

        telemetry.addData("HoldRng", posrng);
        telemetry.addData("ElevatorGoal", round2dp(leftPidController.getGoal().position, 2));
        telemetry.addData("Left at Goal", atLeftGoal());
        telemetry.addData("LeftPositionInches", round2dp(getLeftPositionInches(), 2));
        telemetry.addData("LeftPower", round2dp(getLeftPower(), 2));
        telemetry.addData("LeftVelErr", round2dp(leftPidController.getVelocityError(), 2));
        telemetry.addData("LeftPosErr", round2dp(leftPidController.getPositionError(), 2));
        telemetry.addData("LeftPIDOut", round2dp(leftPidOut, 2));
        telemetry.addData("LeftSetVel", round2dp(leftSetVel, 2));
        telemetry.addData("LeftSetPos", round2dp(leftSetPos, 2));

        telemetry.update();

    }

    public void showRightTelemetry() {
        telemetry.addData("ElevatorRight", showSelect);

        telemetry.addData("HoldRng", posrng);
        telemetry.addData("ElevatorGoal", rightPidController.getGoal().position);
        telemetry.addData("Right at Goal", atRightGoal());
        telemetry.addData("RightPositionInches", getRightPositionInches());
        telemetry.addData("RightPower", getRightPower());
        telemetry.addData("RightVelErr", round2dp(rightPidController.getVelocityError(), 2));
        telemetry.addData("RightPosErr", rightPidController.getPositionError());
        telemetry.addData("RightPIDOut", rightPidOut);
        telemetry.addData("RightSetVel", rightSetVel);
        telemetry.addData("RightSetPos", rightSetPos);

        telemetry.update();

    }
}
