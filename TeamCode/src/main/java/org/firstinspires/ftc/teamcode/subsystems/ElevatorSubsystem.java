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
import com.arcrobotics.ftclib.controller.wpilibcontroller.ElevatorFeedforward;
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
    //units used are per unit motor setting since motor setVolts isn't available
    public static double eks = 0.15;//1% motor power
    public static double ekg = 0.25;
    public static double ekv = .08;//max ips = 16 so 12/16 (assume 50%) = .35 volts per ips
    public static double eka = 0;

    public static double lkp = 0.2;
    public static double lki = 0;
    public static double lkd = 0;

    public static double rkp = 0.2;
    public static double rki = 0;
    public static double rkd = 0;

    public static boolean TUNING = false;

    public static double targetInches;

    public double UPPER_POSITION_LIMIT = 28;
    public int LOWER_POSITION_LIMIT = 0;

    public static double TRAJ_VEL = 15;
    public static double TRAJ_ACCEL = 15;

    public double specimenClawOpenAngle = 0.0;
    public double specimenClawClosedAngle = .3;

    public double bucketUprightAngle = .5;
    public double bucketTravelAngle = .4;
    public double bucketTippedAngle = 0;
    public double releaseDelay = .5;
    private double bucketCycleTime = .75;

    public final double minimumHoldHeight = 1.2;
    private final Telemetry telemetry;
    private final double lrDiffMaxInches = 3;
    public ElapsedTime holdTime;
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
    public int holdCtr;
    public int showSelect = 0;
    public int posrng;
    public double leftPower;
    public double rightPower;
    public boolean openLoop;
    public boolean shutDownElevatorPositioning;
    public double leftTotalPower;
    public double rightTotalPower;
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
    private int inPositionCtr;

    public ElevatorSubsystem(CommandOpMode opMode) {

        myOpmode = opMode;

        constraints = new TrapezoidProfile.Constraints(TRAJ_VEL, TRAJ_ACCEL);


        leftElevatorMotor = new Motor(opMode.hardwareMap, "leftElevator", Motor.GoBILDA.RPM_1150);
        leftElevatorMotor.setInverted(true);
        leftElevatorMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);

        leftElevatorEncoder = leftElevatorMotor.encoder;
        leftElevatorEncoder.setDirection(Motor.Direction.FORWARD);
        leftElevatorEncoder.setDistancePerPulse(1 / Constants.ElevatorConstants.ENCODER_COUNTS_PER_INCH);

        elevatorFeedForward = new ElevatorFeedforward(eks, ekg, ekv, eka);
        leftPidController = new ProfiledPIDController(lkp, lki, lkd, constraints);
        leftPidController.setTolerance(Constants.ElevatorConstants.POSITION_TOLERANCE_INCHES);
        leftPidController.reset();


        rightElevatorMotor = new Motor(opMode.hardwareMap, "rightElevator", Motor.GoBILDA.RPM_1150);
        rightElevatorMotor.setInverted(false);
        rightElevatorMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);

        rightElevatorEncoder = rightElevatorMotor.encoder;
        rightElevatorEncoder.setDirection(Motor.Direction.FORWARD);
        rightElevatorEncoder.setDistancePerPulse(1 / Constants.ElevatorConstants.ENCODER_COUNTS_PER_INCH);

        rightPidController = new ProfiledPIDController(rkp, rki, rkd, constraints);
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
                                new SleepAction(.5),
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

        if (Math.abs(getLeftPositionInches() - getRightPositionInches()) > lrDiffMaxInches)
            shutDownElevatorPositioning = true;


        if (TUNING) {
            setTargetInches(targetInches);
            setNewFFValues();
            setGains();
        }
    }

    public boolean checkOK() {

        return Math.abs(getLeftPositionInches() - getRightPositionInches()) < lrDiffMaxInches;
    }

    public void position() {
        posrng++;

        if (inPositionCtr != 3) inPositionCtr++;

//        double izonel = 1;
//
//        if (Math.abs(leftPidController.getGoal().position - getLeftPositionInches()) < izonel)
//            leftPidController.setI(.0001);
//        else leftPidController.setI(0);

        leftPidout = leftPidController.calculate(getLeftPositionInches());
        leftSetpoint = leftPidController.getSetpoint();
        leftSetVel = leftSetpoint.velocity;
        leftSetPos = leftSetpoint.position;
        leftAccel = (leftSetVel - leftLastVel) * 50;
        leftFf = elevatorFeedForward.calculate(leftSetVel, leftAccel);
        leftElevatorMotor.set(leftFf + leftPidout);
        leftLastVel = leftSetVel;

        rightPidout = rightPidController.calculate(getRightPositionInches());

//        double izoner = 1;
//
//        if (Math.abs(rightPidController.getGoal().position - getRightPositionInches()) < izoner)
//            rightPidController.setI(.0001);
//        else rightPidController.setI(0);


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
        return round2dp(leftElevatorEncoder.getRate(), 2);
    }

    public double getRightPositionInches() {
        return round2dp(rightElevatorEncoder.getDistance(), 2);
    }

    public double getRightVelocityInPerSec() {
        return round2dp(rightElevatorEncoder.getRate(), 2);
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
        telemetry.addData("LeftPositionInches", getLeftPositionInches());
        telemetry.addData("RightPositionInches", getRightPositionInches());
        telemetry.addData("LeftPower", getLeftPower());
        telemetry.addData("RightPower", getRightPower());
        telemetry.addData("LeftTotalPower", leftTotalPower);
        telemetry.addData("RightTotalPower", rightTotalPower);
        telemetry.addData("Shutdown", shutDownElevatorPositioning);
        telemetry.update();

    }


    public void showLeftTelemetry() {
        telemetry.addData("ElevatorLeft", showSelect);

        telemetry.addData("HoldRng", posrng);

        telemetry.addData("LeftPositionInches", getLeftPositionInches());
        telemetry.addData("ElevatorGoal", leftPidController.getGoal().position);
        telemetry.addData("LeftPower", getLeftPower());
        telemetry.addData("LeftPosErr", leftPidController.getPositionError());
        telemetry.addData("RightPosErr", rightPidController.getPositionError());
        telemetry.addData("LeftFF", leftFf);
        telemetry.addData("LeftPIDout", leftPidout);
        telemetry.addData("LeftSetVel", leftSetVel);
        telemetry.addData("LeftSetPos", leftSetPos);

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
