package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.wpilibcontroller.ElevatorFeedforward;
import com.arcrobotics.ftclib.controller.wpilibcontroller.ProfiledPIDController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.trajectory.TrapezoidProfile;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class ElevatorSubsystem extends SubsystemBase {


    public static final double MAX_INCHES_PER_SECOND = 50;// approx
    public static final double UPPER_POSITION_LIMIT = 20;
    public static final int LOWER_POSITION_LIMIT = 5;
    private static final double ENCODER_COUNTS_PER_INCH = 113.8;
    private static final double HOME_POSITION = 0;
    private static final double POSITION_TOLERANCE_INCHES = .1;
    public static double MAX_VEL = 30;
    public static double MAX_ACCEL = 30;
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


    private final Telemetry telemetry;

    public double targetInches;
    ElapsedTime et;
    CommandOpMode myOpmode;

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

    public int holdCtr;
    public int show = 0;
    public int posrng;

    double leftSetVel;
    double leftSetPos;
    double leftFf;
    double leftPidout;
    public double leftPower;

    double rightSetVel;
    double rightSetPos;
    double rightFf;
    double rightPidout;
    public double rightPower;

    private double leftAccel;
    private double leftLastVel;
    private double rightAccel;
    private double rightLastVel;
    private double scanTime;

    public ElevatorSubsystem(CommandOpMode opMode) {

        myOpmode = opMode;

        leftElevatorMotor = new Motor(opMode.hardwareMap, "leftElevatorMotor", Motor.GoBILDA.RPM_312);
        leftElevatorMotor.setInverted(true);
        leftElevatorMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
        leftElevatorEncoder = leftElevatorMotor.encoder;
        leftElevatorEncoder.setDirection(Motor.Direction.FORWARD);
        leftElevatorEncoder.setDistancePerPulse(1 / ENCODER_COUNTS_PER_INCH);
        leftFeedForward = new ElevatorFeedforward(lks, lkg, lkv, lka);
        leftPidController = new ProfiledPIDController(lkp, lki, lkd, constraints);
        leftPidController.setTolerance(POSITION_TOLERANCE_INCHES);
        leftPidController.reset();


        rightElevatorMotor = new Motor(opMode.hardwareMap, "rightElevatorMotor", Motor.GoBILDA.RPM_312);
        rightElevatorMotor.setInverted(true);
        rightElevatorMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
        rightElevatorEncoder = rightElevatorMotor.encoder;
        rightElevatorEncoder.setDirection(Motor.Direction.FORWARD);
        rightElevatorEncoder.setDistancePerPulse(1 / ENCODER_COUNTS_PER_INCH);
        rightFeedForward = new ElevatorFeedforward(lks, lkg, lkv, lka);
        rightPidController = new ProfiledPIDController(lkp, lki, lkd, constraints);
        rightPidController.setTolerance(POSITION_TOLERANCE_INCHES);
        rightPidController.reset();

        constraints = new TrapezoidProfile.Constraints(MAX_VEL, MAX_ACCEL);


        resetElevatorEncoders();

        setTargetInches(HOME_POSITION);

        FtcDashboard dashboard = FtcDashboard.getInstance();

        telemetry = new MultipleTelemetry(opMode.telemetry, dashboard.getTelemetry());


        et = new ElapsedTime();
    }

    public static double round2dp(double number, int dp) {
        double temp = Math.pow(10, dp);
        double temp1 = Math.round(number * temp);
        return temp1 / temp;
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

    public void setTargetInches(double inches) {
        targetInches = inches;
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
