package org.firstinspires.ftc.teamcode.subsystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.wpilibcontroller.ProfiledPIDController;
import com.arcrobotics.ftclib.controller.wpilibcontroller.SimpleMotorFeedforward;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.trajectory.TrapezoidProfile;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.commands_actions.arm.PositionHoldArm;


@Config
public class ExtendArmSubsystem extends SubsystemBase {


    public static double targetInches;
    public static double ks = .12;//1% motor power
    public static double kv = .044;//max ips = 50 so 12/50 (assume 50%) = .06 volts per ips
    public static double ka = 0;
    public static double kp = 0.025;
    public static double ki = 0;
    public static double kd = 0;
    public static boolean TUNING = false;
    public static double OUT_POSITION_LIMIT = 17.00;
    public static double IN_POSITION_LIMIT = .5;
    public static double TRAJ_VEL = 10;
    public static double TRAJ_ACCEL = 10;
    public double lastTargetInches;
    public Motor leftArmMotor;
    public Motor rightArmMotor;
    public Motor.Encoder leftArmEncoder;
    public Motor.Encoder rightArmEncoder;
    public double power;
    public ProfiledPIDController leftArmController;
    public ProfiledPIDController rightArmController;
    public TrapezoidProfile.State leftArmSetpoint = new TrapezoidProfile.State();
    public TrapezoidProfile.State rightArmSetpoint = new TrapezoidProfile.State();

    public TrapezoidProfile.Constraints constraints;
    public SimpleMotorFeedforward armFF;

    public int holdCtr;
    public int showSelect = 0;
    public int tst;
    ElapsedTime et;
    double leftSetVel;
    double leftSetPos;
    double leftff;
    double leftpidout;
    double rightSetVel;
    double rightSetPos;
    double rightff;
    double rightpidout;
    private int inPositionCtr;
    private Telemetry telemetry;
    private double scanTime;
    private double leftAccel;
    private double rightAccel;
    private double lastRightVel;
    private double lastLeftVel;
    private int targetSetCounter;
    private double izonel = 5;
    private double izoner = 5;

    public ExtendArmSubsystem(CommandOpMode opMode) {

        constraints = new TrapezoidProfile.Constraints(TRAJ_VEL, TRAJ_ACCEL);

        leftArmMotor = new Motor(opMode.hardwareMap, "leftArmMotor");
        rightArmMotor = new Motor(opMode.hardwareMap, "rightArmMotor");


        leftArmMotor.setInverted(false);
        leftArmMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        leftArmEncoder = leftArmMotor.encoder;
        leftArmEncoder.setDirection(Motor.Direction.FORWARD);
        leftArmEncoder.setDistancePerPulse(1 / Constants.ExtendArmConstants.ENCODER_COUNTS_PER_INCH);//ENCODER_COUNTS_PER_INCH);
        armFF = new SimpleMotorFeedforward(ks, kv, ka);
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

        setDefaultCommand(new PositionHoldArm(this));

    }

    public static double round2dp(double number, int dp) {
        double temp = Math.pow(10, dp);
        double temp1 = Math.round(number * temp);
        return temp1 / temp;
    }

    public double getTargetInches() {
        return targetInches;
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

    @Override
    public void periodic() {
        if (showSelect == Constants.ShowTelemetryConstants.showArmCommon)
            showTelemetryCommon();
        if (showSelect == Constants.ShowTelemetryConstants.showArm1)
            showTelemetryLeft();
        if (showSelect == Constants.ShowTelemetryConstants.showArm2)
            showTelemetryRight();


        if (holdCtr >= 100) {
            scanTime = et.milliseconds() / holdCtr;
            holdCtr = 0;
            et.reset();
        }
        if (TUNING) tuning();
    }

    public void tuning() {
        if (lastTargetInches != targetInches) {
            setTargetInches(targetInches);
            lastTargetInches = targetInches;
        }
        if (armFF.kv != kv || armFF.ks != ks || armFF.ka != ka)
            setNewFFValues();

        if (leftArmController.getP() != kp)
            leftArmController.setP(kp);
        if (leftArmController.getI() != ki)
            leftArmController.setI(ki);
        if (leftArmController.getD() != kd)
            leftArmController.setD(kd);

        if (rightArmController.getP() != kp)
            rightArmController.setP(kp);
        if (rightArmController.getI() != ki)
            rightArmController.setI(ki);
        if (rightArmController.getD() != kd)
            rightArmController.setD(kd);
    }

    public void setNewFFValues() {
        armFF = new SimpleMotorFeedforward(ks, kv, ka);


    }

    public void position() {

        if (inPositionCtr != 3) inPositionCtr++;
        // Retrieve the profiled setpoint for the next timestep. This setpoint moves

        leftpidout = leftArmController.calculate(getLeftPositionInches());

//
//        if (Math.abs(leftArmController.getGoal().position - getLeftPositionInches()) < izonel)
//            leftArmController.setI(.0001);
//        else {
//            leftArmController.setI(0);
//        }


        leftArmSetpoint = leftArmController.getSetpoint();
        leftSetVel = leftArmSetpoint.velocity;
        leftSetPos = leftArmSetpoint.position;
        leftAccel = (lastLeftVel - leftSetVel) * 50;
        leftff = armFF.calculate(leftSetVel, leftAccel);
        leftArmMotor.set(leftff + leftpidout);
        lastLeftVel = leftSetVel;

        //right
        rightpidout = rightArmController.calculate(getRightPositionInches());

//        if (Math.abs(rightArmController.getGoal().position - getRightPositionInches()) < izonel)
//            rightArmController.setI(.0001);
//        else {
//            rightArmController.setI(0);
//        }


        rightArmSetpoint = rightArmController.getSetpoint();
        rightSetVel = rightArmSetpoint.velocity;
        rightSetPos = rightArmSetpoint.position;
        rightAccel = (lastRightVel - rightSetVel) * 50;
        rightff = armFF.calculate(rightSetVel, rightAccel);
        rightArmMotor.set(rightff + rightpidout);
        lastRightVel = rightSetVel;

        if (TUNING) {
            if (getLeftPositionInches() < IN_POSITION_LIMIT || getLeftPositionInches() > OUT_POSITION_LIMIT
                    || getRightPositionInches() < IN_POSITION_LIMIT || getRightPositionInches() > OUT_POSITION_LIMIT) {
                leftArmMotor.set(0);
                rightArmMotor.set(0);
            }
        }
    }

    public void resetEncoders() {
        leftArmEncoder.reset();
        rightArmEncoder.reset();
        setTargetInches(0);
    }

    public double getLeftPositionInches() {
        return round2dp(leftArmEncoder.getDistance(), 2);
    }

    public double getRightPositionInches() {
        return round2dp(rightArmEncoder.getDistance(), 2);
    }

    public double getLeftInchesPerSec() {
        return round2dp(leftArmEncoder.getRate(), 2);
    }

    public double getRightInchesPerSec() {
        return round2dp(rightArmEncoder.getRate(), 2);
    }


    public boolean leftInPosition() {
        return leftArmController.atSetpoint();
    }

    public double getLeftGoalPosition() {
        return leftArmController.getGoal().position;
    }

    public double getLeftPositionKp() {
        return leftArmController.getP();
    }

    public void setLeftPositionKp(double kp) {
        leftArmController.setP(kp);
    }

    public double getLeftPositionKi() {
        return leftArmController.getD();
    }

    public void setLeftPositionKi(double ki) {
        leftArmController.setI(ki);
    }

    public double getLeftPositionKd() {
        return leftArmController.getD();
    }

    public void setLeftPositionKd(double kd) {
        leftArmController.setD(kd);
    }

    public boolean rightInPosition() {
        return rightArmController.atSetpoint();
    }

    public double getRightGoalPosition() {
        return rightArmController.getGoal().position;
    }

    public double getRightPositionKp() {
        return rightArmController.getP();
    }

    public void setRightPositionKp(double kp) {
        rightArmController.setP(kp);
    }

    public double getRightPositionKi() {
        return rightArmController.getD();
    }

    public void setRightPositionKi(double ki) {
        rightArmController.setI(ki);
    }

    public double getRightPositionKd() {
        return rightArmController.getD();
    }

    public void setRightPositionKd(double kd) {
        rightArmController.setD(kd);
    }

    public void setTrapConstraints(double vel, double acc) {
        leftArmController.setConstraints(new TrapezoidProfile.Constraints(vel, acc));
        rightArmController.setConstraints(new TrapezoidProfile.Constraints(vel, acc));
    }

    public double getLeftPower() {
        return leftArmMotor.get();
    }

    public void setLeftPower(double power) {
        leftArmMotor.set(power);
    }

    public double getRightPower() {
        return rightArmMotor.get();
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

    public Action armToBucketAction() {
        return positionArm(Constants.ExtendArmConstants.bucketDistance);
    }


    public void showTelemetryCommon() {
        telemetry.addData("ArmCommon", showSelect);
        telemetry.addData("ArmLeftInches", getLeftPositionInches());
        telemetry.addData("ArmRightInches", getRightPositionInches());
        telemetry.addData("ArmLeftPower", round2dp(leftArmMotor.get(), 2));
        telemetry.addData("ArmRightInches", getRightPositionInches());
        telemetry.addData("ArmRightPower", round2dp(rightArmMotor.get(), 2));


        telemetry.update();
    }

    public void showTelemetryLeft() {
        telemetry.addData("ArmLeft", showSelect);
        telemetry.addData("Arm Tuning", atGoal());
        telemetry.addData("Arm FF", leftff);
        telemetry.addData("ArmLeftTargetInches", targetInches);
        telemetry.addData("ArmLeftInches", getLeftPositionInches());
        telemetry.addData("ArmLeftSetpointPos", round2dp(leftArmSetpoint.position, 2));
        telemetry.addData("ArmLeftSetpointVel", round2dp(leftArmSetpoint.velocity, 2));
        telemetry.addData("ArmLeftPosError", round2dp(leftArmController.getPositionError(), 2));
        telemetry.addData("ArmLeftVelError", round2dp(leftArmController.getVelocityError(), 2));
        telemetry.addData("ArmLeftVel", round2dp(getLeftInchesPerSec(), 2));
        telemetry.addData("ArmLeftPID", round2dp(leftpidout, 2));
        telemetry.addData("ArmLeftPower", round2dp(leftArmMotor.get(), 2));


        telemetry.update();

    }

    public void showTelemetryRight() {
        telemetry.addData("ArmRight", showSelect);
        telemetry.addData("Arm Tuning", atGoal());
        telemetry.addData("Arm FF", rightff);
        telemetry.addData("ArmRightTargetInches", targetInches);
        telemetry.addData("ArmRightInches", getRightPositionInches());
        telemetry.addData("ArmRightSetpointPos", round2dp(rightArmSetpoint.position, 2));
        telemetry.addData("ArmRightSetpointV", round2dp(rightArmSetpoint.velocity, 2));
        telemetry.addData("ArmRightPosError", round2dp(rightArmController.getPositionError(), 2));
        telemetry.addData("ArmRightVelError", round2dp(rightArmController.getVelocityError(), 2));
        telemetry.addData("ArmRightVel", round2dp(getRightInchesPerSec(), 2));
        telemetry.addData("ArmRightPID", round2dp(rightpidout, 2));
        telemetry.addData("ArmRightPower", round2dp(rightArmMotor.get(), 2));
        telemetry.update();

    }


}
