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
    public static double kv = .02;//100 rps through 5: gear = 20 ipm max so if kv is to make up 60% .6/20 = .03
    public static double ka = 0;
    public static double kp = 0.2;
    public static double ki = 0;
    public static double kd = 0;
    public static boolean TUNING = false;
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
    public SimpleMotorFeedforward leftArmFF;
    public SimpleMotorFeedforward rightArmFF;
    public int holdCtr;
    public int show = 0;
    public int tst;
    public boolean armTest;
    ElapsedTime et;
    double leftSetVel;
    double leftSetPos;
    double leftff;
    double leftpidout;
    double rightSetVel;
    double rightSetPos;
    double rightff;
    double rightpidout;
    private int inPositiobCtr;
    private Telemetry telemetry;
    private double scanTime;
    private double leftAccel;
    private double rightAccel;
    private double lastRightVel;
    private double lastLeftVel;
    private int targetSetCounter;

    public ExtendArmSubsystem(CommandOpMode opMode) {

        constraints = new TrapezoidProfile.Constraints(Constants.ExtendArmConstants.TRAJ_VEL, Constants.ExtendArmConstants.TRAJ_ACCEL);

        leftArmMotor = new Motor(opMode.hardwareMap, "leftArmMotor");
        rightArmMotor = new Motor(opMode.hardwareMap, "rightArmMotor");


        leftArmMotor.setInverted(false);
        leftArmMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        leftArmEncoder = leftArmMotor.encoder;
        leftArmEncoder.setDirection(Motor.Direction.FORWARD);
        leftArmEncoder.setDistancePerPulse(1 / Constants.ExtendArmConstants.ENCODER_COUNTS_PER_INCH);//ENCODER_COUNTS_PER_INCH);
        leftArmFF = new SimpleMotorFeedforward(ks, kv, ka);
        leftArmController = new ProfiledPIDController(kp, ki, kd, constraints);
        leftArmController.setTolerance(Constants.ExtendArmConstants.POSITION_TOLERANCE_INCHES);
        leftArmController.reset(0);

        rightArmMotor.setInverted(false);
        rightArmMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        rightArmEncoder = rightArmMotor.encoder;
        rightArmEncoder.setDirection(Motor.Direction.FORWARD);
        rightArmEncoder.setDistancePerPulse(1 / Constants.ExtendArmConstants.ENCODER_COUNTS_PER_INCH);//ENCODER_COUNTS_PER_INCH);
        rightArmFF = new SimpleMotorFeedforward(ks, kv, ka);
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

    public void setTargetInches(double target) {
        targetInches = target;

        leftArmController.setGoal(targetInches);
        rightArmController.setGoal(targetInches);
    }

    public Action positionArm(double target) {
        return new Action() {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    inPositiobCtr = 0;
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
        showTelemetry();
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
        if (leftArmFF.kv != kv || leftArmFF.ks != ks || leftArmFF.ka != ka)
            setNewFFValues();
        if (leftArmController.getP() != kp)
            leftArmController.setP(kp);
        if (leftArmController.getI() != ki)
            leftArmController.setI(ki);
        if (leftArmController.getD() != kd)
            leftArmController.setD(kd);

        if (rightArmFF.kv != kv || rightArmFF.ks != ks || rightArmFF.ka != ka)
            setNewFFValues();
        if (rightArmController.getP() != kp)
            rightArmController.setP(kp);
        if (rightArmController.getI() != ki)
            rightArmController.setI(ki);
        if (rightArmController.getD() != kd)
            rightArmController.setD(kd);
    }

    public void setNewFFValues() {
        leftArmFF = new SimpleMotorFeedforward(ks, kv, ka);
        rightArmFF = new SimpleMotorFeedforward(ks, kv, ka);

    }

    public void position() {

        if (inPositiobCtr != 3) inPositiobCtr++;
        // Retrieve the profiled setpoint for the next timestep. This setpoint moves

        leftpidout = leftArmController.calculate(getLeftPositionInches());
        leftArmSetpoint = leftArmController.getSetpoint();
        leftSetVel = leftArmSetpoint.velocity;
        leftSetPos = leftArmSetpoint.position;
        leftAccel = (lastLeftVel - leftSetVel) * 50;
        leftff = leftArmFF.calculate(leftSetVel, leftAccel);
        leftArmMotor.set(leftff + leftpidout);
//        leftArmMotor.set(leftff);
        // leftArmMotor.set(leftpidout);
        lastLeftVel = leftSetVel;

        //right
        rightpidout = rightArmController.calculate(getRightPositionInches());
        rightArmSetpoint = rightArmController.getSetpoint();
        rightSetVel = rightArmSetpoint.velocity;
        rightSetPos = rightArmSetpoint.position;
        rightAccel = (lastRightVel - rightSetVel) * 50;
        rightff = rightArmFF.calculate(rightSetVel, rightAccel);
        rightArmMotor.set(rightff + rightpidout);
        //  rightArmMotor.set(rightff);
        //   rightArmMotor.set(rightpidout);
        lastRightVel = rightSetVel;

        if (TUNING) {
            if (getLeftPositionInches() < 0 || getLeftPositionInches() > 15
                    || getRightPositionInches() < 0 || getRightPositionInches() > 15) {
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
        return inPositiobCtr == 3 && leftArmController.atGoal();
    }

    public boolean atRightGoal() {
        return inPositiobCtr == 3 && rightArmController.atGoal();
    }

    public boolean atGoal() {
        return atLeftGoal() && atRightGoal();
    }

    public double getLeftVelocity() {
        return leftArmEncoder.getCorrectedVelocity() / 60;
    }

    public double getRightVelocity() {
        return rightArmEncoder.getCorrectedVelocity() / 60;
    }

    public Action armToPickupAction() {
        return positionArm(Constants.ExtendArmConstants.pickupDistance);
    }

    public Action armToBucketAction() {
        return positionArm(Constants.ExtendArmConstants.bucketDistance);
    }

    public void showTelemetry() {
        telemetry.addData("ArmTest", armTest);
        telemetry.addData("Arm Tuning", atGoal());
        telemetry.addData("Arm FF", leftff);
        telemetry.addData("ArmLeftTargetInches", targetInches);
        telemetry.addData("ArmLeftInches", getLeftPositionInches());
        telemetry.addData("ArmLeftSetpointP", round2dp(leftArmSetpoint.position, 2));
        telemetry.addData("ArmLeftSetpointV", round2dp(leftArmSetpoint.velocity, 2));
        telemetry.addData("ArmLeftPosError", round2dp(leftArmController.getPositionError(), 2));
        telemetry.addData("ArmLeftVelError", round2dp(leftArmController.getVelocityError(), 2));


        telemetry.addData("ArmLeftVel", round2dp(getLeftVelocity(), 2));

        telemetry.addData("ArmLeftPID", round2dp(leftpidout, 2));

        telemetry.addData("ArmLeftPower", round2dp(leftArmMotor.get(), 2));


        telemetry.update();

    }


}
