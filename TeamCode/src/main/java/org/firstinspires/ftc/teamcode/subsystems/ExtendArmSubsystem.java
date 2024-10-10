package org.firstinspires.ftc.teamcode.subsystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.wpilibcontroller.ProfiledPIDController;
import com.arcrobotics.ftclib.controller.wpilibcontroller.SimpleMotorFeedforward;
import com.arcrobotics.ftclib.hardware.motors.CRServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.trajectory.TrapezoidProfile;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants;

@Config
public class ExtendArmSubsystem extends SubsystemBase {

    public static CRServo leftIntakeServo;
    public static CRServo rightIntakeServo;

    public static Servo leftTiltServo;
    public static Servo rightTiltServo;
    public static double targetInches;
    public static double ks = 0;//1% motor power
    public static double kv = .015;//per inch per second (max 17 ips )
    public static double ka = 0;
    public static double kp = 0.01;
    public static double ki = 0;
    public static double kd = 0;
    public static double kP = 1.5;
    public static double kI = 0;
    public static double kD = 0;
    private final FtcDashboard dashboard;
    public Motor armMotor;
    public Motor.Encoder armEncoder;
    public double power;
    public ProfiledPIDController armController = null;
    public TrapezoidProfile.State armGoal = new TrapezoidProfile.State();
    public TrapezoidProfile.State armSetpoint = new TrapezoidProfile.State();
    public TrapezoidProfile.Constraints constraints;
    public SimpleMotorFeedforward armFF;
    public int holdCtr;
    public int show = 0;
    ElapsedTime et;
    double setVel;
    double setPos;
    double ff;
    double pidout;
    private int armDeliverLevel;
    private Telemetry telemetry;
    private double scanTime;
    private double accel;
    private double lastVel;


    public ExtendArmSubsystem(CommandOpMode opMode) {

        armMotor = new Motor(opMode.hardwareMap, "armMotor", Motor.GoBILDA.RPM_312);

        leftIntakeServo = opMode.hardwareMap.get(CRServo.class, "leftIntakeServo");
        rightIntakeServo = opMode.hardwareMap.get(CRServo.class, "rightIntakeServo");

        leftTiltServo = opMode.hardwareMap.get(Servo.class, "leftTiltServo");
        rightTiltServo = opMode.hardwareMap.get(Servo.class, "rightTiltServo");

        leftIntakeServo.setInverted(false);
        rightIntakeServo.setInverted(true);

        leftTiltServo.setDirection(Servo.Direction.FORWARD);
        rightTiltServo.setDirection(Servo.Direction.REVERSE);

        dashboard = FtcDashboard.getInstance();

        telemetry = new MultipleTelemetry(opMode.telemetry, dashboard.getTelemetry());

        armMotor.setInverted(true);

        armMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        armEncoder = armMotor.encoder;

        armEncoder.setDirection(Motor.Direction.FORWARD);

        armEncoder.setDistancePerPulse(1/Constants.ArmConstants.ENCODER_COUNTS_PER_INCH);//ENCODER_COUNTS_PER_INCH);

        armFF = new SimpleMotorFeedforward(ks, kv, ka);


        constraints = new TrapezoidProfile.Constraints(Constants.ArmConstants.TRAJ_VEL, Constants.ArmConstants.TRAJ_ACCEL);


        armController = new ProfiledPIDController(kp, ki, kd, constraints);


        armController.setTolerance(1);

        armController.reset(0);

        resetEncoder();

        // setDefaultCommand(new PositionHoldArm(this));

        FtcDashboard dashboard = FtcDashboard.getInstance();

        telemetry = new MultipleTelemetry(opMode.telemetry, dashboard.getTelemetry());


        et = new ElapsedTime();
    }

    public static double getTargetInches() {
        return targetInches;
    }

    public void setTargetInches(double target) {
        targetInches = target;
        armController.setGoal(targetInches);
    }

//    public Action setTarget(double target) {
//        return new InstantAction(() -> setTargetInches(target));
//    }

    @Override

    public void periodic() {
        if (holdCtr >= 100) {
            scanTime = et.milliseconds() / holdCtr;
            holdCtr = 0;
            et.reset();
        }
    }

    public void position() {
        // Retrieve the profiled setpoint for the next timestep. This setpoint moves

        pidout = armController.calculate(getPositionInches());

        armSetpoint = armController.getSetpoint();

        setVel = armSetpoint.velocity;
        setPos = armSetpoint.position;
        ff = armFF.calculate(setVel, accel);
        armMotor.set(ff + pidout);
    }

    public void resetEncoder() {
        armEncoder.reset();
    }

    public double getPositionInches() {
        return armEncoder.getDistance();
    }

    public boolean inPosition() {
        return armController.atSetpoint();
    }

    public double getGoalPosition() {
        return armController.getGoal().position;
    }

    public double getPositionKp() {
        return armController.getP();
    }

    public void setPositionKp(double kp) {
        armController.setP(kp);
    }

    public double getPositionKi() {
        return armController.getD();
    }

    public void setPositionKi(double ki) {
        armController.setI(ki);
    }

    public double getPositionKd() {
        return armController.getD();
    }

    public void setPositionKd(double kd) {
        armController.setD(kd);
    }

    public void setTrapConstraints(double vel, double acc) {
        armController.setConstraints(new TrapezoidProfile.Constraints(vel, acc));
    }

    public double getPower() {
        return armMotor.get();
    }

    public void setPower(double power) {
        armMotor.set(power);
    }

    public boolean atGoal() {
        return armController.atGoal();
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

    public Action tiltBothDown() {
        return new ParallelAction(
                new InstantAction(() -> leftTiltServo.setPosition(Constants.ArmConstants.leftIntakeTiltDownAngle)),
                new InstantAction(() -> rightTiltServo.setPosition(Constants.ArmConstants.rightIntakeTiltDownAngle)));
    }

    public Action tiltBothClear() {
        return new ParallelAction(
                new InstantAction(() -> leftTiltServo.setPosition(Constants.ArmConstants.leftIntakeTiltClearAngle)),
                new InstantAction(() -> rightTiltServo.setPosition(Constants.ArmConstants.rightIntakeTiltClearAngle)));
    }

    public Action runLeftIntake() {
        return new InstantAction(() -> leftIntakeServo.set(1));
    }

    public Action runRightIntake() {
        return new InstantAction(() -> rightIntakeServo.set(1));
    }

    public Action runIntakeServos() {
        return new ParallelAction(
                runLeftIntake(),
                runRightIntake());
    }

    public Action stopIntakeServos() {
        return new ParallelAction(
                new InstantAction(() -> leftIntakeServo.stop()),
                new InstantAction(() -> rightIntakeServo.stop()));
    }

    public Action goPickupSpecimen() {
        return new ParallelAction(
                setTarget(Constants.ArmConstants.pickupDistance),
                tiltBothDown(),
                runIntakeServos());
    }


    public void showTelemetry() {
        telemetry.addData("Arm", show);
        telemetry.addData("HoldRng", holdCtr);
        telemetry.addData("Scantime", scanTime);

        telemetry.addData("ArmInches", getPositionInches());
        telemetry.addData("GoalInches", getGoalPosition());
        telemetry.addData("TargetInches", targetInches);
        telemetry.addData("ArmPower", armMotor.get());


        telemetry.update();

    }
}
