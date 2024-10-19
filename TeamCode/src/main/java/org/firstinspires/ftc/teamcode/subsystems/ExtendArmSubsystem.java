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
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.wpilibcontroller.ProfiledPIDController;
import com.arcrobotics.ftclib.controller.wpilibcontroller.SimpleMotorFeedforward;;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.trajectory.TrapezoidProfile;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants;

@Config
public class ExtendArmSubsystem extends SubsystemBase {

    public static CRServo leftIntakeServo;
  //  public static CRServo rightIntakeServo;

    public static Servo leftTiltServo;
    public static Servo rightTiltServo;
    public static double targetInches;
    public static double ks = 0;//1% motor power
    public static double kv = .06;//per inch per second (max 17 ips )
    public static double ka = 0;
    public static double kp = 0.05;
    public static double ki = 0;
    public static double kd = 0;
    public static boolean TUNING = false;
    private final FtcDashboard dashboard;
    public Motor armMotor;
    public Motor.Encoder armEncoder;
    public double power;
    public ProfiledPIDController armController;
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
    private int targetSetCounter;


    public ExtendArmSubsystem(CommandOpMode opMode) {

        armMotor = new Motor(opMode.hardwareMap, "armMotor", Motor.GoBILDA.RPM_312);

        //
        leftIntakeServo = opMode.hardwareMap.get(CRServo.class, "leftInServo");


      //  rightIntakeServo = opMode.hardwareMap.get(CRServo.class, "rightInServo");

        leftTiltServo = opMode.hardwareMap.get(Servo.class, "leftTiltServo");
        rightTiltServo = opMode.hardwareMap.get(Servo.class, "rightTiltServo");

      //  leftIntakeServo.setInverted(false);
     //   rightIntakeServo.setInverted(true);

        leftTiltServo.setDirection(Servo.Direction.FORWARD);
        rightTiltServo.setDirection(Servo.Direction.REVERSE);

        dashboard = FtcDashboard.getInstance();

        telemetry = new MultipleTelemetry(opMode.telemetry, dashboard.getTelemetry());

        armMotor.setInverted(true);

        armMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        armEncoder = armMotor.encoder;

        armEncoder.setDirection(Motor.Direction.FORWARD);

        armEncoder.setDistancePerPulse(1 / Constants.ArmConstants.ENCODER_COUNTS_PER_INCH);//ENCODER_COUNTS_PER_INCH);

        armFF = new SimpleMotorFeedforward(ks, kv, ka);


        constraints = new TrapezoidProfile.Constraints(Constants.ArmConstants.TRAJ_VEL, Constants.ArmConstants.TRAJ_ACCEL);


        armController = new ProfiledPIDController(kp, ki, kd, constraints);


        armController.setTolerance(Constants.ArmConstants.POSITION_TOLERANCE_INCHES);

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

    @Override
    public void periodic() {
      //  showTelemetry();
        if (holdCtr >= 100) {
            scanTime = et.milliseconds() / holdCtr;
            holdCtr = 0;
            et.reset();
        }
        if (TUNING) tuning();
    }

    public void tuning() {
        setTargetInches(targetInches);
        if (armFF.kv != kv || armFF.ks != ks || armFF.ka != ka)
            setNewFFValues();
        if (armController.getP() != kp)
            armController.setP(kp);
        if (armController.getI() != ki)
            armController.setI(ki);
        if (armController.getD() != kd)
            armController.setD(kd);
    }

    public void setNewFFValues() {
        armFF = new SimpleMotorFeedforward(ks, kv, ka);
    }


    public void position() {
        // Retrieve the profiled setpoint for the next timestep. This setpoint moves


        armSetpoint = armController.getSetpoint();

        setVel = armSetpoint.velocity;
        setPos = armSetpoint.position;

        accel = (lastVel - setVel) * 50;

        pidout = armController.calculate(getPositionInches());

        ff = armFF.calculate(setVel, accel);

        armMotor.set(ff + pidout);

        lastVel = armSetpoint.velocity;
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


    public Action setAndWaitForAtTarget(double target) {
        return new Action() {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    targetSetCounter = 0;
                    setTargetInches(target);
                    initialized = true;
                }
                packet.put("target", target);
                packet.put("actual", getPositionInches());
                targetSetCounter++;
                return targetSetCounter >= 5 && atGoal();
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
        return new InstantAction(() -> leftIntakeServo.setPower(1));
    }

//    public Action runRightIntake(double speed) {
//        return new InstantAction(() -> rightIntakeServo.set(speed));
//    }

//    public Action runIntakeServos(double speed) {
//        return new ParallelAction(
//                runLeftIntake(speed));
//            //    runRightIntake(speed));
//    }

//    public Action reverseIntakeServos(double speed) {
//        return new ParallelAction(
//                runLeftIntake(speed));
//             //   runRightIntake(speed));
//    }

    public Action stopIntakeServos() {
        return new ParallelAction(
                new InstantAction(() -> leftIntakeServo.setPower(0)));
            //    new InstantAction(() -> rightIntakeServo.stop()));
    }

//    public Action goPickupSample() {
//        return new ParallelAction(
//                new InstantAction(() -> setTargetInches(Constants.ArmConstants.pickupDistance)),
//                tiltBothDown(),
//                runIntakeServos(1));
//    }
//
//    public Action deliverToBucket() {
//        return new SequentialAction(
//                tiltBothClear(),
//                setAndWaitForAtTarget(Constants.ArmConstants.bucketDistance),
//                runIntakeServos(-1));
//    }


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
