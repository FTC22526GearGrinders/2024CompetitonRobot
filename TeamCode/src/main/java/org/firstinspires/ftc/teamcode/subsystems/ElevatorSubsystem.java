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
    public static double ks = 0.;//1% motor power
    public static double kg = 0;
    public static double kv = .04;//per inch per second (max 17 ips )
    public static double ka = 0;
    public static double kp = 0.01;
    public static double ki = 0;
    public static double kd = 0;




    private final Telemetry telemetry;
    public Motor elevatorMotor;
    public Motor.Encoder elevatorEncoder;

    public double targetInches;
    public double power;
    public ProfiledPIDController pidController;
    public TrapezoidProfile.State goal = new TrapezoidProfile.State();
    public TrapezoidProfile.State setpoint = new TrapezoidProfile.State();


    public int holdCtr;
    public int show = 0;
    public ElevatorFeedforward FF;

    public int posrng;
    public TrapezoidProfile.Constraints constraints;
    ElapsedTime et;
    CommandOpMode myOpmode;

    double setVel;
    double setPos;
    double ff;

    double pidout;


    private double scanTime;
    private double accel;

    private double lastVel;


    public ElevatorSubsystem(CommandOpMode opMode) {

        myOpmode = opMode;
        elevatorMotor = new Motor(opMode.hardwareMap, "elevatorMotor", Motor.GoBILDA.RPM_312);

        elevatorMotor.setInverted(true);
        elevatorMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);

        elevatorEncoder = elevatorMotor.encoder;

        elevatorEncoder.setDirection(Motor.Direction.FORWARD);

        elevatorEncoder.setDistancePerPulse(1 / ENCODER_COUNTS_PER_INCH);


        FF = new ElevatorFeedforward(ks, kg, kv,ka);


        constraints = new TrapezoidProfile.Constraints(MAX_VEL, MAX_ACCEL);


        pidController = new ProfiledPIDController(kp, ki, kd, constraints);

        pidController.setTolerance(POSITION_TOLERANCE_INCHES);

        pidController.reset();


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
        showTelemetry();
        //   }
        if (holdCtr >= 100) {
            scanTime = et.milliseconds() / holdCtr;
            holdCtr = 0;
            et.reset();
        }
    }

    public double getVoltsPerIPS() {
        return (power - .4) / getVelocityInPerSec();
    }

    public void position() {

        posrng++;


        // Retrieve the profiled setpoint for the next timestep. This setpoint moves

        pidout = pidController.calculate(getPositionInches());

        setpoint = pidController.getSetpoint();

        setVel = setpoint.velocity;
        setPos = setpoint.position;

      accel   = (setVel - lastVel) * 50;

        ff = FF.calculate(setVel,accel );


        elevatorMotor.set(ff + pidout);


        lastVel = setVel;

    }

    public void setTargetInches(double inches) {
        targetInches = inches;
    }

//    public double getPositionKp() {
//        return pidController.getP();
//    }
//
    public void setPositionKp() {
       pidController.setP(kp);
    }
//
//    public double getPositionKi() {
//        return pidController.getD();
//    }
//
   public void setPositionKi() {
        pidController.setI(ki);
    }
//
//    public double getPositionKd() {
//        return pidController.getD();
//    }
//
    public void setPositionKd() {
        pidController.setD(kd);
    }
//
//    public double getFFKS() {
//        return FF.ks;
//    }

//    public double getFFKg() {
//        return FF.kg;
//    }
//
//    public double getFFKV() {
//        return FF.kv;
//    }
//
//    public double getFFKa() {
//        return FF.ka;
//    }

    public void setNewFFValues() {
        {
            FF = new ElevatorFeedforward(ks, kg, kv, ka);
        }


    }

    public void resetElevatorEncoders() {
        elevatorEncoder.reset();

    }

    public double getPositionInches() {
        return round2dp(elevatorEncoder.getDistance(), 2);
    }


    public double getVelocityInPerSec() {
        return round2dp(elevatorEncoder.getCorrectedVelocity() / 60, 2);
    }


    public boolean inPosition() {
        return pidController.atGoal();
    }

    public double getPower() {
        return elevatorMotor.get();
    }

    public void setPower(double power) {
        elevatorMotor.set(power);
        // rightElevatorMotor.set(power);
    }

    public void showTelemetry() {
        telemetry.addData("Elevator", show);

        telemetry.addData("HoldRng", posrng);

        telemetry.addData("VoltsPerIPS", getVoltsPerIPS());

        telemetry.addData("PositionInches", getPositionInches());

        telemetry.addData("ElevatorGoal", pidController.getGoal().position);


        telemetry.addData("Power", elevatorMotor.get());
        telemetry.addData("PosErr", pidController.getPositionError());
        telemetry.addData("FF", ff);
        telemetry.addData("PIDout", pidout);
        telemetry.addData("SetVel", setVel);
        telemetry.addData("SetPos", setPos);

        telemetry.update();

    }
}
