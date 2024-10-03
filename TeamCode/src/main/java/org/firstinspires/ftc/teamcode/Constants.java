package org.firstinspires.ftc.teamcode;


import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

public final class Constants {

    public static final class RobotConstants {

        public static final double length = 17.75;
        static final double height = 13;
        static final double width = 17.75;
        static final Vector2d manipulatorToRobotCenter = new Vector2d(12, 0);

    }

    public static final class FieldConstants {

        static final double length = 144;
        static final double width = 144;
        static final double tileSize = 24;
        static final double tileTeeth = .75;
        static final double distanceBetwwenSpikeMarks = 10;
    }

public static final class SampleConstants{
        static final double length = 3.5;
        static final double side = 1.5;
}


    public static final class DriveConstants {

        public static final boolean RUN_USING_ENCODER = false;


        public static final double MAX_MOTOR_RPM = 223;


        public static final double GEARBOX_RATIO = 1;
        public static final double WHEEL_DIAMETER_INCH = 5.5;

        public static final double WHEELBASE = 12;//distance between pairs of wheels on the same side of the robot

        public static final double TRACKWIDTH = 16.25;// from code sourec seemed to work better measured is 14.5;//lateral distance between pairs of wheels on different sides of the robot
        public static final double ENCODER_COUNTS_PER_WHEEL_REV = 751.8;//1:1 RATIO
        public static final double WHEEL_CIRCUMFERENCE_INCH = Math.PI * WHEEL_DIAMETER_INCH;//17.27
        public static final double INCHES_PER_ENCODER_COUNT = WHEEL_CIRCUMFERENCE_INCH / ENCODER_COUNTS_PER_WHEEL_REV;//.0234
        public static final double MAX_IPM = MAX_MOTOR_RPM * WHEEL_CIRCUMFERENCE_INCH;// 312 *12.57 = 3900 IPM

        public static final double MAX_IPS = MAX_IPM / 60;//65 IPS
        public static final double POSITION_TOLERANCE_INCHES = .25;
        public static final double AT_BACKBOARD_ANGLE_DISTANCE = 6;
        public static final double POSN_VEL = 4 ;
        public static final double POSN_ACCEL = 4 ;
        public static final double FORWARD_GAIN = .032;

        public static double MAX_VEL = MAX_IPS * .9;
        public static double MAX_ACCEL = 40;
        public static double MAX_ANG_VEL = Math.toRadians(75);
        public static double MAX_ANG_ACCEL = Math.toRadians(120);
        public static double TRAJ_VEL = 50;
        public static double TRAJ_ACCEL = 50;
        public static double TRAJ_ANG_VEL = Math.toRadians(50);
        public static double TRAJ_ANG_ACCEL = Math.toRadians(50);

        public static PIDFCoefficients MOTOR_VELO_PID = new PIDFCoefficients(.1, 0, 0, 0);

        public static double kP = .02;
        public static double kI = 0;
        public static double kD = 0;

        public static double getMotorVelocityF(double ticksPerSecond) {
            // see https://docs.google.com/document/d/1tyWrXDfMidwYyP_5H4mZyVgaEswhOC35gvdmP-V-5hA/edit#heading=h.61g9ixenznbx
            return 32767 / ticksPerSecond;
        }

        //     */
        public static double kV = .0195;//12 volts/60 ips = .02 max
        public static double kA = 0.003;// 0.0005;
        public static double kStatic = 0.0;//0.08;


//        public static double kV =  0.0140;
//        public static double kA = 0.0022;
//        public static double kStatic = 0.022;

        public static final double BATTERY_VOLTS = 12;

        //public static final double kV = BATTERY_VOLTS / MAX_IPM;//12/60 = .2 MAX THEORETICAL VALUE


        public static RevHubOrientationOnRobot.LogoFacingDirection LOGO_FACING_DIR =
                RevHubOrientationOnRobot.LogoFacingDirection.UP;
        public static RevHubOrientationOnRobot.UsbFacingDirection USB_FACING_DIR =
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;


        public static double encoderTicksToInches(double ticks) {
            return 1.05 * WHEEL_DIAMETER_INCH * Math.PI * GEARBOX_RATIO * ticks / ENCODER_COUNTS_PER_WHEEL_REV;

        }

        public static double rpmToVelocity(double rpm) {
            return rpm * GEARBOX_RATIO * 2 * Math.PI * WHEEL_DIAMETER_INCH / 2 / 60.0;
        }

        public static double MAX_AUTO_TURN = 0.3;   //  Clip the turn speed to this max value (adjust for your robot)
        //  Set the GAIN constants to control the relationship between the measured position error, and how much power is
        //  applied to the drive motors to correct the error.
        //  Drive = Error * Gain    Make these values smaller for smoother control, or larger for a more aggressive response.
        public static double STRAFE_GAIN = 0.015;   //  Strafe Speed Control "Gain".  eg: Ramp up to 25% power at a 25 degree Yaw error.   (0.25 / 25.0)
        public static double TURN_GAIN = 0.01;   //  Turn Control "Gain".  eg: Ramp up to 25% power at a 25 degree error. (0.25 / 25.0)
        public static double MAX_AUTO_SPEED = 0.5;   //  Clip the approach speed to this max value (adjust for your robot)
        public static double MAX_AUTO_STRAFE = 0.5;   //  Clip the approach speed to this max value (adjust for your robot)

        public static double BACKBOARD_DISTANCE_OFFSET = 6;

    }

    public static final class PixelHandlerConstants {
        public static final double DROP_OPEN_POSITION = 0;
        public static final double DROP_CLOSED_POSITION = .7;

        public enum LeftGripperSet {
            WIDE_OPEN(.6),
            OPEN(.5),//open position
            MID(.7),//mid
            CLOSED(.2);//closed

            public final double position;

            LeftGripperSet(double position) {
                this.position = position;
            }
        }


        public enum RightGripperSet {
            WIDE_OPEN(.35),
            OPEN(.4),
            MID(.5),
            CLOSED(.65);

            public final double position;

            RightGripperSet(double position) {
                this.position = position;
            }
        }


        public enum TurnGripperSet {
            PICKUP(.2),
            MID(.65),
            DELIVER(0);

            public final double position;

            TurnGripperSet(double position) {
                this.position = position;
            }
        }

        public enum FlipGripperSet {
            PICKUP(0.195),
            LEFT_DOWN(.33),
            DELIVER(.427),

            RIGHT_DOWN(0.06);

            public final double position;

            FlipGripperSet(double position) {
                this.position = position;
            }
        }
    }

    public enum TurnGripperJogSet {
        LO(.001),

        MED(.01),

        HI(.1);

        public final double increment;

        TurnGripperJogSet(double increment) {
            this.increment = increment;
        }
    }


    public static final class ArmConstants {

        public static final double MAX_MOTOR_RPM = 435;

        public static final double MAX_MOTOR_RPSEC = 435 / 60;


        public static final double ENCODER_COUNTS_PER_MOTOR_REV = 384.5;

        public static final double GEARING_RATIO = 4.72;// motor inches per rev

        public static final double ENCODER_COUNTS_PER_INCH = ENCODER_COUNTS_PER_MOTOR_REV / GEARING_RATIO;
        public static final double MAX_INCHES_PER_SECOND = MAX_MOTOR_RPSEC * GEARING_RATIO;
        public static final double POSITION_TOLERANCE_INCHES = .5;
        public static final double UPPER_POSITION_LIMIT = 27.00;
        public static final int LOWER_POSITION_LIMIT = -1;

        public static final double JOG_UP_POWER = +.5;
        public static final double JOG_DOWN_POWER = -.4;
        public static final double AUTO_DELIVER_POSITION = 7.5;
        public static final double HOME_POSITION = 1;
        public static final double MAX_VEL = 30;
        public static final double MAX_ACCEL = 30;
        public static double[] armPositionInches = {0,0.5, 5.8, 9, 13.6, 14.3, 16.8, 19};


        public static double kP = 1.5;
        public static double kI = 0;
        public static double kD = 0;

        public static double POSITION_Kg = 0;

    }

    public static final class ClimberConstants {

        public static final double MAX_MOTOR_RPM = 312;

        public static final double MAX_MOTOR_RPSEC = 312 / 60;


        public static final double ENCODER_COUNTS_PER_MOTOR_REV = 537.7;

        public static final double GEARING_RATIO = 225 / 16;// motor revs per inch

        public static final double ENCODER_COUNTS_PER_INCH = 6.67 * ENCODER_COUNTS_PER_MOTOR_REV / GEARING_RATIO;

        public static final double MOTOR_REVS_PER_INCH = GEARING_RATIO;
        public static final double MAX_INCHES_PER_SECOND = MAX_MOTOR_RPSEC / GEARING_RATIO;
        public static final double POSITION_TOLERANCE_INCHES = .25;
        public static final double UPPER_POSITION_LIMIT = 18.00;
        public static final int LOWER_POSITION_LIMIT = 0;

        public static final double JOG_UP_POWER = +.5;

        public static final double JOG_DOWN_POWER = -.4;
        public static final double AUTO_DELIVER_POSITION = 7.5;
        public static final double HOME_POSITION = 1;
        public static final double MAX_VEL = 4;
        public static final double MAX_ACCEL = 2;

        public static double kP = .5;
        public static double kI = 0;
        public static double kD = 0;

        public static double POSITION_Kg = 0;

        public static double CLEAR_BAR_HEIGHT = 16;

        public static double RAISe_ROBOT_HEIGHT = 10;


    }

    public static final class CatapultConstants {

        public static final double CATAPULT_LOCK_POSITION = .0;
        public static final double CATAPULT_RELEASE_POSITION = .10;

        public static final double CATAPULT_MOTOR_POWER = .5;
        public static final long MOTOR_RUN_MILLISECONDS = 1000 ;
    }


}

