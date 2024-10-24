package org.firstinspires.ftc.teamcode;


import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

public final class Constants {


    private double mmToInch(double mm) {
        return mm * .03937;
    }

    public static final class RobotConstants {

        public static final double length = 17.75;
        public static final double height = 13;
        public static final double width = 17.75;
        public static final Vector2d manipulatorToRobotCenter = new Vector2d(12, 0);

    }

    public static final class FieldConstants {

        static final double length = 144;
        static final double width = 144;
        static final double tileSize = 24;
        static final double tileTeeth = .75;
        static final double distanceBetwwenSpikeMarks = 10;
    }

    public static final class SampleConstants {
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
        public static double MAX_VEL = MAX_IPS * .9;
        public static final double POSITION_TOLERANCE_INCHES = .25;
        public static final double AT_BACKBOARD_ANGLE_DISTANCE = 6;
        public static final double POSN_VEL = 4;
        public static final double POSN_ACCEL = 4;
        public static final double FORWARD_GAIN = .032;
        public static final double BATTERY_VOLTS = 12;
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
        //     */
        public static double kV = .0195;//12 volts/60 ips = .02 max
        public static double kA = 0.003;// 0.0005;
        public static double kStatic = 0.0;//0.08;


        //        public static double kV =  0.0140;
//        public static double kA = 0.0022;
//        public static double kStatic = 0.022;
        public static RevHubOrientationOnRobot.LogoFacingDirection LOGO_FACING_DIR =
                RevHubOrientationOnRobot.LogoFacingDirection.UP;

        //public static final double kV = BATTERY_VOLTS / MAX_IPM;//12/60 = .2 MAX THEORETICAL VALUE
        public static RevHubOrientationOnRobot.UsbFacingDirection USB_FACING_DIR =
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;
        public static double MAX_AUTO_TURN = 0.3;   //  Clip the turn speed to this max value (adjust for your robot)
        //  Set the GAIN constants to control the relationship between the measured position error, and how much power is
        //  applied to the drive motors to correct the error.
        //  Drive = Error * Gain    Make these values smaller for smoother control, or larger for a more aggressive response.
        public static double STRAFE_GAIN = 0.015;   //  Strafe Speed Control "Gain".  eg: Ramp up to 25% power at a 25 degree Yaw error.   (0.25 / 25.0)
        public static double TURN_GAIN = 0.01;   //  Turn Control "Gain".  eg: Ramp up to 25% power at a 25 degree error. (0.25 / 25.0)
        public static double MAX_AUTO_SPEED = 0.5;   //  Clip the approach speed to this max value (adjust for your robot)
        public static double MAX_AUTO_STRAFE = 0.5;   //  Clip the approach speed to this max value (adjust for your robot)
        public static double BACKBOARD_DISTANCE_OFFSET = 6;

        public static double getMotorVelocityF(double ticksPerSecond) {
            // see https://docs.google.com/document/d/1tyWrXDfMidwYyP_5H4mZyVgaEswhOC35gvdmP-V-5hA/edit#heading=h.61g9ixenznbx
            return 32767 / ticksPerSecond;
        }

        public static double encoderTicksToInches(double ticks) {
            return 1.05 * WHEEL_DIAMETER_INCH * Math.PI * GEARBOX_RATIO * ticks / ENCODER_COUNTS_PER_WHEEL_REV;

        }

        public static double rpmToVelocity(double rpm) {
            return rpm * GEARBOX_RATIO * 2 * Math.PI * WHEEL_DIAMETER_INCH / 2 / 60.0;
        }

    }

    public static final class ArmConstants {

        public static final double MAX_MOTOR_RPM = 223;
        public static final double ENCODER_COUNTS_PER_MOTOR_REV = 751.8;

        public static final double GEARING_RATIO = 1;
        public static final double MAX_MOTOR_RPSEC = MAX_MOTOR_RPM / 60;
        public static final double PULLEY_TEETH = 60;
        public static final double BELT_PITCH = 2 * .03937; //.07874
        public static final double DISTANCE_PER_MOTOR_REV = PULLEY_TEETH * BELT_PITCH;//4.5"approx
        public static final double ENCODER_COUNTS_PER_INCH = ENCODER_COUNTS_PER_MOTOR_REV / DISTANCE_PER_MOTOR_REV;// approx

        public static final double MAX_INCHES_PER_SECOND = DISTANCE_PER_MOTOR_REV * MAX_MOTOR_RPSEC * GEARING_RATIO * .8;//13


        public static final double POSITION_TOLERANCE_INCHES = .5;
        public static final double UPPER_POSITION_LIMIT = 27.00;
        public static final int LOWER_POSITION_LIMIT = -1;

        public static final double JOG_UP_POWER = +.5;
        public static final double JOG_DOWN_POWER = -.4;
        public static final double AUTO_DELIVER_POSITION = 7.5;
        public static final double HOME_POSITION = 1;


        public static final double MAX_VEL = 13;
        public static final double MAX_ACCEL = 13;

        public static final double TRAJ_VEL = 10;
        public static final double TRAJ_ACCEL = 10;


        public static double leftIntakeTiltClearAngle = 0;
        public static double leftIntakeTiltDownAngle = 1;
        public static double rightIntakeTiltClearAngle = 0;
        public static double rightIntakeTiltDownAngle = 1;

        public static double homeDistance = 1;
        public static double pickupDistance = 10;

        public static double bucketDistance = 2;
    }

    public static final class ElevatorConstants {

        public static final double MAX_MOTOR_RPM = 312;
        public static final double ENCODER_COUNTS_PER_MOTOR_REV = 537.5;
        public static final double MAX_MOTOR_RPSEC = MAX_MOTOR_RPM / 60;

        public static final double GEARING_RATIO = 1;
        public static final double PULLEY_TEETH = 60;
        public static final double BELT_PITCH = 2 * .03937; //.07874
        public static final double DISTANCE_PER_MOTOR_REV = PULLEY_TEETH * BELT_PITCH;//approx 4.9"
        public static final double ENCODER_COUNTS_PER_INCH = ENCODER_COUNTS_PER_MOTOR_REV / DISTANCE_PER_MOTOR_REV;// approx


        public static final double MAX_INCHES_PER_SECOND = DISTANCE_PER_MOTOR_REV * MAX_MOTOR_RPSEC * GEARING_RATIO;


        public static final double UPPER_POSITION_LIMIT = 20;
        public static final int LOWER_POSITION_LIMIT = 5;
        public static final double HOME_POSITION = 0;
        public static final double POSITION_TOLERANCE_INCHES = .1;


        public static double MAX_VEL = 30;
        public static double MAX_ACCEL = 30;
        public static double TRAJ_VEL = 20;
        public static double TRAJ_ACCEL = 20;
        public static double POSITION_Kg = 0;

        public static double bucketUprightAngle = 0;
        public static double bucketTippedAngle = 1;

        public static double elevatorDownHeight = 0;
        public static double elevatorLowerBasketHeight = 36;
        public static double elevatorUpperBasketHeight = 36;
        public static double elevatorSamplePickupHeight = 36;
        public static double elevatorSampleAboveLowPlaceHeight = 36;
        public static double elevatorSampleLowlaceHeight = 36;
        public static double elevatorSampleAboveTopPlaceHeight = 36;
        public static double elevatorSampleTopPlaceHeight = 36;


        public static double sampleClawOpenAngle = 0.3;
        public static double sampleClawClosedAngle = .5;

        public static double homePosition = 1;
        public static double lowerBasketDeliverPosition = 24;
        public static double upperBasketDeliverPosition = 44;

        public static double sampleApproachPosition = 30;
        public static double sampleDeliverPosition = 30;


    }


}

