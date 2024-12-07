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

    public static final class SpecimenConstants {
        static final double length = 3.5;
        static final double side = 1.5;
    }

    public static final class DriveConstants {

        public static final boolean RUN_USING_ENCODER = false;


        public static final double MAX_MOTOR_RPM = 223;


        public static final double GEARBOX_RATIO = 1;
        public static final double WHEEL_DIAMETER_INCH = 3.5;

        public static final double WHEELBASE = 12;//distance between pairs of wheels on the same side of the robot

        public static final double TRACKWIDTH = 16.25;// from code sourec seemed to work better measured is 14.5;//lateral distance between pairs of wheels on different sides of the robot
        public static final double ENCODER_COUNTS_PER_WHEEL_REV = 751.8;//1:1 RATIO
        public static final double WHEEL_CIRCUMFERENCE_INCH = Math.PI * WHEEL_DIAMETER_INCH;//11.
        public static final double INCHES_PER_ENCODER_COUNT = WHEEL_CIRCUMFERENCE_INCH / ENCODER_COUNTS_PER_WHEEL_REV;//.0146
        public static final double MAX_IPM = MAX_MOTOR_RPM * WHEEL_CIRCUMFERENCE_INCH;// 312 * 11 = 3432 IPM

        public static final double MAX_IPS = MAX_IPM / 60;//57 IPS
        public static double MAX_VEL = MAX_IPS * .9;//51
        public static final double POSITION_TOLERANCE_INCHES = .25;
        public static final double AT_BACKBOARD_ANGLE_DISTANCE = 6;
        public static final double POSN_VEL = 4;
        public static final double POSN_ACCEL = 4;
        public static final double FORWARD_GAIN = .032;
        public static final double BATTERY_VOLTS = 12;
        public static double MAX_ACCEL = 40;
        public static double MAX_ANG_VEL = Math.toRadians(75);
        public static double MAX_ANG_ACCEL = Math.toRadians(120);

        public static double TRAJ_ANG_VEL = Math.toRadians(50);
        public static double TRAJ_ANG_ACCEL = Math.toRadians(50);
        public static PIDFCoefficients MOTOR_VELO_PID = new PIDFCoefficients(.1, 0, 0, 0);

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

    public static final class ExtendArmConstants {

        public static final double MAX_MOTOR_RPM = 6000;
        public static final double ENCODER_COUNTS_PER_MOTOR_REV = 28;

        public static final double GEARING_RATIO = 5;
        public static final double MAX_MOTOR_RPSEC = MAX_MOTOR_RPM / 60;
        public static final double MAX_USABLE_MOTOR_RPSEC = MAX_MOTOR_RPSEC * .8;//50
        public static final double PULLEY_TEETH = 60;//60;
        public static final double BELT_PITCH = 2 * .03937; //.07874
        public static final double DISTANCE_PER_MOTOR_REV = PULLEY_TEETH * BELT_PITCH / GEARING_RATIO;//1"
        public static final double ENCODER_COUNTS_PER_INCH = ENCODER_COUNTS_PER_MOTOR_REV / DISTANCE_PER_MOTOR_REV;//28

        public static final double MAX_INCHES_PER_SECOND = DISTANCE_PER_MOTOR_REV * MAX_USABLE_MOTOR_RPSEC / GEARING_RATIO;//50

        public static final double POSITION_TOLERANCE_INCHES = .5;


        public static double homeDistance = 1;

        public static double pickupDistance = 12;

        public static double bucketDistance = 0;
    }

    public static final class RotateArmConstants {

        public static double armDistanceOKTilt = 5;

    }

    public static final class ElevatorConstants {

        public static final double MAX_MOTOR_RPM = 1150;
        public static final double ENCODER_COUNTS_PER_MOTOR_REV = 145.1;

        public static final double GEARING_RATIO = 1;
        public static final double MAX_MOTOR_RPSEC = MAX_MOTOR_RPM / 60;//19
        public static final double MAX_USABLE_MOTOR_RPSEC = MAX_MOTOR_RPSEC * .8;//15

        public static final double PULLEY_TEETH = 60;//60;
        public static final double BELT_PITCH = 2 * .03937; //.07874
        public static final double DISTANCE_PER_MOTOR_REV = PULLEY_TEETH * BELT_PITCH / GEARING_RATIO;// appox 5"
        public static final double ENCODER_COUNTS_PER_INCH = ENCODER_COUNTS_PER_MOTOR_REV / DISTANCE_PER_MOTOR_REV;// approx 150

        public static final double MAX_INCHES_PER_SECOND = DISTANCE_PER_MOTOR_REV * MAX_USABLE_MOTOR_RPSEC;//75


        public static final double HOME_POSITION = 0;
        public static final double POSITION_TOLERANCE_INCHES = .1;

        public static double elevatorClearOfWall = 4;

        public static double elevatorLowerBasketHeight = 28;
        public static double elevatorUpperBasketHeight = 45;


        public static double elevatorSpecimenAboveLowPlaceHeight = 3;
        public static double elevatorSpecimenLowPlaceHeight = 1;

        public static double elevatorSpecimenAboveHighPlaceHeight = 15;
        public static double elevatorSpecimenAtHighPlaceHeight = 12.5;

    }

    public static final class ShowTelemetryConstants {

        public static final int minShowSelectCount = 0;
        public static final int maxShowSelectCount = 11;

        public static final int showNone = 0;
        public static final int showDrive1 = 1;
        public static final int showDrive2 = 2;
        public static final int showElevatorCommon = 3;
        public static final int showElevatorLeft = 4;
        public static final int showElevatorRight = 5;
        public static final int showArmCommon = 6;
        public static final int showArm1 = 7;
        public static final int showArm2 = 8;
        public static final int showRotateArm1 = 9;
        public static final int showColors = 10;
        public static final int showLimelight = 11;

    }


}

