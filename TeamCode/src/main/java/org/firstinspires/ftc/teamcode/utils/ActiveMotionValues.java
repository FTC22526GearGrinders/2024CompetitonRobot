package org.firstinspires.ftc.teamcode.utils;


import com.arcrobotics.ftclib.geometry.Pose2d;

public class ActiveMotionValues {

    private static boolean basket = false;
    //auto running
    private static boolean redAlliance;

    public static boolean getBasket() {
        return basket;
    }

    public static void setBasket(boolean val) {
        basket = val;
    }

    public static boolean getRedAlliance() {
        return redAlliance;
    }

    public static void setRedAlliance(boolean val) {
        redAlliance = val;
    }

    public static int getLcrpos() {
        return 1;
    }

    public static void setSampleDropOffPose(Pose2d pose2d) {
    }

    public static void setRetractPose(Pose2d pose2d) {

    }
}
