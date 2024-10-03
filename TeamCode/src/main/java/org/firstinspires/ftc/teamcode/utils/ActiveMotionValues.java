package org.firstinspires.ftc.teamcode.utils;



import com.arcrobotics.ftclib.geometry.Pose2d;

import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

public class ActiveMotionValues {

    public static Pose2d getWaitPartnerClearPose() {
        return waitPartnerClearPose;
    }

    public static void setWaitPartnerClearPose(Pose2d clearPose) {
        waitPartnerClearPose = clearPose;
    }

    private static Pose2d waitPartnerClearPose = new Pose2d();

    public static double getActiveTagDistance() {
        return activeTagDistance;
    }

    public static void setActiveTagDistance(double tagDistance) {
        activeTagDistance = tagDistance;
    }

    public static double activeTagDistance = 0;

    public static double getActiveTagRange() {
        return activeTagRange;
    }

    public static void setActiveTagRange(double tagRange) {
        activeTagRange = tagRange;
    }
    public static double activeTagRange = 0;

    public static double getActiveTagBearing() {
        return activeTagBearing;
    }
    public static void setActiveTagBearing(double tagBearing) {
        activeTagBearing = tagBearing;
    }
    public static double activeTagBearing = 0;

    public static double getActiveTagYaw() {
        return activeTagYaw;
    }
    public static void setActiveTagYaw(double yaw) {
        activeTagYaw = yaw;
    }
    static double activeTagYaw = 0;

    public static double getTurnAngle() {
        return turnAngle;
    }

    public static void setTurnAngle(double angle) {
        turnAngle = angle;
    }

    public static double turnAngle;

    public static Pose2d getTagLineupPose() {
        return tagLineupPose;
    }

    public static void setTagLineupPose(Pose2d lineupPose) {
        tagLineupPose = lineupPose;
    }

    public static Pose2d tagLineupPose = new Pose2d();


    public static double getStopSecs() {
        return stopSecs;
    }

    public static void setStopSecs(double secs) {
        stopSecs = secs;
    }

    public static double stopSecs = .1;

    public static boolean getAprilTagSeen() {
        return aprilTagSeen;
    }

    public static void setAprilTagSeen(boolean tagSeen) {
        aprilTagSeen = tagSeen;
    }

    private static boolean aprilTagSeen = false;

    private static AprilTagDetection detection;

    public static AprilTagDetection getDetection() {
        return detection;
    }

    public static void setDetection(AprilTagDetection detect) {
        detection = detect;
    }


    public static Pose2d getFinalTagPose() {
        return finalTagPose;
    }

    public static void setFinalTagPose(Pose2d pose) {
        finalTagPose = pose;
    }

    private static Pose2d finalTagPose = new Pose2d();

    public static Pose2d getTrussSDLineUpPose() {
        return trussSDLineUpPose;
    }

    public static void setTrussSDLineUpPose(Pose2d lineUpPose) {
        trussSDLineUpPose = lineUpPose;
    }

    public static Pose2d getOptionStopPose() {
        return optionStopPose;
    }

    public static void setOptionStopPose(Pose2d opPose) {
        optionStopPose = opPose;
    }

    private static Pose2d trussSDLineUpPose = new Pose2d();
    private static Pose2d retractPose = new Pose2d();

    private static Pose2d optionStopPose = new Pose2d();


    private static Pose2d startPose = new Pose2d();


    public static Pose2d getPreTagPose() {
        return preTagPose;
    }

    public static void setPreTagPose(Pose2d pose) {
        preTagPose = pose;
    }

    private static Pose2d preTagPose = new Pose2d();

    public static Pose2d getStartPose() {
        return startPose;
    }

    public static void setStartPose(Pose2d pose) {
        startPose = pose;
    }


    private static boolean secondPixel = false;

    public static void setSecondPixel(boolean secPixel) {
        secondPixel = secPixel;
    }

    public static boolean getSecondPixel() {
        return secondPixel;
    }

    private static Pose2d advancePose = new Pose2d();


    public static void setAdvancePose(Pose2d pose) {
        advancePose = pose;
    }

    public static Pose2d getAdvancePose() {
        return advancePose;
    }


    public static Pose2d getRetractPose() {
        return retractPose;
    }

    public static void setRetractPose(Pose2d pose) {
        retractPose = pose;
    }

    public static Pose2d getDropOffPose() {
        return dropOffPose;
    }

    public static void setDropOffPose(Pose2d pose) {
        dropOffPose = pose;
    }

    private static Pose2d dropOffPose = new Pose2d();

    public static Pose2d getClearPose() {
        return clearPose;
    }

    public static void setClearPose(Pose2d pose) {
        clearPose = pose;
    }

    private static Pose2d clearPose = new Pose2d();


    public static boolean getRedAlliance() {
        return redAlliance;
    }

    public static void setRedAlliance(boolean redA) {
        redAlliance = redA;
    }

    public static boolean getBBStart() {
        return bbStart;
    }

    public static void setBBStart(boolean bbSt) {
        bbStart = bbSt;
    }

    public static int getLcrpos() {
        return lcrpos;
    }

    public static void setLcrpos(int lcr) {
        lcrpos = lcr;
    }

    public static int getActTag() {
        return actTag;
    }

    public static void setActTag(int act) {
        actTag = act;
    }

    private static Pose2d parkPose = new Pose2d();

    public static void setParkPose(Pose2d pose) {
        parkPose = pose;
    }

    public static Pose2d getParkPose() {
        return parkPose;
    }

    private static Pose2d preParkPose = new Pose2d();

    public static void setPreParkPose(Pose2d pose) {
        preParkPose = pose;
    }

    public static Pose2d getPreParkPose() {
        return preParkPose;
    }


    //auto running
    private static boolean redAlliance;

    private static boolean bbStart;


    private static int lcrpos = 0;

    private static int actTag = 0;

    private static boolean useStageDoor = false;

    private static boolean centerPark = false;

    public static void setCenterPark(boolean cPark) {
        centerPark = cPark;
    }

    public static boolean getCenterPark() {
        return centerPark;
    }

    private static boolean nearPark = false;

    public static void setNearPark(boolean nPark) {
        nearPark = nPark;
    }

    public static boolean getNearPark() {
        return nearPark;
    }

    public static void setUseStageDoor(boolean useStDoor) {
        useStageDoor = useStDoor;
    }

    public static boolean getUseStageDoor() {
        return useStageDoor;
    }

    private static Pose2d targetPose = new Pose2d();

    public static void setTargetPose(Pose2d target) {
        targetPose = target;
    }

    public static Pose2d getTargetPose() {
        return targetPose;
    }

    private static Pose2d endAutoPose = new Pose2d();

    public static void setEndAutoPose(Pose2d opt) {
        endAutoPose = opt;
    }

    public static Pose2d getEndAutoPose() {
        return endAutoPose;
    }


    public static Pose2d getClearToTurnPose() {
        return clearToTurnPose;
    }

    public static void setClearToTurnPose(Pose2d clearTurnPose) {
        ActiveMotionValues.clearToTurnPose = clearTurnPose;
    }

    private static Pose2d clearToTurnPose = new Pose2d();

}
