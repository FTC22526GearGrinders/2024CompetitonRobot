package com.example.meepmeeptesting;


import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;


public final class FieldConstantsBlue {

    /*
     *
     *
     * */

    public static Pose2d basketSideStartPose = flipPoseBlue(FieldConstantsRed.basketSideStartPose);
    public static Pose2d basketDeliverPose = flipPoseBlue(FieldConstantsRed.basketDeliverPose);

    public static Pose2d innerYellowPickupPose = flipPoseBlue(FieldConstantsRed.innerYellowPickupPose);
    public static Pose2d midYellowPickupPose = flipPoseBlue(FieldConstantsRed.midYellowPickupPose);
    public static Pose2d outerYellowPickupPose = flipPoseBlue(FieldConstantsRed.outerYellowPickupPose);



    public static Pose2d flipPoseBlue(Pose2d pose) {
        double x = -pose.getX();
        double y = -pose.getY();
        double heading = pose.getHeading() + Math.PI;
        return new Pose2d(x, y, heading);
    }


//


}






