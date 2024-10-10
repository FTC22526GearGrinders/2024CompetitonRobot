package org.firstinspires.ftc.teamcode;


import com.acmerobotics.roadrunner.Pose2d;


public final class FieldConstantsBlue {

    /*
     *
     *
     * */


    public static Pose2d basketSideStartPose = flipPoseBlue(FieldConstantsRed.basketSideStartPose);
    public static Pose2d basketDeliverPose = flipPoseBlue(FieldConstantsRed.basketDeliverPose);

    public static Pose2d innerYellowPickupPose = flipPoseBlue(FieldConstantsRed.innerYellowPickupPose);
    public static Pose2d midYellowPickupPose = flipPoseBlue(FieldConstantsRed.midYellowPickupPose);
    public static Pose2d outerYellowApproachPose = flipPoseBlue(FieldConstantsRed.outerYellowApproachPose);
    public static Pose2d outerYellowPickupPose = flipPoseBlue(FieldConstantsRed.outerYellowPickupPose);



    public static Pose2d flipPoseBlue(Pose2d pose) {
        double x = -pose.position.x;
        double y = -pose.position.y;
        double heading = pose.heading.toDouble();
        heading = heading % 360;
        if (heading > 180) {
            heading -= 360;
        } else if (heading < -180) {
            heading += 360;
        }
        return new Pose2d(x, y, heading);

    }


//


}






