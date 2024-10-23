package org.firstinspires.ftc.teamcode.commandsandactions.auto;


import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.CommandOpMode;


public class SelectMotionValuesBlue extends CommandBase {


    private final CommandOpMode opMode;

    public SelectMotionValuesBlue(CommandOpMode opMode) {
        this.opMode = opMode;
    }


    @Override
    public void initialize() {
        opMode.telemetry.addData("BLUE", "");
        opMode.telemetry.update();
    }

    @Override
    public void execute() {

//        boolean bbstart = ActiveMotionValues.getBBStart();
     //  int lcr = ActiveMotionValues.getLcrpos();
//        ActiveMotionValues.setAdvancePose(new Pose2d());
//        ActiveMotionValues.setRetractPose(new Pose2d());
//        ActiveMotionValues.setClearPose(new Pose2d());
//        ActiveMotionValues.setClearToTurnPose(new Pose2d());
//        ActiveMotionValues.setParkPose(new Pose2d());
//        ActiveMotionValues.setPreParkPose(new Pose2d());
//        ActiveMotionValues.setPreTagPose(new Pose2d());
//        ActiveMotionValues.setTagLineupPose(new Pose2d());
//        ActiveMotionValues.setTurnAngle(0);
//
//        ActiveMotionValues.setStopSecs(.1);

//        if (lcr < 1 || lcr > 3) lcr = 1;
//
        int motionSelected = 0;//lcr;

      //  if (!bbstart) motionSelected += 10;


        switch (motionSelected) {

            //******************************************************************************************
            //******************************************************************************************


            case 1://left tape not truss side
//                double dropOffsetX = -FieldConstantsBlue.pixelLeftDropX;
//
//                Pose2d dropOffsetPose = new Pose2d(dropOffsetX, 0, 0);
//
//
//                ActiveMotionValues.setStartPose(FieldConstantsBlue.XPYP.startPos);//start pose
//
//                ActiveMotionValues.setAdvancePose(FieldConstantsBlue.XPYP.leftAdvancePose);
//
//                ActiveMotionValues.setActTag(1);
//
//                ActiveMotionValues.setDropOffPose(FieldConstantsBlue.XPYP.leftDropPose);
//
//                ActiveMotionValues.setRetractPose(FieldConstantsBlue.getActiveTagLineupPose(ActiveMotionValues.getActTag())
//                        .minus(dropOffsetPose));
//
//                ActiveMotionValues.setTurnAngle(FieldConstantsBlue.turnToTagRadians);
//
//                ActiveMotionValues.setPreTagPose(FieldConstantsBlue.getActiveTagPose(ActiveMotionValues.getActTag())
//                        .minus(FieldConstantsBlue.AprilTagConstants.tagLookAheadPoseLeft));
//
//                ActiveMotionValues.setParkPose(new Pose2d());
//                ActiveMotionValues.setPreParkPose(new Pose2d());
//
//                if (ActiveMotionValues.getCenterPark()) {
//                    ActiveMotionValues.setPreParkPose(FieldConstantsBlue.centerPreParkPoseBB);
//                    ActiveMotionValues.setParkPose(FieldConstantsBlue.centerParkPoseBB);
//                }
//                if (ActiveMotionValues.getNearPark()) {
//                    ActiveMotionValues.setPreParkPose(FieldConstantsBlue.nearPreParkPoseBB);
//                    ActiveMotionValues.setParkPose(FieldConstantsBlue.nearParkPoseBB);
//                }
//
//                break;
//
//            //******************************************************************************************
//            //******************************************************************************************
//            case 2://center straight motion to midddle of center tape
//
//                //robot moves in Y
//
//                ActiveMotionValues.setStartPose(FieldConstantsBlue.XPYP.startPos);//start pose
//
//                ActiveMotionValues.setDropOffPose(FieldConstantsBlue.XPYP.centerDropPose);
//
//                ActiveMotionValues.setRetractPose(FieldConstantsBlue.XPYP.centerRetractPose);
//
//                ActiveMotionValues.setTurnAngle(FieldConstantsBlue.turnToTagRadians);
//
//                ActiveMotionValues.setActTag(2);
//
//                ActiveMotionValues.setPreTagPose(FieldConstantsBlue.getActiveTagPose(ActiveMotionValues.getActTag())
//                        .minus(FieldConstantsBlue.AprilTagConstants.tagLookAheadPoseCenter));
//
//                ActiveMotionValues.setParkPose(new Pose2d());
//
//                ActiveMotionValues.setParkPose(new Pose2d());
//                ActiveMotionValues.setPreParkPose(new Pose2d());
//
//                if (ActiveMotionValues.getCenterPark()) {
//                    ActiveMotionValues.setPreParkPose(FieldConstantsBlue.centerPreParkPoseBB);
//                    ActiveMotionValues.setParkPose(FieldConstantsBlue.centerParkPoseBB);
//                }
//                if (ActiveMotionValues.getNearPark()) {
//                    ActiveMotionValues.setPreParkPose(FieldConstantsBlue.nearPreParkPoseBB);
//                    ActiveMotionValues.setParkPose(FieldConstantsBlue.nearParkPoseBB);
//                }
//
//                break;
//
//            //******************************************************************************************
//            //******************************************************************************************
//            case 3://truss side
//
//
//                ActiveMotionValues.setStartPose(FieldConstantsBlue.XPYP.startPos);//start pose
//
//
//                ActiveMotionValues.setAdvancePose(FieldConstantsBlue.XPYP.rightAdvancePose);
//
//
//                ActiveMotionValues.setDropOffPose(FieldConstantsBlue.XPYP.rightDropPose);
//
//
//                ActiveMotionValues.setRetractPose(FieldConstantsBlue.XPYP.rightRetractPose);
//
//                ActiveMotionValues.setClearPose(FieldConstantsBlue.XPYP.rightClearPose);
//
//                ActiveMotionValues.setTurnAngle(FieldConstantsBlue.turnToTagRadians);
//
//                ActiveMotionValues.setActTag(3);
//
//                ActiveMotionValues.setTagLineupPose(FieldConstantsBlue.getActiveTagLineupPose(ActiveMotionValues.getActTag()));
//
//                ActiveMotionValues.setPreTagPose(FieldConstantsBlue.getActiveTagPose(ActiveMotionValues.getActTag())
//                        .minus(FieldConstantsBlue.AprilTagConstants.tagLookAheadPoseRight));
//
//                ActiveMotionValues.setParkPose(new Pose2d());
//
//                ActiveMotionValues.setParkPose(new Pose2d());
//                ActiveMotionValues.setPreParkPose(new Pose2d());
//
//                if (ActiveMotionValues.getCenterPark()) {
//                    ActiveMotionValues.setPreParkPose(FieldConstantsBlue.centerPreParkPoseBB);
//                    ActiveMotionValues.setParkPose(FieldConstantsBlue.centerParkPoseBB);
//                }
//                if (ActiveMotionValues.getNearPark()) {
//                    ActiveMotionValues.setPreParkPose(FieldConstantsBlue.nearPreParkPoseBB);
//                    ActiveMotionValues.setParkPose(FieldConstantsBlue.nearParkPoseBB);
//                }
//
//                break;
//
//            //******************************************************************************************
//            //******************************************************************************************
//
//            case 11://left tape
//
//                ActiveMotionValues.setStartPose(FieldConstantsBlue.XMYP.startPose);//start pose
//
//                ActiveMotionValues.setAdvancePose(FieldConstantsBlue.XMYP.leftAdvancePose);
//
//                ActiveMotionValues.setDropOffPose(FieldConstantsBlue.XMYP.leftDropPose);
//
//                ActiveMotionValues.setRetractPose(FieldConstantsBlue.XMYP.leftRetractPose);
//
//                ActiveMotionValues.setClearPose(FieldConstantsBlue.XMYP.clearPose);
//
//
//                ActiveMotionValues.setActTag(1);
//
//                setOptions();
//
//
//                break;
//
//            //******************************************************************************************
//            //******************************************************************************************
//
//            case 12://center
//
//                ActiveMotionValues.setStartPose(FieldConstantsBlue.XMYP.startPose);//start pose
//
//                ActiveMotionValues.setDropOffPose(FieldConstantsBlue.XMYP.centerDropPose);
//
//                ActiveMotionValues.setRetractPose(FieldConstantsBlue.XMYP.centerRetractPose);
//
//
//                ActiveMotionValues.setActTag(2);
//
//                setOptions();
//
//                break;
//
//            //******************************************************************************************
//            //******************************************************************************************
//            case 13://right tape Blue
//
//                ActiveMotionValues.setStartPose(FieldConstantsBlue.XMYP.startPose);//start pose
//
//                ActiveMotionValues.setAdvancePose(FieldConstantsBlue.XMYP.rightAdvancePose);
//
//                ActiveMotionValues.setDropOffPose(FieldConstantsBlue.XMYP.rightDropPose);
//
//                ActiveMotionValues.setRetractPose(FieldConstantsBlue.XMYP.rightRetractPose);
//
//                ActiveMotionValues.setClearPose(FieldConstantsBlue.XMYP.clearPose);
//
//
//                ActiveMotionValues.setActTag(3);
//
//                setOptions();
//
//                break;

        }
    }

    private void setOptions() {

//        int lcr = ActiveMotionValues.getLcrpos() + 10;
//        boolean useStageDoor = ActiveMotionValues.getUseStageDoor();
//        boolean secondPixel = ActiveMotionValues.getSecondPixel();
//
//        ActiveMotionValues.setParkPose(new Pose2d());
//
//        boolean useTruss = !useStageDoor;
//
//
//        double stop2PixelSecs = .2;
//
//
//        if (ActiveMotionValues.getCenterPark())
//
//            ActiveMotionValues.setParkPose(FieldConstantsBlue.centerParkPose);
//        else
//            ActiveMotionValues.setParkPose(FieldConstantsBlue.nearParkPose);
//
//
//        if (useTruss) {
//            if (!secondPixel) {
//                ActiveMotionValues.setTrussSDLineUpPose(FieldConstantsBlue.nearTrussLineUpPoseNoTurn);
//                ActiveMotionValues.setOptionStopPose(FieldConstantsBlue.nearOptionPoseNoTurn);
//                ActiveMotionValues.setParkPose(FieldConstantsBlue.nearParkPose);
//
//            } else {
//                ActiveMotionValues.setTrussSDLineUpPose(FieldConstantsBlue.nearTrussLineUpPose);
//                ActiveMotionValues.setOptionStopPose(FieldConstantsBlue.nearOptionPose);
//                ActiveMotionValues.setWaitPartnerClearPose(FieldConstantsBlue.nearPartnerClearPose);
//                ActiveMotionValues.setOptionStopPose(FieldConstantsBlue.nearOptionPose);
//                ActiveMotionValues.setTurnAngle(FieldConstantsBlue.turnToTagRadians);
//                ActiveMotionValues.setStopSecs(stop2PixelSecs);
//                ActiveMotionValues.setTargetPose(FieldConstantsBlue.getActiveTagPose(ActiveMotionValues.getActTag())
//                        .plus(FieldConstantsBlue.AprilTagConstants.tagStrafeOffsetPoseTruss));
//            }
//        }
//
//        if (useStageDoor) {
//
//            if (lcr == 11 || lcr == 13) {
//
//                ActiveMotionValues.setTrussSDLineUpPose((FieldConstantsBlue.stageDoorLineUpPose13));
//            }
//            if (lcr == 12) {
//                ActiveMotionValues.setClearPose(FieldConstantsBlue.XMYP.centerSDClearPixePose);
//                ActiveMotionValues.setTrussSDLineUpPose((FieldConstantsBlue.stageDoorLineUpPose2));
//            }
//
//            if (!secondPixel) {
//                ActiveMotionValues.setOptionStopPose(FieldConstantsBlue.centerParkOptionPose);
//                ActiveMotionValues.setParkPose(FieldConstantsBlue.centerParkPose);
//            } else {
//                ActiveMotionValues.setWaitPartnerClearPose(FieldConstantsBlue.centerPartnerClearPose);
//                ActiveMotionValues.setOptionStopPose(FieldConstantsBlue.centerOptionPose);
//
//                ActiveMotionValues.setTurnAngle(FieldConstantsBlue.turnToTagRadians);
//                ActiveMotionValues.setStopSecs(stop2PixelSecs);
//                ActiveMotionValues.setTargetPose(FieldConstantsBlue.getActiveTagPose(ActiveMotionValues.getActTag())
//                        .plus(FieldConstantsBlue.AprilTagConstants.tagStrafeOffsetPoseSD));
//            }
        }
    }

//   // @Override
//    public void end(boolean interrupted) {
//
//    }
//
//    @Override
//    public boolean isFinished() {
//        return true;
//    }

