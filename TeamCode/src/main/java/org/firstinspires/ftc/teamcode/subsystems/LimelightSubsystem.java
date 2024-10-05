package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

public class LimelightSubsystem extends SubsystemBase {

    private final Limelight3A limelight;
    public Telemetry telemetry;
    public boolean show = true;

    public int yellowPipeline = 0;
    public int redPipeline = 1;
    public int bluePipeline = 2;
    public int aprilTagPipeline = 3;
    public int test;

    public LimelightSubsystem(CommandOpMode opMode) {
        limelight = opMode.hardwareMap.get(Limelight3A.class, "limelight");
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(opMode.telemetry, dashboard.getTelemetry());
        telemetry.setMsTransmissionInterval(11);
        limelight.pipelineSwitch(3);
        limelight.start();
    }
    /*
     * Pipelines used
     * 0=RED
     * 1=YELLOW
     * 2=BLUE
     * 3 = April Tag
     *
     *
     *
     * */

    @Override
    public void periodic() {

//        if (show) {
//            showTelemetry();
//        }

    }


    public Pose2d convertPose3dTo2d(Pose3D pose3) {
        return new Pose2d(pose3.getPosition().x, pose3.getPosition().y, Math.PI);//pose3.getOrientation().getYaw());
    }
    public double getP3x(){
        return limelight.getLatestResult().getBotpose().getPosition().x;
    }
    public double getP3y(){
        return limelight.getLatestResult().getBotpose().getPosition().y;
    }

    public Pose2d getBotPose2d() {//
        LLResult result = limelight.getLatestResult();
        if (result.isValid())
            return convertPose3dTo2d(result.getBotpose()); //getBotpose_MT2
        else return new Pose2d(20, 20, Math.PI);
    }

    public void setPipeline(int number) {
        limelight.pipelineSwitch(number);
    }

    public void setYellowNotePipeline() {
        setPipeline(yellowPipeline);
    }

    public void setRedNotePipeline() {
        setPipeline(redPipeline);
    }

    public void setBlueNotePipeline() {
        setPipeline(bluePipeline);
    }

    public void setAprilTagPipeline() {
        setPipeline(aprilTagPipeline);
    }

    public LLResult getLatestResult() {
        return limelight.getLatestResult();
    }

    public double getTX() {
        return limelight.getLatestResult().getTx();
    }

    public double getTY() {
        return limelight.getLatestResult().getTy();
    }

    public double getTA() {
        return limelight.getLatestResult().getTa();
    }

    public double getVersion() {
        return limelight.getVersion();
    }

    double getCaptureLatency() {
        return limelight.getLatestResult().getCaptureLatency();
    }

    double getTargetingLatency() {
        return limelight.getLatestResult().getTargetingLatency();
    }

    int getNumberTagsSeen() {
        return getLatestResult().getBotposeTagCount();
    }


    public void showTelemetry() {
        telemetry.addData("Limelight", show);
        telemetry.addData("ResultValid", getLatestResult().isValid());
        telemetry.addData("DDD", test);
        telemetry.addData("Connected", limelight.isConnected());
        telemetry.addData("Running", limelight.isRunning());
        telemetry.addData("TagCount", getNumberTagsSeen());
        telemetry.addData("UpdateTime", limelight.getTimeSinceLastUpdate());
//
        telemetry.addData("Tx", getTX());
        telemetry.addData("Ty", getTY());


        telemetry.update();

    }
}
