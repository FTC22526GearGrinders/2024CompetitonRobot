package org.firstinspires.ftc.teamcode.commandsandactions.drive;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

public class FailoverAction implements Action {
    private final Action mainAction;
    private final Action failoverAction;
    private boolean failedOver = false;

    public FailoverAction(Action mainAction, Action failoverAction, boolean failedOver) {
        this.mainAction = mainAction;
        this.failoverAction = failoverAction;
        this.failedOver = failedOver;
    }

    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
        if (failedOver) {
            return failoverAction.run(telemetryPacket);
        }

        return mainAction.run(telemetryPacket);
    }

//    public void failover() {
//        failedOver = true;
//    }
}