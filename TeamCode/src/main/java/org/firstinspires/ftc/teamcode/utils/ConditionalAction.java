package org.firstinspires.ftc.teamcode.utils;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

public class ConditionalAction implements Action {
    private final Action firstAction;
    private final Action secondAction;
    private boolean select = false;
    private boolean initialized = false;
    private boolean latchCondition = false;

    public ConditionalAction(Action firstAction, Action secondAction, boolean select) {
        this.firstAction = firstAction;
        this.secondAction = secondAction;
        this.select = select;
    }

    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
        if (!initialized) {
            latchCondition = select;
            initialized = true;
        }

        if (!latchCondition) {
            return firstAction.run(telemetryPacket);
        } else return secondAction.run(telemetryPacket);
    }


}