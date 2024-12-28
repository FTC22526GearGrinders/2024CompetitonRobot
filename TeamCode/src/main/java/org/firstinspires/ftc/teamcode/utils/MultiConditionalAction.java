package org.firstinspires.ftc.teamcode.utils;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

public class MultiConditionalAction implements Action {
    private final Action firstAction;
    private final Action secondAction;
    private final Action thirdAction;
    private int select;
    private boolean initialized = false;
    private int latchCondition;

    public MultiConditionalAction(Action firstAction, Action secondAction, Action thirdAction, int select) {
        this.firstAction = firstAction;
        this.secondAction = secondAction;
        this.thirdAction = thirdAction;
        this.select = select;
    }

    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
        if (select > 2) select = 0;
        if (!initialized) {
            latchCondition = select;
            initialized = true;
        }

        if (latchCondition == 0) {
            return firstAction.run(telemetryPacket);
        } else if (latchCondition == 1) {
            return secondAction.run(telemetryPacket);
        } else
            return thirdAction.run(telemetryPacket);
    }
}