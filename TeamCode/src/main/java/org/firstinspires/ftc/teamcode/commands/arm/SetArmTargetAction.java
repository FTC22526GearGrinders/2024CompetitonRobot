package org.firstinspires.ftc.teamcode.commands.arm;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

import org.firstinspires.ftc.teamcode.subsystems.ExtendArmSubsystem;

public class SetArmTargetAction {
   private  static ExtendArmSubsystem arm;

    public SetArmTargetAction() {

    }

    public static Action setTarget(double targetInches) {
        return new Action() {
            private boolean initialized = false;


            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                  arm.setTargetInches(targetInches);
                    initialized = true;
                }
                packet.put("targetInches", arm.getTargetInches());
                packet.put("actualInches", arm.getPositionInches());

                return arm.atGoal();
            }
        };
    }


}
