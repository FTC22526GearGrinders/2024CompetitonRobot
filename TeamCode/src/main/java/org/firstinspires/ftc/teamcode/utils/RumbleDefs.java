package org.firstinspires.ftc.teamcode.utils;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.Gamepad;

// credit Tarun from 12791 (@_skull.emoji_) for this class
public class RumbleDefs {
    public static final Gamepad.RumbleEffect twoStepRumbleEffect = new Gamepad.RumbleEffect.Builder()
            .addStep(0.0, 1.0, 500)  //  Rumble right motor 100% for 500 mSec
            .addStep(0.0, 0.0, 300)  //  Pause for 300 mSec
            .build();
    ;
    public static final Gamepad.RumbleEffect threeStepRumbleEffect = new Gamepad.RumbleEffect.Builder()
            .addStep(0.0, 1.0, 500)  //  Rumble right motor 100% for 500 mSec
            .addStep(0.0, 0.0, 300)  //  Pause for 300 mSec
            .addStep(1.0, 0.0, 250)  //  Rumble left motor 100% for 250 mSec
            .build();
    ;
    public static final Gamepad.RumbleEffect fourStepRumbleEffect = new Gamepad.RumbleEffect.Builder()
            .addStep(0.0, 1.0, 500)  //  Rumble right motor 100% for 500 mSec
            .addStep(0.0, 0.0, 300)  //  Pause for 300 mSec
            .addStep(1.0, 0.0, 250)  //  Rumble left motor 100% for 250 mSec
            .addStep(0.0, 0.0, 250)  //  Pause for 250 mSec
            .build();
    ;
    public static final Gamepad.RumbleEffect fiveStepRumbleEffect = new Gamepad.RumbleEffect.Builder()
            .addStep(0.0, 1.0, 500)  //  Rumble right motor 100% for 500 mSec
            .addStep(0.0, 0.0, 300)  //  Pause for 300 mSec
            .addStep(1.0, 0.0, 250)  //  Rumble left motor 100% for 250 mSec
            .addStep(0.0, 0.0, 250)  //  Pause for 250 mSec
            .addStep(1.0, 0.0, 250)  //  Rumble left motor 100% for 250 mSec
            .build();

    public static Action rumble(Gamepad gamepad, Gamepad.RumbleEffect gre, double duration_ms) {
        return new Action() {
            private boolean initialized = false;
            private long startTime;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    startTime = System.currentTimeMillis();
                    initialized = true;
                }
                gamepad.runRumbleEffect(RumbleDefs.threeStepRumbleEffect);

                packet.put("rumbling", true);
                return System.currentTimeMillis() < startTime + duration_ms;
            }
        };

    }

}