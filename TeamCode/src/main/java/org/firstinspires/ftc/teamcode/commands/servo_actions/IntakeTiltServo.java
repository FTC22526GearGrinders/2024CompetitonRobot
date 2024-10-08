package org.firstinspires.ftc.teamcode.commands.servo_actions;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Constants;

public class IntakeTiltServo {
    public static Servo tiltServo;


    public IntakeTiltServo(HardwareMap hardwareMap) {
        tiltServo = hardwareMap.get(Servo.class, "tiltServo");
    }

    public static Action tiltDown() {
        return new Action() {
            private boolean initialized = false;
            private double currentTiltPosition = 0;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    currentTiltPosition = Constants.ArmConstants.intakeTiltDownAngle;
                    tiltServo.setPosition(currentTiltPosition);
                    initialized = true;
                }
                packet.put("position", currentTiltPosition);
                return currentTiltPosition == Constants.ArmConstants.intakeTiltDownAngle;
            }
        };
    }

    public static Action tiltClear() {
        return new Action() {
            private boolean initialized = false;
            private double currentTiltPosition = 0;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    currentTiltPosition = Constants.ArmConstants.intakeTiltClearAngle;
                    tiltServo.setPosition(currentTiltPosition);
                    initialized = true;
                }

                packet.put("position", currentTiltPosition);
                return currentTiltPosition == Constants.ArmConstants.intakeTiltClearAngle;
            }
        };
    }
}
