package org.firstinspires.ftc.teamcode.commands.servo_actions;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Constants;

public class SampleClawServo {
    public static Servo sampleClawServo;


    public SampleClawServo(HardwareMap hardwareMap) {
        sampleClawServo = hardwareMap.get(Servo.class, "sampleClawServo");
    }

    public static Action clawClose() {
        return new Action() {
            private boolean initialized = false;
            private double currentClawPosition = 0;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    currentClawPosition = Constants.ElevatorConstants.sampleClawClosedAngle;
                    sampleClawServo.setPosition(currentClawPosition);
                    initialized = true;
                }
                packet.put("position", currentClawPosition);
                return currentClawPosition == Constants.ElevatorConstants.sampleClawClosedAngle;
            }
        };
    }

    public static Action clawOpen() {
        return new Action() {
            private boolean initialized = false;
            private double currentTiltPosition = 0;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    currentTiltPosition = Constants.ElevatorConstants.sampleClawOpenAngle;
                    sampleClawServo.setPosition(currentTiltPosition);
                    initialized = true;
                }

                packet.put("position", currentTiltPosition);
                return currentTiltPosition == Constants.ElevatorConstants.sampleClawOpenAngle;
            }
        };
    }
}
