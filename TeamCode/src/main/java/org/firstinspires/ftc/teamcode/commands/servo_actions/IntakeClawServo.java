package org.firstinspires.ftc.teamcode.commands.servo_actions;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Constants;

public class IntakeClawServo {
    public static Servo intakeClawServo;


    public IntakeClawServo(HardwareMap hardwareMap) {
        intakeClawServo = hardwareMap.get(Servo.class, "intakeClawServo");
    }

    public static Action clawClose() {
        return new Action() {
            private boolean initialized = false;
            private double currentTiltPosition = 0;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    currentTiltPosition = Constants.ArmConstants.intakeClawClosedAngle;
                    intakeClawServo.setPosition(currentTiltPosition);
                    initialized = true;
                }
                packet.put("position", currentTiltPosition);
                return currentTiltPosition == Constants.ArmConstants.intakeClawClosedAngle;
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
                    currentTiltPosition = Constants.ArmConstants.intakeClawOpenAngle;
                    intakeClawServo.setPosition(currentTiltPosition);
                    initialized = true;
                }

                packet.put("position", currentTiltPosition);
                return currentTiltPosition == Constants.ArmConstants.intakeClawOpenAngle;
            }
        };
    }
}
