package org.firstinspires.ftc.teamcode.commands.servo_actions;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Constants;

public class BucketTipServo {
    public static Servo bucketServo;


    public BucketTipServo(HardwareMap hardwareMap) {
        bucketServo = hardwareMap.get(Servo.class, "bucketServo");
    }

    public static Action tipBucket() {
        return new Action() {
            private boolean initialized = false;
            private double currentBucketPosition = 0;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    currentBucketPosition = Constants.ElevatorConstants.bucketTippedAngle;
                    bucketServo.setPosition(currentBucketPosition);
                    initialized = true;
                }
                packet.put("bucketposition", currentBucketPosition);
                return currentBucketPosition == Constants.ElevatorConstants.bucketTippedAngle;
            }
        };
    }

    public static Action levelBucket() {
        return new Action() {
            private boolean initialized = false;
            private double currentTiltPosition = 0;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    currentTiltPosition = Constants.ElevatorConstants.bucketUprightAngle;
                    bucketServo.setPosition(currentTiltPosition);
                    initialized = true;
                }

                packet.put("bucketposition", currentTiltPosition);
                return currentTiltPosition == Constants.ElevatorConstants.bucketUprightAngle;
            }
        };
    }
}
