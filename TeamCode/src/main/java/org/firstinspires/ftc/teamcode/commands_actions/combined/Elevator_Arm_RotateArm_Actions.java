package org.firstinspires.ftc.teamcode.commands_actions.combined;


import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.subsystems.ElevatorSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ExtendArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.RotateArmSubsystem;
import org.firstinspires.ftc.teamcode.utils.FailoverAction;

public class Elevator_Arm_RotateArm_Actions {
    public final Gamepad.RumbleEffect twoStepRumbleEffect;
    public final Gamepad.RumbleEffect threeStepRumbleEffect;
    public final Gamepad.RumbleEffect fourStepRumbleEffect;
    public final Gamepad.RumbleEffect fiveStepRumbleEffect;
    private final ExtendArmSubsystem arm;
    private final RotateArmSubsystem rotateArm;
    private final ElevatorSubsystem elevetor;
    private CommandOpMode opmode;

    public Elevator_Arm_RotateArm_Actions(ElevatorSubsystem elevetor, ExtendArmSubsystem arm, RotateArmSubsystem rotateArm, CommandOpMode opmode) {
        this.elevetor = elevetor;
        this.arm = arm;
        this.rotateArm = rotateArm;
        this.opmode = opmode;

        twoStepRumbleEffect = new Gamepad.RumbleEffect.Builder()
                .addStep(0.0, 1.0, 500)  //  Rumble right motor 100% for 500 mSec
                .addStep(0.0, 0.0, 300)  //  Pause for 300 mSec
                .build();
        threeStepRumbleEffect = new Gamepad.RumbleEffect.Builder()
                .addStep(0.0, 1.0, 500)  //  Rumble right motor 100% for 500 mSec
                .addStep(0.0, 0.0, 300)  //  Pause for 300 mSec
                .addStep(1.0, 0.0, 250)  //  Rumble left motor 100% for 250 mSec
                .build();
        fourStepRumbleEffect = new Gamepad.RumbleEffect.Builder()
                .addStep(0.0, 1.0, 500)  //  Rumble right motor 100% for 500 mSec
                .addStep(0.0, 0.0, 300)  //  Pause for 300 mSec
                .addStep(1.0, 0.0, 250)  //  Rumble left motor 100% for 250 mSec
                .addStep(0.0, 0.0, 250)  //  Pause for 250 mSec
                .build();
        fiveStepRumbleEffect = new Gamepad.RumbleEffect.Builder()
                .addStep(0.0, 1.0, 500)  //  Rumble right motor 100% for 500 mSec
                .addStep(0.0, 0.0, 300)  //  Pause for 300 mSec
                .addStep(1.0, 0.0, 250)  //  Rumble left motor 100% for 250 mSec
                .addStep(0.0, 0.0, 250)  //  Pause for 250 mSec
                .addStep(1.0, 0.0, 250)  //  Rumble left motor 100% for 250 mSec
                .build();
    }

    public Action rumble(Gamepad gamepad, Gamepad.RumbleEffect gre, double duration_ms) {
        return new Action() {
            private boolean initialized = false;
            private long startTime;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    startTime = System.currentTimeMillis();
                    initialized = true;
                }
                gamepad.runRumbleEffect(gre);

                packet.put("rumbling", true);
                return System.currentTimeMillis() < startTime + duration_ms;
            }
        };

    }

    //auto actions - these differ from teleop

    /*
     * moves arm to pickup up distance, tilts rotate arm to pickup angle and runs intake until sample seen
     * ot timeout expires
     * In either event tilt will raise and arm will return to basket position.
     * if a sample is seen at intake rollers will run reverse to put sample in basket
     * Trajectory will return to basket delivery point but will only deliver to field basket if a sample was picked up.
     * Trajectories will resume in either event
     */

    public Action getSampleAutoBasket() {
        return new SequentialAction(
                arm.armToPickupAction(),
                rotateArm.tiltBothDown(),
                rotateArm.runUntilSampleOrTimeout(10),
                rotateArm.tiltBothClear(3),
                arm.armToBucketAction(),
                new FailoverAction(rotateArm.reverseIntakeServosTimed(2), null, !rotateArm.sampleAtIntake()));
    }

    public Action deliverSampleToUpperBasket() {
        return
                new SequentialAction(
                        elevetor.elevatorToUpperBasket(),
                        elevetor.cycleBucket(2),
                        elevetor.elevatorToHome());
    }

    public Action deliverSampleToBucket(boolean hasSample) {//use in auto to not raise elevator if sample wasn't picked up
        return
                new FailoverAction(deliverSampleToBucket(), null, hasSample);
    }

    public Action collectSpecimenFromWall() {
        return
                elevetor.closeSpecimenClaw();
    }

    public Action deliverSpecimenToUpperSubmersible() {
        return
                new ParallelAction(
                        elevetor.elevatorToUpperSubmersible(),
                        new SequentialAction(
                                new SleepAction(.5),
                                elevetor.openSpecimenClaw()));

    }

    public Action deliverSpecimenToLowerSubmersible() {
        return
                new ParallelAction(
                        elevetor.elevatorToLowSubmersible(),
                        new SequentialAction(
                                new SleepAction(.5),
                                elevetor.openSpecimenClaw()));

    }


    public Action testSampleColor() {
        return new SequentialAction(
                new InstantAction(rotateArm::clearColors),
                rotateArm.colorDetectAction(1),
                new FailoverAction(deliverSampleToBucket(), rotateArm.rejectSampleAction(), rotateArm.wrongAllianceSampleSeen()));
    }

    public Action deliverSampleToBucket() {
        return
                new SequentialAction(
                        rotateArm.tiltBothClear(3),
                        arm.armToBucketAction(),
                        rotateArm.reverseIntakeServosTimed(2));
    }

    //this may result in a sample or not - need to check
    public Action pickupSample(double timeout_secs) {
        return new SequentialAction(
                arm.armToPickupAction(),
                rotateArm.tiltBothDown(),
                rotateArm.runUntilSampleOrTimeout(timeout_secs),
                checkSamplePresent());
    }

    //if sample present go check its color, if not rumble thr driver
    public Action checkSamplePresent() {
        return
                new FailoverAction(checkValidColor(), rumble(opmode.gamepad1, fourStepRumbleEffect, 1500), !rotateArm.sampleAtIntake());
    }

    //if valid color deliver to bucket else kick it out of intake
    public Action checkValidColor() {
        return
                new FailoverAction(deliverSampleToBucket(), rejectSampleWarnDriver(), !rotateArm.colorDetected);
    }

    public Action rejectSampleWarnDriver() {
        return
                new ParallelAction(
                        rotateArm.reverseIntakeServosTimed(2),
                        rumble(opmode.gamepad1, fourStepRumbleEffect, 1200));
    }


    public Action deliverSampleToZone() {
        return rotateArm.reverseIntakeServosTimed(3);
    }


}


