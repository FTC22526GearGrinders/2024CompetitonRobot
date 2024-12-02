package org.firstinspires.ftc.teamcode.commands_actions.combined;


import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.arcrobotics.ftclib.command.CommandOpMode;

import org.firstinspires.ftc.teamcode.subsystems.ElevatorSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ExtendArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.RotateArmSubsystem;
import org.firstinspires.ftc.teamcode.utils.FailoverAction;
import org.firstinspires.ftc.teamcode.utils.RumbleDefs;

public class Elevator_Arm_RotateArm_Actions {

    private final ExtendArmSubsystem arm;
    private final RotateArmSubsystem rotateArm;
    private final ElevatorSubsystem elevator;
    private CommandOpMode opmode;

    public Elevator_Arm_RotateArm_Actions(ElevatorSubsystem elevetor, ExtendArmSubsystem arm, RotateArmSubsystem rotateArm, CommandOpMode opmode) {
        this.elevator = elevetor;
        this.arm = arm;
        this.rotateArm = rotateArm;
        this.opmode = opmode;
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
                rotateArm.tiltBothToBucket(2),
                arm.armToBucketAction(),
                rotateArm.reverseIntakeServosTimed(2));
    }

    public Action deliverSampleToUpperBasket() {
        return
                new SequentialAction(
                        elevator.elevatorToUpperBasket(),
                        elevator.cycleBucket(2),
                        elevator.elevatorToHome());
    }

    public Action deliverSampleToBucket(boolean hasSample) {//use in auto to not raise elevator if sample wasn't picked up
        return
                new FailoverAction(deliverSampleToBucket(), null, hasSample);
    }

    public Action collectSpecimenFromWall() {
        return
                elevator.closeSpecimenClaw();
    }

    public Action deliverSpecimenToUpperSubmersible() {
        return
                new ParallelAction(
                        elevator.elevatorToUpperSubmersible(),
                        new SequentialAction(
                                new SleepAction(.5),
                                elevator.openSpecimenClaw()));

    }

    public Action deliverSpecimenToLowerSubmersible() {
        return
                new ParallelAction(
                        elevator.elevatorToLowSubmersible(),
                        new SequentialAction(
                                new SleepAction(.5),
                                elevator.openSpecimenClaw()));

    }


    public Action testSampleColor() {
        return new SequentialAction(
                new InstantAction(rotateArm::clearColors),
                rotateArm.colorDetectAction(),
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
                new FailoverAction(checkValidColor(), RumbleDefs.rumble(opmode.gamepad1, RumbleDefs.fourStepRumbleEffect, 1500), !rotateArm.sampleAtIntake());
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
                        RumbleDefs.rumble(opmode.gamepad1, RumbleDefs.fourStepRumbleEffect, 1200));
    }


    public Action grabSpecimenAndClearWall() {
        return new SequentialAction(
                elevator.closeSpecimenClaw(),
                new SleepAction(1),
                elevator.elevatorToClearWall());
    }


}


