package org.firstinspires.ftc.teamcode.commands_actions.combined;


import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.arcrobotics.ftclib.command.CommandOpMode;

import org.firstinspires.ftc.teamcode.subsystems.ElevatorSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ExtendArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.RotateArmSubsystem;

public class Elevator_Arm_RotateArm_Actions {

    private final ExtendArmSubsystem arm;
    private final RotateArmSubsystem rotateArm;
    private final ElevatorSubsystem elevator;
    private CommandOpMode opmode;
    private boolean initialized = false;

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


//    public Action deliverSampleToBasketThenHome(boolean upper) {
//        return
//                new SequentialAction(
//                        new ConditionalAction(
//                                elevator.elevatorToUpperBasket(),
//                                elevator.elevatorToLowBasket(),
//                                upper),
//                        elevator.cycleBucket(2),
//                        elevator.elevatorToHome());
//    }


    public Action deliverSpecimenToUpperSubmersible() {
        return
                new ParallelAction(
                        elevator.elevatorToUpperSubmersible(),
                        new SequentialAction(
                                new SleepAction(elevator.releaseDelay),
                                elevator.openSpecimenClaw()));
    }


    public Action tiltAndArmMoveThenDeliverToBucket() {
        return
                new SequentialAction(
                        rotateArm.tiltToBucketDeliver(),
                        arm.armToHome(),
                        rotateArm.openIntakeClaw(),
                        new SleepAction(.5),
                        elevator.travelBucket(),
                        rotateArm.tiltBothVertical());
    }


    public Action cancelPickupSample() {
        return
                new SequentialAction(
                        rotateArm.tiltToBucketDeliver(),
                        arm.armToHome());
    }


    //this may result in a sample or not - need to check
    public Action tiltToPickupCloseClawRaiseTiltAboveSubmersible() {
        return new SequentialAction(
                arm.armToPickupAction(),
                rotateArm.tiltToPickup(),
                new SleepAction(.1),
                rotateArm.closeIntakeClaw(),
                new SleepAction(1),
                rotateArm.tiltAboveSubmersible());
    }


    public Action armOutTiltAboveSamplesOpenClaw() {
        return new SequentialAction(
                arm.armToPickupAction(),
                new ParallelAction(
                        rotateArm.tiltAboveSamples(),
                        rotateArm.openIntakeClaw()));
    }

    public Action autoArmPickupThenBucketDrop() {
        return
                new SequentialAction(
                        rotateArm.closeIntakeClaw(),
                        new SleepAction(0.5),
                        elevator.levelBucket(),
                        // new SleepAction(.5),//claw close wait
                        rotateArm.tiltToBucketDeliver(),
                        arm.armToHome(),//arm tolerance is 2" so will finish early
                        new SleepAction(.5),//wait for arm and tilt
                        rotateArm.openIntakeClaw(),
                        new SleepAction(1),//claw open wait for sample to drop
                        elevator.travelBucket(),
                        rotateArm.tiltBothVertical());
    }

    public Action autoArmOutTiltToPickup() {
        return
                new ParallelAction(
                        arm.armToAutoPickupAction(),// 2" in position means ends early but continues to position
                        new SequentialAction(
                                new SleepAction(.5),//wait before tilt out
                                rotateArm.tiltToPickup()));
    }

    public Action autoArmOutTiltToVertical() {
        return
                new ParallelAction(
                        arm.armToAutoPickupAction(),// 2" in position means ends early but continues to position
                        new SequentialAction(
                                new SleepAction(.5),//wait before tilt out
                                rotateArm.tiltBothVertical()));
    }



    public Action testDeliver() {
        return new SequentialAction(
                elevator.elevatorToUpperBasket(),
                elevator.cycleBucketToVertical());//keep clear of rotate for now
    }

    public Action testPickup() {
        return new SequentialAction(
                new ParallelAction(
                        elevator.elevatorToHome(),//start elevator down
                        new SequentialAction(
                                new SleepAction(1),//wait before arm extend
                                autoArmOutTiltToPickup())),// extend arm and rotate to pickup

                elevator.levelBucket(),
                autoArmPickupThenBucketDrop());//close claw, raise tilt, arm to bucket open claw
    }


}


