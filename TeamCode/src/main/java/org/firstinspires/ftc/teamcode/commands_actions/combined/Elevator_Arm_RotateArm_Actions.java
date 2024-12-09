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


    public Action tiltClearArmToBucket(double tilt_timeout_secs) {
        return
                new SequentialAction(
                        rotateArm.tiltBothClear(tilt_timeout_secs),
                        arm.armToBucketAction(),
                        rotateArm.openIntakeClaw(),
                        new SleepAction(.5),
                        elevator.travelBucket(),
                        rotateArm.tiltBothVertical(.2));
    }


    public Action cancelPickupSample() {
        return
                new SequentialAction(
                        rotateArm.tiltBothClear(3),
                        arm.armToBucketAction());
    }


    //this may result in a sample or not - need to check
    public Action armOutTiltToPickupOpenClaw() {
        return new SequentialAction(
                arm.armToPickupAction(),
                new ParallelAction(
                        rotateArm.tiltToPickup(),
                        rotateArm.openIntakeClaw()));
    }


    public Action armOutTiltAboveSamplesOpenClaw(double timeout_secs) {
        return new SequentialAction(
                arm.armToPickupAction(),
                new ParallelAction(
                        rotateArm.tiltBothAboveSamples(timeout_secs),
                        rotateArm.openIntakeClaw()));
    }


}


