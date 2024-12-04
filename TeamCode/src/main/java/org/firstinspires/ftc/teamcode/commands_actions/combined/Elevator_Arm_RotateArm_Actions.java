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


    public Action deliverSampleToUpperBasket() {
        return
                new SequentialAction(
                        elevator.elevatorToUpperBasket(),
                        elevator.cycleBucket(2),
                        elevator.elevatorToHome());
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
                                new SleepAction(ElevatorSubsystem.releaseDelay),
                                elevator.openSpecimenClaw()));
    }

    public Action deliverSpecimenToLowerSubmersible() {
        return
                new ParallelAction(
                        elevator.elevatorToLowerSubmersible(),
                        new SequentialAction(
                                new SleepAction(.5),
                                elevator.openSpecimenClaw()));
    }

    public Action deliverSpecimenToSubmersible() {
        return
                new ParallelAction(
                        elevator.elevatorToNearestSubmersible(),
                        new SequentialAction(
                                new SleepAction(.5),
                                elevator.openSpecimenClaw()));
    }



    public Action deliverSampleToBucket() {
        return
                new SequentialAction(
                        rotateArm.tiltBothClear(3),
                        arm.armToBucketAction(),
                        rotateArm.openIntakeClaw(),
                        new SleepAction(1),
                        elevator.elevatorToHome());
    }

    public Action cancelPickupSample() {
        return
                new SequentialAction(
                        rotateArm.tiltBothClear(3),
                        arm.armToBucketAction());
    }


    //this may result in a sample or not - need to check
    public Action prepareToPickupSample() {
        return new SequentialAction(
                arm.armToPickupAction(),
                new ParallelAction(
                        rotateArm.tiltBothDown(),
                        rotateArm.openIntakeClaw()));
    }




}


