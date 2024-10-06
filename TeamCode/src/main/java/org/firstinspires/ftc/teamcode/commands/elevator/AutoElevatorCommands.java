package org.firstinspires.ftc.teamcode.commands.elevator;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.subsystems.ElevatorSubsystem;


public class AutoElevatorCommands extends CommandBase {


    private final ElevatorSubsystem elevator;

    public AutoElevatorCommands(ElevatorSubsystem elevator) {
        this.elevator = elevator;
    }


    public Command openSampleClaw() {
        return new InstantCommand(() -> elevator.setSampleClawServoAngle(Constants.ElevatorConstants.sampleClawOpenAngle));
    }

    public Command closeSampleClaw() {
        return new InstantCommand(() -> elevator.setSampleClawServoAngle(Constants.ElevatorConstants.sampleClawClosedAngle));
    }

    public Command tipBucket() {
        return new InstantCommand(() -> elevator.setBucketServoAngle(Constants.ElevatorConstants.bucketTippedAngle));
    }

    public Command straightenBucket() {
        return new InstantCommand(() -> elevator.setBucketServoAngle(Constants.ElevatorConstants.bucketUprightAngle));
    }

    public Command deliverHighBasket() {
        return new SequentialCommandGroup(
                new PositionElevator(elevator, Constants.ElevatorConstants.elevatorUpperBasketHeight),
                tipBucket(),
                new WaitCommand(1000),
                straightenBucket(),
                new WaitCommand(1000),
                new PositionElevator(elevator, Constants.ElevatorConstants.elevatorDownHeight));
    }

    public Command deliverLowBasket() {
        return new SequentialCommandGroup(
                new PositionElevator(elevator, Constants.ElevatorConstants.elevatorLowerBasketHeight),
                tipBucket(),
                new WaitCommand(1000),
                straightenBucket(),
                new WaitCommand(1000),
                new PositionElevator(elevator, Constants.ElevatorConstants.elevatorDownHeight));
    }

    public Command preparePickupSample(boolean dropElevator) {
        return new SequentialCommandGroup(
                new ParallelCommandGroup(
                        openSampleClaw(),
                        new PositionElevator(elevator, Constants.ElevatorConstants.elevatorSamplePickupHeight)));
    }

    public Command attachSample() {
        return new SequentialCommandGroup(
                new PositionElevator(elevator, Constants.ElevatorConstants.elevatorSamplePlaceHeight),
                openSampleClaw(),
                new PositionElevator(elevator, Constants.ElevatorConstants.elevatorDownHeight));
    }


}
