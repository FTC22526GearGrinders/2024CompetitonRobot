package org.firstinspires.ftc.teamcode.commands.arm;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.subsystems.ExtendArmSubsystem;


public class AutoArmCommands extends CommandBase {


    private final ExtendArmSubsystem arm;

    public AutoArmCommands(ExtendArmSubsystem arm) {
        this.arm = arm;
    }

    public Command openIntakeClaw() {
        return new InstantCommand(() -> arm.setClawServoAngle(Constants.ArmConstants.intakeClawOpenAngle));
    }

    public Command closeIntakeClaw() {
        return new InstantCommand(() -> arm.setClawServoAngle(Constants.ArmConstants.intakeClawClosedAngle));
    }

    public Command tiltToIntakePosition() {
        return new InstantCommand(() -> arm.setTiltServoAngle(Constants.ArmConstants.intakeTiltDownAngle));
    }

    public Command tiltToClearPosition() {
        return new InstantCommand(() -> arm.setTiltServoAngle(Constants.ArmConstants.intakeTiltClearAngle));
    }

    public Command tiltToBucketDeliverPosition() {
        return new InstantCommand(() -> arm.setTiltServoAngle(Constants.ArmConstants.intakeTiltBucketDeliverAngle));
    }

    public Command prepareToPickup() {
        return new SequentialCommandGroup(
                new PositionArm(arm, Constants.ArmConstants.pickupDistance),
                openIntakeClaw(),
                tiltToIntakePosition());
    }

    public Command pickup() {
        return new SequentialCommandGroup(
                closeIntakeClaw(),
                new WaitCommand(1000),
                tiltToClearPosition());
    }

    public Command deliverToBucket() {
        return new SequentialCommandGroup(
                new PositionArm(arm, Constants.ArmConstants.bucketDistance),
                tiltToBucketDeliverPosition(),
                openIntakeClaw(),
                tiltToClearPosition());
    }



}
