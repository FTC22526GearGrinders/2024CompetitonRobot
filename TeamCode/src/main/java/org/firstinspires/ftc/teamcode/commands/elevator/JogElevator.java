package org.firstinspires.ftc.teamcode.commands.elevator;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.subsystems.ElevatorSubsystem;


public class JogElevator extends CommandBase {

    private final GamepadEx gamepad;
    private final ElevatorSubsystem elevator;

    public JogElevator(ElevatorSubsystem elevator, GamepadEx gamepad) {
        this.elevator = elevator;
        this.gamepad = gamepad;
        addRequirements(this.elevator);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {

        elevator.leftPower = -gamepad.getRightY() / 2;
        elevator.rightPower = -gamepad.getRightY() / 2;
        if (elevator.leftPower > 0 && elevator.getLeftPositionInches() < Constants.ElevatorConstants.UPPER_POSITION_LIMIT
                || elevator.leftPower < 0 && elevator.getLeftPositionInches() > Constants.ArmConstants.LOWER_POSITION_LIMIT) {
            elevator.setLeftMotorPower(elevator.leftPower);
        } else
            elevator.leftPower = 0;
        elevator.setLeftMotorPower(elevator.leftPower);

        if (elevator.rightPower > 0 && elevator.getRightPositionInches() < Constants.ElevatorConstants.UPPER_POSITION_LIMIT
                || elevator.rightPower < 0 && elevator.getRightPositionInches() > Constants.ArmConstants.LOWER_POSITION_LIMIT) {
            elevator.setRightMotorPower(elevator.rightPower);
        } else
            elevator.rightPower = 0;
        elevator.setRightMotorPower(elevator.rightPower);



        elevator.setTargetInches(elevator.getLeftPositionInches());
    }

    @Override
    public void end(boolean interrupted) {
        elevator.leftElevatorMotor.set(0);

    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
