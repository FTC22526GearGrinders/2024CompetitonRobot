package org.firstinspires.ftc.teamcode.commands_actions.elevator;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.subsystems.ElevatorSubsystem;


public class JogElevator extends CommandBase {

    private final Gamepad gamepad;
    private final ElevatorSubsystem elevator;
    private final double deadband = .01;

    public JogElevator(ElevatorSubsystem elevator, Gamepad gamepad) {
        this.elevator = elevator;
        this.gamepad = gamepad;
        addRequirements(this.elevator);
    }

    @Override
    public void initialize() {
        gamepad.rumble(250);
    }

    @Override
    public void execute() {

        double stickValue = -gamepad.left_stick_y;

        if (Math.abs(stickValue) < deadband) stickValue = 0;

        elevator.leftPower = stickValue;
        elevator.rightPower = stickValue;

        boolean overrideLimits = gamepad.start;

        if (overrideLimits || (elevator.leftPower > 0 && elevator.getLeftPositionInches() < elevator.UPPER_POSITION_LIMIT
                || elevator.leftPower < 0 && elevator.getLeftPositionInches() > elevator.LOWER_POSITION_LIMIT)
                || (elevator.rightPower > 0 && elevator.getRightPositionInches() < elevator.UPPER_POSITION_LIMIT
                || elevator.rightPower < 0 && elevator.getRightPositionInches() > elevator.LOWER_POSITION_LIMIT)) {
            elevator.setLeftMotorPower(elevator.leftPower);
            elevator.setRightMotorPower(elevator.rightPower);
        } else {
            elevator.setLeftMotorPower(0);
            elevator.setRightMotorPower(0);
        }

        elevator.setTargetInches(elevator.getLeftPositionInches());
    }

    @Override
    public void end(boolean interrupted) {
        elevator.setTargetInches(elevator.getLeftPositionInches());
        elevator.leftElevatorMotor.set(0);
        elevator.rightElevatorMotor.set(0);
        gamepad.rumble(250);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
