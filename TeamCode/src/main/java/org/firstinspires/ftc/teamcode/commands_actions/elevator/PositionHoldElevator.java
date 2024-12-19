package org.firstinspires.ftc.teamcode.commands_actions.elevator;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.ElevatorSubsystem;


public class PositionHoldElevator extends CommandBase {
    private final ElevatorSubsystem elevator;

    public PositionHoldElevator(ElevatorSubsystem elevator) {
        this.elevator = elevator;
        addRequirements(this.elevator);
    }

    @Override
    public void initialize() {
        elevator.posrng = 911;
        double temp = elevator.getLeftPositionInches();
        elevator.setTargetInches(temp);
    }

    @Override
    public void execute() {

        elevator.position();
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
