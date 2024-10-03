package org.firstinspires.ftc.teamcode.commands.elevator;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.ElevatorSubsystem;


public class PositionHoldElevator extends CommandBase {
    private final ElevatorSubsystem elevator;

    private double power;


    public PositionHoldElevator(ElevatorSubsystem rotateArm) {
        this.elevator = rotateArm;
        addRequirements(this.elevator);
    }

    @Override
    public void initialize() {
        double temp = elevator.getPositionInches();
        elevator.pidController.setGoal(temp);
    }

    @Override
    public void execute() {
        elevator.holdCtr++;
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
