package org.firstinspires.ftc.teamcode.commands_actions.elevator;

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
        double temp = elevator.getLeftPositionInches();
        elevator.leftPidController.setGoal(temp);
        temp = elevator.getRightPositionInches();
        elevator.rightPidController.setGoal(temp);
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
