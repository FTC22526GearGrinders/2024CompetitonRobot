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
        double temp = elevator.getLeftPositionInches();
        elevator.leftPidController.setGoal(temp);
        temp = elevator.getRightPositionInches();
        elevator.rightPidController.setGoal(temp);
    }

    @Override
    public void execute() {


        if (!elevator.shutDownElevatorPositioning && elevator.getTargetInches() > elevator.minimumHoldHeight
                && elevator.getLeftPositionInches() < elevator.minimumHoldHeight * 3)
            elevator.position();
        else {
            elevator.leftElevatorMotor.set(0);
            elevator.rightElevatorMotor.set(0);
        }


    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
