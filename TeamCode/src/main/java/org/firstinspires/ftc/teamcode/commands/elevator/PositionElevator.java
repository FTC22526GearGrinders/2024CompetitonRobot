package org.firstinspires.ftc.teamcode.commands.elevator;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.ElevatorSubsystem;


public class PositionElevator extends CommandBase {
    private final ElevatorSubsystem elevator;

    private final double targetInches;

    int lpctr;


    public PositionElevator(ElevatorSubsystem elevator, double targetInches) {
        this.elevator = elevator;
        this.targetInches = targetInches;
        addRequirements(this.elevator);
    }

    @Override
    public void initialize() {
        elevator.setNewFFValues();
        elevator.setLeftPositionKp();

        elevator.leftPidController.reset(elevator.getLeftPositionInches());
        elevator.leftPidController.setGoal(targetInches);

        elevator.rightPidController.reset(elevator.getRightPositionInches());
        elevator.rightPidController.setGoal(targetInches);


        lpctr = 0;
    }

    @Override
    public void execute() {
        lpctr++;
        elevator.position();
    }


    @Override
    public void end(boolean interrupted) {
        elevator.leftElevatorMotor.set(0);
        elevator.rightElevatorMotor.set(0);
    }

    @Override
    public boolean isFinished() {
        return lpctr > 5 && elevator.leftPidController.atGoal()  && elevator.rightPidController.atGoal() ||lpctr>250;
    }
}
