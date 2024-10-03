package org.firstinspires.ftc.teamcode.commands.elevator;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.ElevatorSubsystem;


public class PositionElevator extends CommandBase {
    private final ElevatorSubsystem elevator;

    private final double targetInches;

    int lpctr;

    private double accelLeft;
    private double accelRight;

    private double lastLeftVel;
    private double lastRightVel;


    public PositionElevator(ElevatorSubsystem elevator, double targetInches) {
        this.elevator = elevator;
        this.targetInches = targetInches;
        addRequirements(this.elevator);
    }

    @Override
    public void initialize() {
        elevator.setNewFFValues();
        elevator.setPositionKp();

        elevator.pidController.reset(elevator.getPositionInches());
        elevator.pidController.setGoal(targetInches);


        lpctr = 0;
    }

    @Override
    public void execute() {
        lpctr++;
        elevator.position();
    }


    @Override
    public void end(boolean interrupted) {
        elevator.elevatorMotor.set(0);
       // elevator.rightElevatorMotor.set(0);
    }

    @Override
    public boolean isFinished() {
        return lpctr > 5 && elevator.pidController.atGoal()  ||lpctr>250;
    }
}
