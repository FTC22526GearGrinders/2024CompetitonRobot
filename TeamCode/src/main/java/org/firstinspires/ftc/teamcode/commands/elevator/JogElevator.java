package org.firstinspires.ftc.teamcode.commands.elevator;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;

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

        elevator.power = -gamepad.getRightY() / 2;

//        if (rotateArm.power > 0 && rotateArm.getPositionDegrees() < Constants.ArmConstants.UPPER_POSITION_LIMIT
//                || rotateArm.power < 0 && rotateArm.getPositionDegrees() > Constants.ArmConstants.LOWER_POSITION_LIMIT) {
//        elevator.setPower(elevator.power);
//        } else
//            rotateArm.power = 0;

        elevator.setPower(elevator.power);

        elevator.setTargetInches(elevator.getPositionInches());
    }

    @Override
    public void end(boolean interrupted) {
        elevator.elevatorMotor.set(0);

    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
