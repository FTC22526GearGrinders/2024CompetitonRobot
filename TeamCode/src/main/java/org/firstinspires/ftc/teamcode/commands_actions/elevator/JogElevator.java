package org.firstinspires.ftc.teamcode.commands_actions.elevator;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.controller.wpilibcontroller.SimpleMotorFeedforward;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.subsystems.ElevatorSubsystem;


public class JogElevator extends CommandBase {

    private final Gamepad gamepad;
    private final ElevatorSubsystem elevator;
    public SimpleMotorFeedforward armFF = new SimpleMotorFeedforward(.18, .03);
    private double deadband = .01;
    private double ipsec = 5.;//ips
    private double ff;


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

        double stickValue = -gamepad.right_stick_y;


        double speedips = -gamepad.right_stick_y * ipsec;//5ips

        ff = armFF.calculate(speedips);
        elevator.leftPower = ff;
        elevator.rightPower = ff;

        boolean overrideLimits = gamepad.start;

        ff = stickValue;

        if (overrideLimits || (elevator.leftPower > 0 && elevator.getLeftPositionInches() < Constants.ElevatorConstants.UPPER_POSITION_LIMIT
                || elevator.leftPower < 0 && elevator.getLeftPositionInches() > Constants.ElevatorConstants.LOWER_POSITION_LIMIT)
                || (elevator.rightPower > 0 && elevator.getRightPositionInches() < Constants.ElevatorConstants.UPPER_POSITION_LIMIT
                || elevator.rightPower < 0 && elevator.getRightPositionInches() > Constants.ElevatorConstants.LOWER_POSITION_LIMIT)) {
            elevator.setLeftMotorPower(ff);
            elevator.setRightMotorPower(ff);
        } else {
            elevator.setLeftMotorPower(0);
            elevator.setRightMotorPower(0);
        }

        elevator.setTargetInches(elevator.getLeftPositionInches());
    }

    @Override
    public void end(boolean interrupted) {
        elevator.leftElevatorMotor.set(0);
        gamepad.rumble(250);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
