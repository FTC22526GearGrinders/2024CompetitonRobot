package org.firstinspires.ftc.teamcode.commands_actions.elevator;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.wpilibcontroller.SimpleMotorFeedforward;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.subsystems.ElevatorSubsystem;


public class JogElevator extends CommandBase {

    private final Gamepad gamepad;
    private final ElevatorSubsystem elevator;
    private final double deadband = .01;
    public SimpleMotorFeedforward armFF = new SimpleMotorFeedforward(ElevatorSubsystem.eks, ElevatorSubsystem.ekv, ElevatorSubsystem.eka);
    private double ipsec = 5;//ips
    private double ff;

    private PIDController leftPIDController = new PIDController(.2, 0, 0);
    private PIDController rightPIDController = new PIDController(.2, 0, 0);


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


        double speedips = -gamepad.left_stick_y * ipsec;

        if (Math.abs(stickValue) < deadband) stickValue = 0;

        ff = armFF.calculate(speedips);

        double leftPID = leftPIDController.calculate(elevator.leftElevatorEncoder.getRate(), speedips);
        double rightPID = rightPIDController.calculate(elevator.leftElevatorEncoder.getRate(), speedips);

        boolean useFF = false;

        if (useFF) {
            elevator.leftPower = ff + leftPID;
            elevator.rightPower = ff + rightPID;
        } else {
//using gamepad direct
            elevator.leftPower = stickValue;
            elevator.rightPower = stickValue;
        }
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
        elevator.leftElevatorMotor.set(0);
        gamepad.rumble(250);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
