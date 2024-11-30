package org.firstinspires.ftc.teamcode.commands_actions.elevator;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.controller.PIDController;
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

    private PIDController leftPIDController = new PIDController(.3, 0, 0);
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

        double stickValue = -gamepad.right_stick_y / 2;
        double speedips = -gamepad.right_stick_y * ipsec;//5ips
        ff = armFF.calculate(speedips);
        double leftPID = leftPIDController.calculate(elevator.leftElevatorEncoder.getCorrectedVelocity());
        double rightPID = rightPIDController.calculate(elevator.leftElevatorEncoder.getCorrectedVelocity());

        elevator.leftPower = ff + leftPID;
        elevator.rightPower = ff + rightPID;

        boolean overrideLimits = gamepad.start;

//        elevator.leftPower = stickValue;
//        elevator.rightPower = stickValue;

        if (overrideLimits || (elevator.leftPower > 0 && elevator.getLeftPositionInches() < Constants.ElevatorConstants.UPPER_POSITION_LIMIT
                || elevator.leftPower < 0 && elevator.getLeftPositionInches() > Constants.ElevatorConstants.LOWER_POSITION_LIMIT)
                || (elevator.rightPower > 0 && elevator.getRightPositionInches() < Constants.ElevatorConstants.UPPER_POSITION_LIMIT
                || elevator.rightPower < 0 && elevator.getRightPositionInches() > Constants.ElevatorConstants.LOWER_POSITION_LIMIT)) {
            elevator.setLeftMotorPower(elevator.leftPower);
            elevator.setRightMotorPower(elevator.leftPower);
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
