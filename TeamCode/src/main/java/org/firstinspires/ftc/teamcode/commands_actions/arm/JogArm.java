package org.firstinspires.ftc.teamcode.commands_actions.arm;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.subsystems.ExtendArmSubsystem;


public class JogArm extends CommandBase {


    private ExtendArmSubsystem arm;
    private Gamepad gamepad;
    private double deadband = .01;


    public JogArm(ExtendArmSubsystem arm, Gamepad gamepad) {
        this.arm = arm;
        this.gamepad = gamepad;
        addRequirements(this.arm);
    }

    @Override
    public void initialize() {
        gamepad.rumble(250);
    }

    @Override
    public void execute() {
        arm.power = gamepad.right_stick_x / 2;

        if (Math.abs(arm.power) < deadband)
            arm.power = 0;

        boolean overrideLimits = gamepad.start;
//check inside limits for each direction
        if (overrideLimits || (arm.power > 0 && arm.getLeftPositionInches() < arm.OUT_POSITION_LIMIT
                && arm.getRightPositionInches() < arm.OUT_POSITION_LIMIT
                || arm.power < 0 && arm.getLeftPositionInches() > arm.IN_POSITION_LIMIT
                && arm.getRightPositionInches() > arm.IN_POSITION_LIMIT)) {
            arm.setLeftPower(arm.power);
            arm.setRightPower(arm.power);
        } else {
            arm.setLeftPower(0);
            arm.setRightPower(0);
        }

        arm.setTargetInches(arm.getLeftPositionInches());
    }

    @Override
    public void end(boolean interrupted) {
        arm.setTargetInches(arm.getLeftPositionInches());
        arm.power = 0;
        arm.setLeftPower(0);
        arm.setRightPower(0);
        gamepad.rumble(250);

    }

    @Override
    public boolean isFinished() {
        return false;
    }

}