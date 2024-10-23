package org.firstinspires.ftc.teamcode.commandsandactions.arm;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;

import org.firstinspires.ftc.teamcode.subsystems.ExtendArmSubsystem;


public class JogArm extends CommandBase {

    private final GamepadEx gamepad;
    private ExtendArmSubsystem arm;


    public JogArm(ExtendArmSubsystem arm, GamepadEx gamepad) {
        this.arm = arm;
        this.gamepad = gamepad;
        addRequirements(this.arm);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {

        arm.power = gamepad.getRightX() / 2;

//        if (arm.power > 0 && arm.getPositionInches() < 50
//
//                || arm.power < 0 && arm.getPositionInches() >2) {
        arm.armMotor.set(arm.power);
//        } else
//            arm.power = 0;


        // arm.armController.setGoal(arm.getPositionInches());
    }

    @Override
    public void end(boolean interrupted) {
        arm.armMotor.set(0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
