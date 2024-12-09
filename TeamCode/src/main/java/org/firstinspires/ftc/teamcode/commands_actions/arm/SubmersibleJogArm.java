package org.firstinspires.ftc.teamcode.commands_actions.arm;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.controller.wpilibcontroller.SimpleMotorFeedforward;

import org.firstinspires.ftc.teamcode.subsystems.ExtendArmSubsystem;


public class SubmersibleJogArm extends CommandBase {


    private final double rate;
    public SimpleMotorFeedforward armFF = new SimpleMotorFeedforward(ExtendArmSubsystem.ks, ExtendArmSubsystem.kv, ExtendArmSubsystem.ka);
    private ExtendArmSubsystem arm;


    public SubmersibleJogArm(ExtendArmSubsystem arm, double rate) {
        this.arm = arm;
        this.rate = rate;
        addRequirements(this.arm);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {


        arm.power = rate;//for telemetry


//check inside limits for each direction
        if (arm.power > 0 && arm.getLeftPositionInches() < ExtendArmSubsystem.OUT_POSITION_LIMIT
                && arm.getRightPositionInches() < ExtendArmSubsystem.OUT_POSITION_LIMIT
                || arm.power < 0 && arm.getLeftPositionInches() > ExtendArmSubsystem.IN_POSITION_LIMIT
                && arm.getRightPositionInches() > ExtendArmSubsystem.IN_POSITION_LIMIT) {
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
        arm.power = 0;
        arm.setLeftPower(0);
        arm.setRightPower(0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

}