package org.firstinspires.ftc.teamcode.commands_actions.arm;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.ExtendArmSubsystem;


public class PositionHoldArm extends CommandBase {
    private final ExtendArmSubsystem arm;


    public PositionHoldArm(ExtendArmSubsystem arm) {
        this.arm = arm;
        addRequirements(this.arm);
    }

    @Override
    public void initialize() {
        double temp = arm.getLeftPositionInches();
        arm.setTargetInches(temp);
    }

    @Override
    public void execute() {
        arm.holdCtr++;
        arm.position();
    }


    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return false;
    }

}
