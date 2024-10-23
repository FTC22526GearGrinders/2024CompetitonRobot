package org.firstinspires.ftc.teamcode.commandsandactions.arm;

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
        arm.armController.setGoal(arm.getPositionInches());
    }

    @Override
    public void execute() {
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
