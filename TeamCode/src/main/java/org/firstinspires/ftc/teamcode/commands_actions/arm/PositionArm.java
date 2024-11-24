package org.firstinspires.ftc.teamcode.commands_actions.arm;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.ExtendArmSubsystem;


public class PositionArm extends CommandBase {
    private final ExtendArmSubsystem arm;

    private final double targetInches;

    int lpctr;

    public PositionArm(ExtendArmSubsystem arm, double targetInches) {
        this.arm = arm;
        this.targetInches = targetInches;
        addRequirements(this.arm);
    }

    @Override
    public void initialize() {
        arm.leftArmController.setGoal(targetInches);
        lpctr = 0;
    }

    @Override
    public void execute() {
        lpctr++;
     arm.position();

    }


    @Override
    public void end(boolean interrupted) {
        arm.leftArmMotor.set(0);
    }

    @Override
    public boolean isFinished() {
        return lpctr > 5 && arm.leftArmController.atGoal();
    }
}