package org.firstinspires.ftc.teamcode.commands_actions.arm;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.controller.wpilibcontroller.SimpleMotorFeedforward;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.subsystems.ExtendArmSubsystem;


public class JogArm extends CommandBase {


    public SimpleMotorFeedforward armFF = new SimpleMotorFeedforward(.18, .03);
    private ExtendArmSubsystem arm;
    private Gamepad gamepad;
    private double deadband = .01;
    private double ipsec = 5.;//ips
    private double ff;


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

        double speedips = gamepad.right_stick_x * ipsec;//5ips

        ff = armFF.calculate(speedips);

        arm.power = speedips;//for telemetry

        boolean overrideLimits = gamepad.start;
//check inside limits for each direction
        if (overrideLimits || (ff > 0 && arm.getLeftPositionInches() < Constants.ExtendArmConstants.OUT_POSITION_LIMIT
                && arm.getRightPositionInches() < Constants.ExtendArmConstants.OUT_POSITION_LIMIT
                || ff < 0 && arm.getLeftPositionInches() > Constants.ExtendArmConstants.IN_POSITION_LIMIT
                && arm.getRightPositionInches() > Constants.ExtendArmConstants.IN_POSITION_LIMIT)) {
            arm.setLeftPower(ff);
            arm.setRightPower(ff);
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
        gamepad.rumble(250);
//        ExtendArmSubsystem.targetInches = arm.getLeftPositionInches();
//        arm.setTargetInches(ExtendArmSubsystem.targetInches);


    }

    @Override
    public boolean isFinished() {
        return false;
    }

}