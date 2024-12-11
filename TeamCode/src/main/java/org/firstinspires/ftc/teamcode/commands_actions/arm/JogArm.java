package org.firstinspires.ftc.teamcode.commands_actions.arm;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.wpilibcontroller.SimpleMotorFeedforward;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.subsystems.ExtendArmSubsystem;


public class JogArm extends CommandBase {


    public SimpleMotorFeedforward armFF = new SimpleMotorFeedforward(ExtendArmSubsystem.ks, ExtendArmSubsystem.kv, ExtendArmSubsystem.ka);
    private ExtendArmSubsystem arm;
    private Gamepad gamepad;
    private double deadband = .01;
    private double ipsec = 5.;//ips
    private double ff;
    private PIDController armPIDController = new PIDController(.2, 0, 0);

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

        double armPID = armPIDController.calculate(arm.getLeftInchesPerSec(), speedips);

        arm.power = ff + armPID;//for telemetry

        //using gamepad direct

        arm.power = gamepad.right_stick_x / 2;

        boolean overrideLimits = gamepad.start;
//check inside limits for each direction
        if (overrideLimits || (arm.power > 0 && arm.getLeftPositionInches() < ExtendArmSubsystem.OUT_POSITION_LIMIT
                && arm.getRightPositionInches() < ExtendArmSubsystem.OUT_POSITION_LIMIT
                || arm.power < 0 && arm.getLeftPositionInches() > ExtendArmSubsystem.IN_POSITION_LIMIT
                && arm.getRightPositionInches() > ExtendArmSubsystem.IN_POSITION_LIMIT)) {
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