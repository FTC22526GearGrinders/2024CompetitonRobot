package org.firstinspires.ftc.teamcode.commandsandactions.auto;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.subsystems.ElevatorSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ExtendArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;

public class AutoFactory extends CommandBase {
    MecanumDriveSubsystem drive;
    ElevatorSubsystem elevator;
    ExtendArmSubsystem arm;

    CommandOpMode opMode;


    boolean doLogging = true;


    public AutoFactory(CommandOpMode opmode, MecanumDriveSubsystem drive,
                       ElevatorSubsystem elevator, ExtendArmSubsystem arm) {
        this.opMode = opmode;
        this.drive = drive;
        this.elevator = elevator;
        this.arm = arm;
    }


    public Command getAllianceData(CommandOpMode opmode, boolean red) {
        return new ConditionalCommand(
                new SelectMotionValuesRed(opmode),
                new SelectMotionValuesBlue(opmode),
                () -> red);
    }


    public Command getBasketSequences() {
        return new WaitCommand(10);
    }

    public Command getSamplesSequences() {
        return new WaitCommand(1);
    }

//    public Command buildAndRunTrajectory() {
//
//        return new SequentialCommandGroup(
//                new SelectAndBuildTrajectory(opMode, drive, phss),
//                new ShowTrajectoryInfo(drive, opMode),
//                new RunTrajSequence(drive, opMode));
//    }


}