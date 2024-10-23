package org.firstinspires.ftc.teamcode.commandsandactions.auto;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.subsystems.ElevatorSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ExtendArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.utils.ActiveMotionValues;

public class AutoActionsSequences extends SequentialCommandGroup {

    public AutoActionsSequences(CommandOpMode opMode, MecanumDriveSubsystem drive, ElevatorSubsystem elevator,
                                ExtendArmSubsystem arm,  AutoFactory af) {

        boolean red = ActiveMotionValues.getRedAlliance();



        addCommands(

                new SequentialCommandGroup(

//                        af.closeGrippers(),
//
//                        af.raiseArmOffFloor(),
//
//                        af.getTeamProp(),

                        af.getAllianceData(opMode, red),

//                        af.buildAndRunTrajectory(),

//                        new ConditionalCommand(
//
//                                new SequentialCommandGroup(
//
//                                        af.raiseArmToPosition(),
//
//                                        new ParallelCommandGroup(
//
//                                                new SequentialCommandGroup(
//
//                                                        af.raiseGripperToBoard(),
//
//                                                        new WaitCommand(1000)),
//
//                                                    //    new InstantCommand(phss::flipGrippersToLeftDown)),
//
//
//
//                                                af.detectTags()),
//
//                                        af.trajToBackboard(),
//
//                                        new WaitCommand(1000),

//                                        new InstantCommand(phss::openLeftGripperWide),
//
//                                        new WaitCommand(1000),
//
//                                        new InstantCommand(phss::flipGrippersToPickup),
//
//                                        new WaitCommand(500),
//
//                                        new InstantCommand(phss::lowerGrippersToPickup),

                                        new WaitCommand(500)));

                                //        af.positionArmHome(),

                                  //      af.parkCheck()));


//                                new DoNothing(),
//
//                                () -> ActiveMotionValues.getBBStart() || !ActiveMotionValues.getBBStart() && ActiveMotionValues.getSecondPixel())))
        ;


    }
}
