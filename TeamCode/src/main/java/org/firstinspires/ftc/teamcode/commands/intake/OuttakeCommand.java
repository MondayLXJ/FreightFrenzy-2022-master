package org.firstinspires.ftc.teamcode.commands.intake;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;

public class OuttakeCommand extends CommandBase {
    private IntakeSubsystem intakeSubsystem;

    public OuttakeCommand(IntakeSubsystem intakeSubsystem) {
        this.intakeSubsystem = intakeSubsystem;
    }

    @Override
    public void execute(){intakeSubsystem.out();}

    @Override
    public void end(boolean interrupted){intakeSubsystem.stop();}
}
