package org.firstinspires.ftc.teamcode.commands.drive;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.DrivetrainSubsystem;

public class DriveBackwardCommand extends CommandBase {
    private DrivetrainSubsystem drivetrainSubsystem;

    public DriveBackwardCommand(DrivetrainSubsystem drivetrainSubsystem) {
        this.drivetrainSubsystem = drivetrainSubsystem;
    }

    @Override
    public void execute(){drivetrainSubsystem.backward();}

    @Override
    public void end(boolean interrupted){drivetrainSubsystem.stop();}
}
