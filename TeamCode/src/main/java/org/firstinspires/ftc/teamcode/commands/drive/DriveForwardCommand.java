package org.firstinspires.ftc.teamcode.commands.drive;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.DrivetrainSubsystem;

public class DriveForwardCommand extends CommandBase {
    private DrivetrainSubsystem drivetrainSubsystem;

    public DriveForwardCommand(DrivetrainSubsystem drivetrainSubsystem) {
        this.drivetrainSubsystem = drivetrainSubsystem;
    }

    @Override
    public void execute(){drivetrainSubsystem.forward();}

    @Override
    public void end(boolean interrupted){drivetrainSubsystem.stop();}
}
