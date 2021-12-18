package org.firstinspires.ftc.teamcode.commands.arm;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;

public class DumpArmCommand extends CommandBase {
    private ArmSubsystem armSubsystem;

    public DumpArmCommand(ArmSubsystem armSubsystem){
        this.armSubsystem = armSubsystem;
    }

    @Override
    public void execute(){ this.armSubsystem.dumpArm(); }
}
