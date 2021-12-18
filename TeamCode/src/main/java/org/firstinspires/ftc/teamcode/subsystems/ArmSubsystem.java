package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.Servo;

public class ArmSubsystem extends SubsystemBase {
    private Servo arm;
    private boolean dumping = false;

    public ArmSubsystem(Servo arm){ this.arm = arm; }

    public void resetArm(){
        arm.setPosition(.1);
        this.dumping = false;
    }

    public boolean isDumping() {
        return this.dumping;
    }

    public void dumpArm(){
        arm.setPosition(.75);
        this.dumping = true;
    }
}
