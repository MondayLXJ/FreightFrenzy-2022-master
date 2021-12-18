package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;

public class IntakeSubsystem extends SubsystemBase {
    private Motor intakeMotor;

    public IntakeSubsystem(Motor motor){
        this.intakeMotor = motor;
    }

    public void in() {
        intakeMotor.set(1.0);
    }

    public void out() {
        intakeMotor.set(-1.0);
    }

    public void stop() {
        intakeMotor.stopMotor();
    }
}
