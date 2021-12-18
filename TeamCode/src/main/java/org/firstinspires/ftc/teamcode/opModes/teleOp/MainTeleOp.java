package org.firstinspires.ftc.teamcode.opModes.teleOp;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.commands.arm.DumpArmCommand;
import org.firstinspires.ftc.teamcode.commands.arm.ResetArmCommand;
import org.firstinspires.ftc.teamcode.commands.drive.DriveBackwardCommand;
import org.firstinspires.ftc.teamcode.commands.drive.DriveForwardCommand;
import org.firstinspires.ftc.teamcode.commands.intake.IntakeCommand;
import org.firstinspires.ftc.teamcode.commands.intake.OuttakeCommand;
import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DrivetrainSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;

@TeleOp
public class MainTeleOp extends CommandOpMode {
    //motors
    private Motor intakeMotor,  leftMotorFront, leftMotorBack, rightMotorFront,  rightMotorBack;
    private Servo armServo;

    //subsystems
    private IntakeSubsystem intakeSubsystem;
    private DrivetrainSubsystem drivetrainSubsystem;
    private ArmSubsystem armSubsystem;

    //commands
    private IntakeCommand intakeCommand;
    private OuttakeCommand outtakeCommand;
    private DumpArmCommand dumpArmCommand;
    private ResetArmCommand resetArmCommand;
    private DriveForwardCommand driveForwardCommand;
    private DriveBackwardCommand driveBackwardCommand;

    //gamepads
    private GamepadEx driver;

    @Override
    public void initialize() {
        this.intakeMotor = new Motor(hardwareMap, "intake");
        this.leftMotorFront = new Motor(hardwareMap, "leftMotor");
        this.rightMotorFront = new Motor(hardwareMap, "rightMotor");
        //this.armServo = hardwareMap.get(Servo.class, "armServo");

        this.intakeSubsystem = new IntakeSubsystem(this.intakeMotor);
        this.drivetrainSubsystem = new DrivetrainSubsystem(this.leftMotorFront,this.rightMotorFront);
        //this.drivetrainSubsystem = new DrivetrainSubsystem(this.leftMotorFront, this.rightMotorFront);
        //this.armSubsystem = new ArmSubsystem(this.armServo);

        this.intakeCommand = new IntakeCommand(this.intakeSubsystem);
        this.outtakeCommand = new OuttakeCommand(this.intakeSubsystem);
        this.driveForwardCommand = new DriveForwardCommand(this.drivetrainSubsystem);
        this.driveBackwardCommand = new DriveBackwardCommand(this.drivetrainSubsystem);

        //this.dumpArmCommand = new DumpArmCommand(this.armSubsystem);
        //this.resetArmCommand = new ResetArmCommand(this.armSubsystem);
        driver = new GamepadEx(gamepad1);

        driver.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenHeld(this.intakeCommand);
        driver.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenHeld(this.outtakeCommand);
        driver.getGamepadButton(GamepadKeys.Button.Y).whenHeld(this.driveForwardCommand);
        driver.getGamepadButton(GamepadKeys.Button.X).whenHeld(this.driveBackwardCommand);
        //driver.getGamepadButton(GamepadKeys.Button.A).whenPressed(
        //        armSubsystem.isDumping() ? this.resetArmCommand : this.dumpArmCommand
        //);
    }
}