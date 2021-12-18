package org.firstinspires.ftc.teamcode.subsystems;/* package org.firstinspires.ftc.teamcode.subsystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.drive.TankDrive;
import com.acmerobotics.roadrunner.followers.TankPIDVAFollower;
import com.acmerobotics.roadrunner.followers.TrajectoryFollower;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TankVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.acmerobotics.roadrunner.util.NanoClock;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_ACCEL;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_ANG_VEL;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_VEL;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MOTOR_VELO_PID;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.RUN_USING_ENCODER;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.TRACK_WIDTH;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.kA;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.kStatic;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.kV;

@Config
class SampleTankDrive extends TankDrive {
    public static PIDCoefficients AXIAL_PID = new PIDCoefficients(0, 0, 0);
    public static PIDCoefficients CROSS_TRACK_PID = new PIDCoefficients(0, 0, 0);
    public static PIDCoefficients HEADING_PID = new PIDCoefficients(0, 0, 0);

    public static double VX_WEIGHT = 1;
    public static double OMEGA_WEIGHT = 1;

    public enum Mode {
        IDLE,
        TURN,
        FOLLOW_TRAJECTORY
    }


    private FtcDashboard dashboard;
    private NanoClock clock;

    private Mode mode;

    private PIDFController turnController;
    private MotionProfile turnProfile;
    private double turnStart;

    private TrajectoryVelocityConstraint velConstraint;
    private TrajectoryAccelerationConstraint accelConstraint;
    private TrajectoryFollower follower;

    private List<Pose2d> poseHistory;

    private List<DcMotorEx> motors, leftMotors, rightMotors;
    //MIGHT HAVE TO CHANGE I DUNNO, PLS CHECK BEFORE RUNNING
    private BNO055IMU imu;

    private VoltageSensor batteryVoltageSensor;

    public SampleTankDrive(double kV, double kA, double kStatic, double trackWidth) {
        super(kV, kA, kStatic, trackWidth);
    }

    public SampleTankDrive(HardwareMap hardwareMap){
        super(kV, kA, kStatic, TRACK_WIDTH);

        dashboard = FtcDashboard.getInstance();
        dashboard.setTelemetryTransmissionInterval(25);

        clock = NanoClock.system();

        mode = Mode.IDLE;

        turnController = new PIDFController(HEADING_PID);
        turnController.setInputBounds(0, 2 * Math.PI);

        velConstraint = new MinVelocityConstraint(Arrays.asList(
                new AngularVelocityConstraint(MAX_ANG_VEL),
                new TankVelocityConstraint(MAX_VEL, TRACK_WIDTH)
        ));

        accelConstraint = new ProfileAccelerationConstraint(MAX_ACCEL);
        follower = new TankPIDVAFollower(AXIAL_PID, CROSS_TRACK_PID,
                new Pose2d(0.5, 0.5, Math.toRadians(5.0)), 0.5);

        poseHistory = new ArrayList<>();

        org.firstinspires.ftc.teamcode.util.LynxModuleUtil.ensureMinimumFirmwareVersion(hardwareMap);

        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();

        for (LynxModule module : hardwareMap.getAll(LynxModule.class)){
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        //IMU STUFF PLS CROSS CHECK THIS BEFORE RUNNING
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);

        //MOTORS. ALSO FIGURE IT OUT
        DcMotorEx leftMiddle = hardwareMap.get(DcMotorEx.class, "leftMiddle");
        DcMotorEx rightMiddle = hardwareMap.get(DcMotorEx.class, "rightMiddle");

        motors = Arrays.asList(leftMiddle, rightMiddle);
        leftMotors = Arrays.asList(leftMiddle);
        rightMotors = Arrays.asList(rightMiddle);

        for (DcMotorEx motor: motors){
            MotorConfigurationType motorConfigurationType = motor.getMotorType();
            motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
            motor.setMotorType(motorConfigurationType);
        }

        //IF ENCODER. WEIRD ERROR FIX THIS
        if (RUN_USING_ENCODER){
            setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        if (RUN_USING_ENCODER && MOTOR_VELO_PID != null){
            setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, MOTOR_VELO_PID);
        }




        public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior zeroPowerBehavior){
            for (DcMotorEx motor: motors) {
                motor.setZeroPowerBehavior(zeroPowerBehavior);
            }
        }

        public void setMode(){
            for (DcMotorEx motor: motors){
                motor.setMode(runMode);
            }
        }

        public TrajectoryBuilder trajectoryBuilder(Pose2d startPose){
            return new TrejectoryBuilder
        }


    }


    @Override
    protected double getRawExternalHeading() {
        return 0;
    }

    @NonNull
    @Override
    public List<Double> getWheelPositions() {
        return null;
    }

    @Override
    public void setMotorPowers(double v, double v1) {

    }
}
*/

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;

public class DrivetrainSubsystem extends SubsystemBase {

    private Motor leftMotorFront;
    private Motor leftMotorBack;
    private Motor rightMotorFront;
    private Motor rightMotorBack;


    public DrivetrainSubsystem(Motor leftMotorFront, Motor rightMotorFront) {
        //this.leftMotorBack = leftMotorBack;
        this.leftMotorFront = leftMotorFront;
        //this.rightMotorBack = rightMotorBack;
        this.rightMotorFront = rightMotorFront;
    }

    public void forward() {
        leftMotorFront.set(1.0);
        //leftMotorBack.set(1.0);
        rightMotorFront.set(1.0);
        //rightMotorBack.set(1.0);
    }

    public void backward() {
        leftMotorFront.set(-1.0);
        //leftMotorBack.set(-1.0);
        rightMotorFront.set(-1.0);
        //rightMotorBack.set(-1.0);
    }


    public void left() {
        leftMotorFront.set(-1.0);
        //leftMotorBack.set(1.0);
        rightMotorFront.set(1.0);
        //rightMotorBack.set(-1.0);
    }

    public void right() {
        leftMotorFront.set(1.0);
        //leftMotorBack.set(-1.0);
        rightMotorFront.set(-1.0);
        //rightMotorBack.set(1.0);
    }

    public void stop() {
        //leftMotorBack.stopMotor();
        //rightMotorBack.stopMotor();
        leftMotorFront.stopMotor();
        rightMotorFront.stopMotor();
    }
}
