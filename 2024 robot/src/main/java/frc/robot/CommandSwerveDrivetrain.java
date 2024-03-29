package frc.robot;

import java.util.function.Supplier;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.commands.auto.AutoCommands;

/**
 * Class that extends the Phoenix SwerveDrivetrain class and implements
 * subsystem
 * so it can be used in command-based projects easily.
 */
public class CommandSwerveDrivetrain extends SwerveDrivetrain implements Subsystem {
    private static final double kSimLoopPeriod = 0.005; // 5 ms
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;

    private SwerveRequest.RobotCentric m_robotCentric = new SwerveRequest.RobotCentric();
    private SwerveRequest.FieldCentric m_fieldCentric = new SwerveRequest.FieldCentric();
    private SwerveRequest.SwerveDriveBrake m_brake = new SwerveRequest.SwerveDriveBrake();
    private Supplier<Boolean> m_isFieldCentric;

    public CommandSwerveDrivetrain(SwerveDrivetrainConstants driveTrainConstants, double OdometryUpdateFrequency,
            Supplier<Boolean> fieldCentric, SwerveModuleConstants... modules) {
        super(driveTrainConstants, OdometryUpdateFrequency, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }

        m_isFieldCentric = fieldCentric;

        setupAuto();
    }

    public CommandSwerveDrivetrain(SwerveDrivetrainConstants driveTrainConstants, Supplier<Boolean> fieldCentric,
            SwerveModuleConstants... modules) {
        super(driveTrainConstants, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }

        m_isFieldCentric = fieldCentric;

        setupAuto();
    }

    public void drive(double vx, double vy, double omega, double multiplier) {
        SmartDashboard.putNumber("omega", omega);

        if (m_isFieldCentric.get()) {
            setControl(m_fieldCentric.withVelocityX(vx * multiplier)
                    .withVelocityY(vy * multiplier)
                    .withRotationalRate(omega * multiplier)
                    .withDriveRequestType(DriveRequestType.OpenLoopVoltage));
            return;
        }

        setControl(m_robotCentric.withVelocityX(vx * multiplier)
                .withVelocityY(vy * multiplier)
                .withRotationalRate(omega * multiplier)
                .withDriveRequestType(DriveRequestType.OpenLoopVoltage));
    }

    public void drive(double vx, double vy, double omega) {
        drive(vx, vy, omega, 1);
    }

    public void driveClosedLoop(double vx, double vy, double omega) {
        if (m_isFieldCentric.get()) {
            setControl(m_fieldCentric.withVelocityX(vx)
                    .withVelocityY(vy)
                    .withRotationalRate(omega)
                    .withDriveRequestType(DriveRequestType.Velocity)
                    .withDeadband(.1));
            return;
        }

        setControl(m_robotCentric.withVelocityX(vx)
                .withVelocityY(vy)
                .withRotationalRate(omega)
                .withDriveRequestType(DriveRequestType.Velocity)
                .withDeadband(.1));
    }

    public void brake() {
        setControl(m_brake);
    }

    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> this.setControl(requestSupplier.get()));
    }

    private void startSimThread() {
        m_lastSimTime = Utils.getCurrentTimeSeconds();

        /* Run simulation at a faster rate so PID gains behave more reasonably */
        m_simNotifier = new Notifier(() -> {
            final double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - m_lastSimTime;
            m_lastSimTime = currentTime;

            /* use the measured time delta, get battery voltage from WPILib */
            updateSimState(deltaTime, RobotController.getBatteryVoltage());
        });
        m_simNotifier.startPeriodic(kSimLoopPeriod);
    }

    private Pose2d getEstimatedPosition() {
        m_stateLock.readLock().lock();
        var pose = m_odometry.getEstimatedPosition();
        m_stateLock.readLock().unlock();

        return pose;
    }

    // helper function because the resetPose passed to Pathplanner is only given a
    // pose
    private void setPose(Pose2d pose) {
        if (!odometryIsValid())
            return;
        m_stateLock.writeLock().lock();
        m_odometry.resetPosition(m_pigeon2.getRotation2d(), m_modulePositions, pose);
        m_stateLock.writeLock().unlock();
    }

    // helper function to put all speed values together into an array
    private ChassisSpeeds getChassisSpeeds() {
        var states = new SwerveModuleState[4];

        m_stateLock.readLock().lock();
        for (int i = 0; i < Modules.length; i++) {
            states[i] = Modules[i].getCurrentState();
        }
        m_stateLock.readLock().unlock();

        return m_kinematics.toChassisSpeeds(states);
    }

    // Converts from ChassisSpeeds to a swerve request for Pathplanner
    private void autoDriveRobotRelative(ChassisSpeeds robotChassisSpeeds) {
        var discrete = ChassisSpeeds.discretize(robotChassisSpeeds, 1.0 / 20.0); // TODO: is this the right frequency?

        // var targetStates = m_kinematics.toSwerveModuleStates(discrete);

        setControl(m_robotCentric
                .withVelocityX(discrete.vxMetersPerSecond)
                .withVelocityY(discrete.vyMetersPerSecond)
                .withRotationalRate(discrete.omegaRadiansPerSecond));
    }

    // Returns the config for pathplanner use
    private HolonomicPathFollowerConfig getConfig() {
        var drivePid = new PIDConstants(15, 0, 0);
        var turnPid = new PIDConstants(1, 0, 0);

        return new HolonomicPathFollowerConfig(drivePid,
                turnPid,
                7.27, // TODO: this number is Cookiezi. Please give a non-arbitrary value
                Math.sqrt(9.75 * 9.75 + 12.5 * 12.5), // radius (wtf)
                new ReplanningConfig());
    }

    private void setupAuto() {
        AutoBuilder.configureHolonomic(
                this::getEstimatedPosition,
                this::setPose,
                this::getChassisSpeeds,
                this::autoDriveRobotRelative,
                getConfig(),
                () -> {
                    return (DriverStation.getAlliance().isPresent() &&
                            DriverStation.getAlliance().get() == DriverStation.Alliance.Red);
                },
                this);
    }

    @Override
    public void periodic() {
        for (SwerveModule m : Modules) {
            SmartDashboard.putNumber("Module " + m.getDriveMotor().getDeviceID() / 2 + " speed",
                    Math.abs(m.getCurrentState().speedMetersPerSecond));
        }

        SmartDashboard.putNumber("Pigeon rotation speed", m_pigeon2.getRate());
        SmartDashboard.putNumber("Pigeon reported pose angle", m_pigeon2.getAngle());
    }
}
