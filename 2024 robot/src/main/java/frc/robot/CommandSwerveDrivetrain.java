package frc.robot;

import java.util.function.Supplier;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.generated.TunerConstants;

/**
 * Class that extends the Phoenix SwerveDrivetrain class and implements
 * subsystem
 * so it can be used in command-based projects easily.
 */
public class CommandSwerveDrivetrain extends SwerveDrivetrain implements Subsystem {
    private static final double kSimLoopPeriod = 0.005; // 5 ms
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;

    private SwerveRequest.RobotCentric m_autoDrive; // DO NOT DRIVE WITH THIS OTHERWISE... before I refactor!

    public CommandSwerveDrivetrain(SwerveDrivetrainConstants driveTrainConstants, double OdometryUpdateFrequency,
            SwerveModuleConstants... modules) {
        super(driveTrainConstants, OdometryUpdateFrequency, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }

        setupAuto();
    }

    public CommandSwerveDrivetrain(SwerveDrivetrainConstants driveTrainConstants, SwerveModuleConstants... modules) {
        super(driveTrainConstants, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
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

    // helper function because the resetPose passed to Pathplanner is only given a
    // pose
    private void setPose(Pose2d pose) {
        m_odometry.resetPosition(m_pigeon2.getRotation2d(), m_modulePositions, pose);
    }

    // helper function to put all speed values together into an array
    private ChassisSpeeds getChassisSpeeds() {
        var states = new SwerveModuleState[4];
        for (int i = 0; i < Modules.length; i++) {
            states[i] = Modules[i].getCurrentState();
        }

        return m_kinematics.toChassisSpeeds(states);
    }

    // Converts from ChassisSpeeds to a swerve request for Pathplanner
    private void autoDriveRobotRelative(ChassisSpeeds robotChassisSpeeds) {
        var discrete = ChassisSpeeds.discretize(robotChassisSpeeds, 1.0 / 250.0); // TODO: is this the right frequency?

        var targetStates = m_kinematics.toSwerveModuleStates(discrete);

        applyRequest(() -> {
            return m_autoDrive
                    .withVelocityX(discrete.vxMetersPerSecond)
                    .withVelocityY(discrete.vyMetersPerSecond)
                    .withRotationalRate(discrete.omegaRadiansPerSecond);
        });
    }

    // Returns the config for pathplanner use
    private HolonomicPathFollowerConfig getConfig() {
        var driveCfg = TunerConstants.driveGains;
        var turnCfg = TunerConstants.steerGains;

        var drivePid = new PIDConstants(driveCfg.kP, driveCfg.kI, driveCfg.kD);
        var turnPid = new PIDConstants(turnCfg.kP, turnCfg.kI, turnCfg.kD);

        double SQRT_2 = 1.4142135623730951455;
        return new HolonomicPathFollowerConfig(drivePid,
                turnPid,
                4.5, // max speed
                SQRT_2 * 12.25, // radius (wtf)
                new ReplanningConfig());
    }

    private void setupAuto() {
        AutoBuilder.configureHolonomic(
            m_odometry::getEstimatedPosition,
            this::setPose,
            this::getChassisSpeeds,
            this::autoDriveRobotRelative,
            getConfig(),
            () -> {
                return (DriverStation.getAlliance().isPresent() && 
                           DriverStation.getAlliance().get() == DriverStation.Alliance.Red);
                        },
            (Subsystem) this
        );
    }
}
