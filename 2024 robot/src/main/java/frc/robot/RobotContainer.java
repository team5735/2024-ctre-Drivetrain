// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.Supplier;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.drivetrain.BrakeCommand;
import frc.robot.commands.drivetrain.DriveCommand;
import frc.robot.commands.drivetrain.DriveStraightCommand;
import frc.robot.generated.TunerConstants;

public class RobotContainer {
    // private double MaxSpeed = .1; // 6 meters per second desired top speed
    // private double MaxAngularRate = 1.5 * Math.PI; // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final CommandXboxController joystick = new CommandXboxController(0); // My joystick
    private final CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain; // My drivetrain

    private final SendableChooser<Command> m_autoSmartDashboard = AutoBuilder.buildAutoChooser();

    // This is a sin, ignore this
    private static boolean isFieldCentric = false;
    public static Supplier<Boolean> getFieldCentric = () -> isFieldCentric;

    // private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
    //         .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
    //         .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric
    //                                                                  // driving in open loop
    // private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    private final Telemetry logger = new Telemetry(.1);

    private void configureBindings() {
        drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
                new DriveCommand(drivetrain,
                        () -> joystick.getLeftX(),
                        () -> joystick.getLeftY(),
                        () -> {
                            return (joystick.getRightTriggerAxis() - joystick.getLeftTriggerAxis()) * Math.PI;
                        }, () -> joystick.leftStick().getAsBoolean()));

        joystick.a().whileTrue(new BrakeCommand(drivetrain));
        joystick.b().whileTrue(drivetrain
                .applyRequest(
                        () -> point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))));

        // reset the field-centric heading on left bumper press
        joystick.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative()));
        joystick.rightBumper().onTrue(drivetrain.runOnce(() -> {
            isFieldCentric = !isFieldCentric;
        }));

        if (Utils.isSimulation()) {
            drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
        }
        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public RobotContainer() {
        configureBindings();

        SmartDashboard.putData("Choose Auto", m_autoSmartDashboard);
    }

    public Command getAutonomousCommand() {
        // var auto = m_autoSmartDashboard.getSelected();
        // return auto == null ? new BrakeCommand(drivetrain) : auto;
        return new ParallelDeadlineGroup(new WaitCommand(250), new DriveStraightCommand(drivetrain));
    }
}
