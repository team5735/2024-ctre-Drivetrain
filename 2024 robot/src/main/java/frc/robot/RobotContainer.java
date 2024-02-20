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
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.auto.AutoCommands;
import frc.robot.commands.drivetrain.BrakeCommand;
import frc.robot.commands.drivetrain.DriveCommand;
import frc.robot.commands.drivetrain.DriveStraightCommand;
import frc.robot.commands.drivetrain.Turn90Command;
import frc.robot.generated.TunerConstants;

public class RobotContainer {
    // private double MaxSpeed = .1; // 6 meters per second desired top speed
    // private double MaxAngularRate = 1.5 * Math.PI; // 3/4 of a rotation per
    // second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final CommandXboxController joystick = new CommandXboxController(0); // My joystick
    private final CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain; // My drivetrain

    private final SendableChooser<Command> m_autoSmartDashboard = AutoBuilder.buildAutoChooser();

    // This is a sin, ignore this
    private static boolean isFieldCentric = true;
    public static Supplier<Boolean> getFieldCentric = () -> isFieldCentric;

    // private final SwerveRequest.FieldCentric drive = new
    // SwerveRequest.FieldCentric()
    // .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) //
    // Add a 10% deadband
    // .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want
    // field-centric
    // // driving in open loop
    // private final SwerveRequest.SwerveDriveBrake brake = new
    // SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    private final Telemetry logger = new Telemetry(.1);

    private double turboMultiplier = 10;
    private double normalMultiplier = 2;
    private double slowMultiplier = 1;

    private static final double deadband = 0.1;

    private static double deadband(double input) {
        if (Math.abs(input) <= deadband) {
            return 0;
        }
        return input;
    }

    private void configureBindings() {
        drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
                new DriveCommand(drivetrain,
                        () -> deadband(joystick.getLeftX()),
                        () -> joystick.povUp().getAsBoolean() ? -1.0 : deadband(joystick.getLeftY()),
                        () -> {
                            return deadband(joystick.getLeftTriggerAxis() - joystick.getRightTriggerAxis());
                        }, () -> {
                            return joystick.leftStick().getAsBoolean() ? slowMultiplier
                                    : (joystick.x().getAsBoolean() ? turboMultiplier : normalMultiplier);
                        }));

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

        SmartDashboard.putNumber("turboMultiplier", turboMultiplier);
        SmartDashboard.putNumber("normalMultiplier", normalMultiplier);
        SmartDashboard.putNumber("slowMultiplier", slowMultiplier);

        joystick.y().onTrue(new InstantCommand(() -> {
            turboMultiplier = SmartDashboard.getNumber("turboMultiplier", turboMultiplier);
            normalMultiplier = SmartDashboard.getNumber("normalMultiplier", normalMultiplier);
            slowMultiplier = SmartDashboard.getNumber("slowMultiplier", slowMultiplier);
        }));
    }

    public RobotContainer() {
        configureBindings();

        SmartDashboard.putData("Choose Auto", m_autoSmartDashboard);

        AutoCommands.setup(drivetrain);
    }

    public Command getAutonomousCommand() {

        // var auto = m_autoSmartDashboard.getSelected();
        // if (auto == null) {
        //     System.out.println("auto is null!");
        // }
        // return auto == null ? new BrakeCommand(drivetrain) : auto;

        // return new ParallelDeadlineGroup(new WaitCommand(250), new
        // DriveStraightCommand(drivetrain));
        return new Turn90Command(drivetrain);
    }
}
