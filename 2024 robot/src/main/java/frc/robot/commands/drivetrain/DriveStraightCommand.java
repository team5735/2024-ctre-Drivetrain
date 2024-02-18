package frc.robot.commands.drivetrain;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.CommandSwerveDrivetrain;

public class DriveStraightCommand extends Command {
    private final CommandSwerveDrivetrain m_drivetrain;
    public DriveStraightCommand(CommandSwerveDrivetrain d) {
        m_drivetrain = d;
        addRequirements(m_drivetrain);
    }

    public void execute() {
        m_drivetrain.drive(1.00, 0, 0);
    }
}