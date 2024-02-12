package frc.robot.commands.drivetrain;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.CommandSwerveDrivetrain;

public class BrakeCommand extends Command {
    private final CommandSwerveDrivetrain m_drivetrain;

    public BrakeCommand(CommandSwerveDrivetrain train) {
        addRequirements(train);
        m_drivetrain = train;
    }

    public void execute() {
        m_drivetrain.brake();
    }
}
