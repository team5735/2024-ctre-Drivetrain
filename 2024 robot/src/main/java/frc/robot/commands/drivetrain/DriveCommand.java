package frc.robot.commands.drivetrain;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.CommandSwerveDrivetrain;

public class DriveCommand extends Command {
    private final CommandSwerveDrivetrain m_drivetrain;
    private final Supplier<Double> m_stickX;
    private final Supplier<Double> m_stickY;
    private final Supplier<Double> m_rotate;
    private final Supplier<Double> m_multiplier;
    public DriveCommand(CommandSwerveDrivetrain drivetrain, Supplier<Double> stickX, Supplier<Double> stickY, Supplier<Double> rotate, Supplier<Double> multiplier) {
        m_drivetrain = drivetrain;

        m_stickX = stickX;
        m_stickY = stickY;
        m_rotate = rotate;
        m_multiplier = multiplier;

        addRequirements(m_drivetrain);
    }

    public void execute() {
        m_drivetrain.drive(-m_stickY.get(), -m_stickX.get(), m_rotate.get(), m_multiplier.get());
    }
}
