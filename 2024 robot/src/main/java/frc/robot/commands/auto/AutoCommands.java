package frc.robot.commands.auto;

import java.util.HashMap;
import java.util.Map;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.CommandSwerveDrivetrain;
import frc.robot.commands.drivetrain.BrakeCommand;

public final class AutoCommands {
    public static void setup(CommandSwerveDrivetrain train) {
        Map<String, Command> commands = new HashMap<>();
        commands.put("brake", setupBrake(train));
        NamedCommands.registerCommands(commands);
    }
    public static Command setupBrake(CommandSwerveDrivetrain t) {
        Command bcmd = new ParallelDeadlineGroup(new WaitCommand(1),new BrakeCommand(t));
        return bcmd;
    }
}
