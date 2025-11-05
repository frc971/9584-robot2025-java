package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Superstructure;
import frc.robot.NetworkTables;

public class AutoCommands {
    private final Superstructure intake;
    private final NetworkTables networkTables;

    public AutoCommands(Superstructure intake, NetworkTables networkTables) {
        this.intake = intake;
        this.networkTables = networkTables;
    }

    public Command EjectCoral() {
        return Commands.sequence(
            intake.CoralEjectPressed(),
            Commands.waitSeconds(networkTables.getTimeValue(NetworkTables.ConstantId.AutoEjectCoralWait).in(edu.wpi.first.units.Units.Seconds)),
            intake.CoralEjectReleased()
        );
    }

    public Command IntakeAlgae() {
        return Commands.sequence(
            intake.AlgaeIntakePressed(),
            Commands.waitSeconds(networkTables.getTimeValue(NetworkTables.ConstantId.AutoIntakeAlgaeWait).in(edu.wpi.first.units.Units.Seconds)),
            intake.AlgaeIntakeReleased()
        );
    }

    public Command EjectAlgae() {
        return Commands.sequence(
            // Implement algae eject
            Commands.waitSeconds(networkTables.getTimeValue(NetworkTables.ConstantId.AutoEjectAlgaeWait).in(edu.wpi.first.units.Units.Seconds))
        );
    }

    public Command IntakeCoral() {
        return intake.AutoIntakeCoral();
    }
}