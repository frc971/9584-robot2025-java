package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Superstructure;
import frc.robot.NetworkTables;

public class AutoCommands {
    private final Superstructure superstructure;
    private final NetworkTables networkTables;

    public AutoCommands(Superstructure superstructure, NetworkTables networkTables) {
        this.superstructure = superstructure;
        this.networkTables = networkTables;
    }

    public Command EjectCoral() {
        return Commands.sequence(
            superstructure.CoralEjectPressed(),
            Commands.waitSeconds(networkTables.getTimeValue(NetworkTables.ConstantId.AutoEjectCoralWait).in(edu.wpi.first.units.Units.Seconds)),
            superstructure.CoralEjectReleased()
        );
    }

    public Command IntakeAlgae() {
        return Commands.sequence(
            superstructure.AlgaeIntakePressed(),
            Commands.waitSeconds(networkTables.getTimeValue(NetworkTables.ConstantId.AutoIntakeAlgaeWait).in(edu.wpi.first.units.Units.Seconds)),
            superstructure.AlgaeIntakeReleased()
        );
    }

    public Command EjectAlgae() {
        return Commands.sequence(
            // Implement algae eject
            Commands.waitSeconds(networkTables.getTimeValue(NetworkTables.ConstantId.AutoEjectAlgaeWait).in(edu.wpi.first.units.Units.Seconds)),
            superstructure.AlgaeEjectReleased()
        );
    }

    public Command IntakeCoral() {
        return superstructure.AutoIntakeCoral();
    }
}