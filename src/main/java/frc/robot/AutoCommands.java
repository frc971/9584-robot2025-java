package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.NetworkTables;
import frc.robot.subsystems.Intake;

public class AutoCommands {
    private final Intake intake;
    private final NetworkTables networkTables;

    public AutoCommands(Intake intake, NetworkTables networkTables) {
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