package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import frc.robot.subsystems.Intake;

public class AutoCommands {
    private final Intake m_intake;
    private final NetworkTables m_networkTables;

    public AutoCommands(Intake intake, NetworkTables networkTables) {
        m_intake = intake;
        m_networkTables = networkTables;
    }

    public Command IntakeAlgae() {
        return Commands.sequence(
            m_intake.AlgaeIntakePressed(),
            Commands.waitSeconds(m_networkTables.getTimeValue(NetworkTables.ConstantId.AutoIntakeAlgaeWait).in(Seconds)),
            m_intake.AlgaeIntakeReleased()
        );
    }

    public Command EjectAlgae() {
        return Commands.sequence(
            m_intake.AlgaeEjectPressed(),
            Commands.waitSeconds(m_networkTables.getTimeValue(NetworkTables.ConstantId.AutoEjectAlgaeWait).in(Seconds)),
            m_intake.AlgaeEjectReleased()
        );
    }

    public Command EjectCoral() {
        return Commands.sequence(
            m_intake.CoralEjectPressed(),
            Commands.waitSeconds(m_networkTables.getTimeValue(NetworkTables.ConstantId.AutoEjectCoralWait).in(Seconds)),
            m_intake.CoralEjectReleased()
        );
    }

    public Command IntakeCoral() {
        return m_intake.AutoIntakeCoral();
    }
}