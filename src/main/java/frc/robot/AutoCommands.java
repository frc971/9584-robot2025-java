package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import frc.robot.subsystems.Intake;
import frc.robot.Constants;
import frc.robot.subsystems.NetworkTables;

public class AutoCommands {
  private final Intake m_intake;
  private final NetworkTables m_networkTables;

  public AutoCommands(Intake intake, NetworkTables networkTables) {
    this.m_intake = intake;
    this.m_networkTables = networkTables;
  }

  public Command intakeAlgae() {
    return Commands.sequence(
        m_intake.algaeIntakePressed(),
        Commands.waitSeconds(m_networkTables.getTimeValue(NetworkTables.ConstantId.AutoIntakeAlgaeWait)),
        m_intake.algaeIntakeReleased()
    );
  }

  public Command ejectAlgae() {
    return Commands.sequence(
        m_intake.algaeEjectPressed(),
        Commands.waitSeconds(m_networkTables.getTimeValue(NetworkTables.ConstantId.AutoEjectAlgaeWait)),
        m_intake.algaeEjectReleased()
    );
  }

  public Command ejectCoral() {
    return Commands.sequence(
        m_intake.coralEjectPressed(),
        Commands.waitSeconds(m_networkTables.getTimeValue(NetworkTables.ConstantId.AutoEjectCoralWait)),
        m_intake.coralEjectReleased()
    );
  }

  public Command intakeCoral() {
    return m_intake.autoIntakeCoral();
  }
}
