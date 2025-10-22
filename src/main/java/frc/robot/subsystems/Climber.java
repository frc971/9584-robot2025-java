package frc.robot.subsystems;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import frc.robot.NetworkTables;
import frc.robot.NetworkTables.ConstantId;
import frc.robot.sim.PhysicsSim;

import static edu.wpi.first.units.Units.*;

public class Climber extends edu.wpi.first.wpilibj2.command.SubsystemBase {
    private final TalonFX m_motor = new TalonFX(13, "rio");
    private final NetworkTables m_networkTables;
    private double maxCurrentGoingUp = 0;

    public Climber(NetworkTables networkTables) {
        m_networkTables = networkTables;

        TalonFXConfiguration cfg = new TalonFXConfiguration();

        FeedbackConfigs fdb = cfg.Feedback;
        fdb.SensorToMechanismRatio = 125;

        MotorOutputConfigs moc = cfg.MotorOutput;
        moc.NeutralMode = NeutralModeValue.Brake;

        MotionMagicConfigs mm = cfg.MotionMagic;
        mm.MotionMagicCruiseVelocity = 5;
        mm.MotionMagicAcceleration = 10;
        mm.MotionMagicJerk = 100;

        Slot0Configs slot0 = cfg.Slot0;
        slot0.kS = 0.25;
        slot0.kV = 0.12;
        slot0.kA = 0.01;
        slot0.kP = 60;
        slot0.kI = 0;
        slot0.kD = 0.5;

        StatusCode status = m_motor.getConfigurator().apply(cfg, 1.0);
        if (!status.isOK()) {
            System.out.println("Could not configure climber motor: " + status.toString());
        }

        if (edu.wpi.first.wpilibj.RobotBase.isSimulation()) {
            PhysicsSim.getInstance().addTalonFX(m_motor, KilogramSquareMeters.of(0.001));
        }
    }

    public Command ClimbPressed() {
        return Commands.sequence(
            Commands.runOnce(() -> {
                maxCurrentGoingUp = 0;
                System.out.println("Climbing");
                m_motor.set(m_networkTables.getDoubleValue(ConstantId.ClimbVelocity));
            }),
            Commands.waitSeconds(0.1),
            Commands.waitUntil(() -> {
                double torqueCurrent = m_motor.getTorqueCurrent().getValueAsDouble();
                if (Math.abs(torqueCurrent) > maxCurrentGoingUp) {
                    maxCurrentGoingUp = torqueCurrent;
                }
                System.out.println("Going up. Torque Current: " + torqueCurrent + " Max current ever: " + maxCurrentGoingUp);
                return Math.abs(torqueCurrent) > m_networkTables.getCurrentValue(ConstantId.ClimberTorqueCurrentLimit).in(Amps);
            }),
            Commands.runOnce(() -> {
                System.out.println("Stopping climb because it is at full extension");
                m_motor.set(0);
                m_motor.setControl(new PositionDutyCycle(m_motor.getPosition().getValueAsDouble()));
            })
        );
    }

    public Command ClimbReleased() {
        return Commands.runOnce(() -> {
            System.out.println("Climbing stopped");
            m_motor.set(0);
            m_motor.setControl(new PositionDutyCycle(m_motor.getPosition().getValueAsDouble()));
        });
    }

    public Command UnclimbPressed() {
        return Commands.runOnce(() -> {
            System.out.println("Unclimbing");
            m_motor.set(m_networkTables.getDoubleValue(ConstantId.UnclimbVelocity));
        });
    }

    public Command UnclimbReleased() {
        return Commands.runOnce(() -> {
            System.out.println("Unclimbing stopped");
            m_motor.set(0);
        });
    }
}