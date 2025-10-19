package frc.robot.subsystems;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import frc.robot.Constants;
import frc.robot.NetworkTables;

public class Climber {
  private final TalonFX m_motor = new TalonFX(Constants.CLIMBER_MOTOR_ID);
  private final NetworkTables m_networkTables;
  private double maxCurrentGoingUp = 0.0;

  public Climber(NetworkTables networkTables) {
    this.m_networkTables = networkTables;

    // --- Configure TalonFX ---
    TalonFXConfiguration cfg = new TalonFXConfiguration();

    // Feedback (sensor to mechanism ratio)
    cfg.Feedback.SensorToMechanismRatio = 125.0; // 125 rotor rotations per mechanism rotation

    // Motor output configuration
    cfg.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    // Motion Magic configuration
    cfg.MotionMagic.MotionMagicCruiseVelocity = 5;   // rotations per second
    cfg.MotionMagic.MotionMagicAcceleration = 10;    // rotations per second squared
    cfg.MotionMagic.MotionMagicJerk = 100;           // rotations per second cubed

    // Slot 0 PIDF configuration
    cfg.Slot0.kS = 0.25;
    cfg.Slot0.kV = 0.12;
    cfg.Slot0.kA = 0.01;
    cfg.Slot0.kP = 60;
    cfg.Slot0.kI = 0;
    cfg.Slot0.kD = 0.5;

    // Try applying configuration up to 5 times
    StatusCode status = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {
      status = m_motor.getConfigurator().apply(cfg);
      if (status.isOK()) break;
    }
    if (!status.isOK()) {
      System.out.println("Could not configure device. Error: " + status.getName());
    }

    // Simulation setup
    if (RobotBase.isSimulation()) {
      // In simulation, you would normally register this motor with your sim physics engine
      // PhysicsSim.getInstance().addTalonFX(m_motor, 0.001); // if you have a sim helper class
    }
  }

  // --- Command Methods ---

  public Command climbPressed() {
    return Commands.sequence(
        Commands.runOnce(() -> {
          maxCurrentGoingUp = 0;
          System.out.println("Climbing");
          m_motor.set(m_networkTables.getDoubleValue(NetworkTables.ConstantId.ClimbVelocity));
        }),
        Commands.waitSeconds(0.1),
        Commands.waitUntil(() -> {
          double torqueCurrent = m_motor.getTorqueCurrent().getValueAsDouble();
          if (Math.abs(torqueCurrent) > maxCurrentGoingUp) {
            maxCurrentGoingUp = Math.abs(torqueCurrent);
          }
          System.out.println("Going up. Torque Current: " + torqueCurrent +
              " Max current ever: " + maxCurrentGoingUp);

          double limit = m_networkTables
              .getCurrentValue(NetworkTables.ConstantId.ClimberTorqueCurrentLimit)
              .inAmperes();

          return Math.abs(torqueCurrent) > limit;
        }),
        Commands.runOnce(() -> {
          System.out.println("Stopping climb because it is at full extension");
          m_motor.set(0);
          m_motor.setControl(new PositionDutyCycle(m_motor.getPosition().getValue()));
        })
    );
  }

  public Command climbReleased() {
    return Commands.runOnce(() -> {
      System.out.println("Climbing stopped");
      m_motor.set(0);
      m_motor.setControl(new PositionDutyCycle(m_motor.getPosition().getValue()));
    });
  }

  public Command unclimbPressed() {
    return Commands.runOnce(() -> {
      System.out.println("Unclimbing");
      m_motor.set(m_networkTables.getDoubleValue(NetworkTables.ConstantId.UnclimbVelocity));
    });
  }

  public Command unclimbReleased() {
    return Commands.runOnce(() -> {
      System.out.println("Unclimbing stopped");
      m_motor.set(0);
    });
  }
}

