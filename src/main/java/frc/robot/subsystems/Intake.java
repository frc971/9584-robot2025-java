package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import frc.robot.NetworkTables;
import frc.robot.NetworkTables.ConstantId;
import com.ctre.phoenix6.swerve.SwerveRequest;

public class Intake extends edu.wpi.first.wpilibj2.command.SubsystemBase {
    private final DigitalInput m_coralBeamBreak = new DigitalInput(0);
    private final NetworkTables m_networkTables;
    private final SwerveRequest.RobotCentric m_robotCentricDrive;

    private final TalonFX armMotor = new TalonFX(16);
    private final TalonFX rollerMotor = new TalonFX(17); // Added missing roller motor

    private final PositionVoltage armPositionControl = new PositionVoltage(0);
    private final VoltageOut rollerVoltageControl = new VoltageOut(0);

    public Intake(NetworkTables networkTables, SwerveRequest.RobotCentric robotCentricDrive) {
        m_networkTables = networkTables;
        m_robotCentricDrive = robotCentricDrive;
    }

    public void RobotInit() {
        // Configure arm motor using Phoenix 6 API
        TalonFXConfiguration armConfig = new TalonFXConfiguration();
        
        Slot0Configs slot0 = armConfig.Slot0;
        slot0.kP = m_networkTables.getDoubleValue(ConstantId.ArmMotorProportionalGainValue);
        slot0.kI = m_networkTables.getDoubleValue(ConstantId.ArmMotorIntegralGainValue);
        slot0.kD = m_networkTables.getDoubleValue(ConstantId.ArmMotorDerivativeGainValue);
        slot0.kS = m_networkTables.getDoubleValue(ConstantId.ArmMotorFeedForwardGainValue);
        
        MotorOutputConfigs motorOutput = armConfig.MotorOutput;
        motorOutput.NeutralMode = NeutralModeValue.Brake;
        
        armMotor.getConfigurator().apply(armConfig);
        armMotor.setPosition(m_networkTables.getDoubleValue(ConstantId.ArmSelectedSensorPosition));
        
        // Configure roller motor
        TalonFXConfiguration rollerConfig = new TalonFXConfiguration();
        rollerConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        rollerMotor.getConfigurator().apply(rollerConfig);
    }

    public void TeleopInit() {}

    public void ResetDefaultPosition() {
        System.out.println("Resetting position");
        armMotor.setPosition(0);
        armMotor.setControl(armPositionControl.withPosition(
            m_networkTables.getDoubleValue(ConstantId.ArmDefaultPosition)
        ));
    }

    public Command ResetEncoderPositionCommand() {
        return Commands.runOnce(this::ResetDefaultPosition);
    }

    public void AutonomousInit() {
        ResetDefaultPosition();
        rollerMotor.setControl(rollerVoltageControl.withOutput(0));
        armMotor.setControl(armPositionControl.withPosition(
            m_networkTables.getDoubleValue(ConstantId.ArmDefaultPosition)
        ));
    }

    public void PrintPosition() {
        System.out.println("Position: " + armMotor.getPosition().getValue());
    }

    public Command AlgaeIntakePressed() {
        return Commands.runOnce(() -> {
            System.out.println("============ AlgaeIntakePressed");
            armMotor.setControl(armPositionControl.withPosition(
                m_networkTables.getDoubleValue(ConstantId.ArmIntakePosition)
            ));
            rollerMotor.setControl(rollerVoltageControl.withOutput(
                m_networkTables.getDoubleValue(ConstantId.RollerMovementAlgaeIntakeVelocity) * 12.0
            ));
        });
    }

    public Command AlgaeIntakeReleased() {
        return Commands.runOnce(() -> {
            System.out.println("============ AlgaeIntakeReleased");
            armMotor.setControl(armPositionControl.withPosition(
                m_networkTables.getDoubleValue(ConstantId.ArmDefaultPosition)
            ));
            rollerMotor.setControl(rollerVoltageControl.withOutput(0));
        });
    }

    public Command ArmUpPressed() {
        return Commands.runOnce(() -> {
            System.out.println("============ ArmUpPressed");
            armMotor.set(m_networkTables.getDoubleValue(ConstantId.ArmUpVelocity));
        });
    }

    public Command ArmUpReleased() {
        return Commands.runOnce(() -> {
            System.out.println("============ ArmUpReleased");
            armMotor.set(0);
        });
    }

    public Command AutoIntakeCoral() {
        return Commands.waitUntil(() -> {
            System.out.println("Beambreak value: " + m_coralBeamBreak.get());
            return m_coralBeamBreak.get();
        });
    }

    public Command CoralEjectPressed() {
        return Commands.runOnce(() -> {
            System.out.println("============ CoralEjectPressed");
            armMotor.setControl(armPositionControl.withPosition(
                m_networkTables.getDoubleValue(ConstantId.ArmCoralEjectPosition)
            ));
            rollerMotor.setControl(rollerVoltageControl.withOutput(
                m_networkTables.getDoubleValue(ConstantId.RollerMovementCoralEjectVelocity) * 12.0
            ));
        });
    }

    public Command CoralEjectReleased() {
        return Commands.runOnce(() -> {
            System.out.println("============ CoralEjectReleased");
            armMotor.setControl(armPositionControl.withPosition(
                m_networkTables.getDoubleValue(ConstantId.ArmDefaultPosition)
            ));
            rollerMotor.setControl(rollerVoltageControl.withOutput(0));
        });
    }
}