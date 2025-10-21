package frc.robot.subsystems;

import com.ctre.phoenix6.*;
import com.ctre.phoenix6.hardware.TalonFX;

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

    private final com.ctre.phoenix6.hardware.TalonFX armMotor = new TalonFX(16);

    public Intake(NetworkTables networkTables, SwerveRequest.RobotCentric robotCentricDrive) {
        m_networkTables = networkTables;
        m_robotCentricDrive = robotCentricDrive;
    }

    public void RobotInit() {
        armMotor.configFactoryDefault();
        armMotor.config_kP(0, m_networkTables.getDoubleValue(ConstantId.ArmMotorProportionalGainValue), 10);
        armMotor.config_kI(0, m_networkTables.getDoubleValue(ConstantId.ArmMotorIntegralGainValue), 10);
        armMotor.config_kD(0, m_networkTables.getDoubleValue(ConstantId.ArmMotorDerivativeGainValue), 10);
        armMotor.config_kF(0, m_networkTables.getDoubleValue(ConstantId.ArmMotorFeedForwardGainValue), 10);
        armMotor.setSelectedSensorPosition(m_networkTables.getDoubleValue(ConstantId.ArmSelectedSensorPosition), 0, 10);
        armMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute, 0, 10);
        armMotor.configAllowableClosedloopError(0, m_networkTables.getDoubleValue(ConstantId.ArmMotorAllowableCloseLoopError), 10);

        armMotor.configNominalOutputForward(m_networkTables.getDoubleValue(ConstantId.ArmMotorForwardNominalPercentOutput), 10);
        armMotor.configNominalOutputReverse(m_networkTables.getDoubleValue(ConstantId.ArmMotorReverseNominalPercentOutput), 10);
        armMotor.configPeakOutputForward(m_networkTables.getDoubleValue(ConstantId.ArmMotorForwardPeakPercentOutput), 10);
        armMotor.configPeakOutputReverse(m_networkTables.getDoubleValue(ConstantId.ArmMotorReversePeakPercentOutput), 10);
        armMotor.configMotionCruiseVelocity(m_networkTables.getDoubleValue(ConstantId.ArmMotorMagicMotionCruiseVelocity), 10);
        armMotor.configMotionAcceleration(m_networkTables.getDoubleValue(ConstantId.ArmMotorMagicMotionAccelerationVelocity), 10);
        armMotor.setSensorPhase(false);

        armMotor.setNeutralMode(NeutralMode.Brake);
        rollerMotor.setNeutralMode(NeutralMode.Brake);
    }

    public void TeleopInit() {}

    public void ResetDefaultPosition() {
        System.out.println("Resetting position");
        armMotor.setSelectedSensorPosition(0, 0, 10);
        armMotor.set(ControlMode.Position, m_networkTables.getDoubleValue(ConstantId.ArmDefaultPosition));
    }

    public Command ResetEncoderPositionCommand() {
        return Commands.runOnce(this::ResetDefaultPosition);
    }

    public void AutonomousInit() {
        ResetDefaultPosition();
        rollerMotor.set(ControlMode.PercentOutput, 0);
        armMotor.set(ControlMode.Position, m_networkTables.getDoubleValue(ConstantId.ArmDefaultPosition));
    }

    public void PrintPosition() {
        System.out.println("Position: " + armMotor.getSelectedSensorPosition(0));
    }

    public Command AlgaeIntakePressed() {
        return Commands.runOnce(() -> {
            System.out.println("============ AlgaeIntakePressed");
            armMotor.set(ControlMode.Position, m_networkTables.getDoubleValue(ConstantId.ArmIntakePosition));
            rollerMotor.set(ControlMode.PercentOutput, m_networkTables.getDoubleValue(ConstantId.RollerMovementAlgaeIntakeVelocity));
        });
    }

    public Command AlgaeIntakeReleased() {
        return Commands.runOnce(() -> {
            System.out.println("============ AlgaeIntakeReleased");
            armMotor.set(ControlMode.Position, m_networkTables.getDoubleValue(ConstantId.ArmDefaultPosition));
            rollerMotor.set(ControlMode.PercentOutput, 0);
        });
    }

    // Similarly implement other commands as in previous messages
    public Command AutoIntakeCoral() {
        return Commands.waitUntil(() -> {
            System.out.println("Beambreak value: " + m_coralBeamBreak.get());
            return m_coralBeamBreak.get();
        });
    }

    public Command CoralEjectPressed() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'CoralEjectPressed'");
    }
}