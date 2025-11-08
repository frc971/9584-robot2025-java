package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.NetworkTables;
import frc.robot.NetworkTables.ConstantId;

public class Superstructure extends edu.wpi.first.wpilibj2.command.SubsystemBase {
    private final DigitalInput m_coralBeamBreak = new DigitalInput(0);
    private final NetworkTables m_networkTables;
    private final CommandSwerveDrivetrain m_robotDrivetrain;

    private final TalonSRX armMotor = new TalonSRX(16);
    private final VictorSPX rollerMotor = new VictorSPX(18);

    public Superstructure(NetworkTables networkTables, CommandSwerveDrivetrain drivetrain) {
        m_networkTables = networkTables;
        m_robotDrivetrain = drivetrain;
    }

    public void RobotInit() {
        // Configure arm motor using Phoenix 5 API
        armMotor.configFactoryDefault();
        armMotor.setNeutralMode(NeutralMode.Brake);
        
        // Configure PID for arm motor
        armMotor.config_kP(0, m_networkTables.getDoubleValue(ConstantId.ArmMotorProportionalGainValue));
        armMotor.config_kI(0, m_networkTables.getDoubleValue(ConstantId.ArmMotorIntegralGainValue));
        armMotor.config_kD(0, m_networkTables.getDoubleValue(ConstantId.ArmMotorDerivativeGainValue));
        armMotor.config_kF(0, m_networkTables.getDoubleValue(ConstantId.ArmMotorFeedForwardGainValue));
        
        // Configure motion magic
        armMotor.configMotionCruiseVelocity(
            m_networkTables.getDoubleValue(ConstantId.ArmMotorMagicMotionCruiseVelocity)
        );
        armMotor.configMotionAcceleration(
            m_networkTables.getDoubleValue(ConstantId.ArmMotorMagicMotionAccelerationVelocity)
        );
        
        // Configure output limits
        armMotor.configNominalOutputForward(
            m_networkTables.getDoubleValue(ConstantId.ArmMotorForwardNominalPercentOutput)
        );
        armMotor.configNominalOutputReverse(
            m_networkTables.getDoubleValue(ConstantId.ArmMotorReverseNominalPercentOutput)
        );
        armMotor.configPeakOutputForward(
            m_networkTables.getDoubleValue(ConstantId.ArmMotorForwardPeakPercentOutput)
        );
        armMotor.configPeakOutputReverse(
            m_networkTables.getDoubleValue(ConstantId.ArmMotorReversePeakPercentOutput)
        );
        
        // Set initial position
        armMotor.setSelectedSensorPosition(
            m_networkTables.getDoubleValue(ConstantId.ArmSelectedSensorPosition)
        );
        
        // Configure roller motor
        rollerMotor.configFactoryDefault();
        rollerMotor.setNeutralMode(NeutralMode.Brake);
    }

    public void TeleopInit() {}

    public void ResetDefaultPosition() {
        System.out.println("Resetting position");
        armMotor.setSelectedSensorPosition(0);
        armMotor.set(ControlMode.MotionMagic, 
            m_networkTables.getDoubleValue(ConstantId.ArmDefaultPosition)
        );
    }

    public Command ResetEncoderPositionCommand() {
        return Commands.runOnce(this::ResetDefaultPosition);
    }

    public void AutonomousInit() {
        ResetDefaultPosition();
        rollerMotor.set(ControlMode.PercentOutput, 0);
        armMotor.set(ControlMode.MotionMagic, 
            m_networkTables.getDoubleValue(ConstantId.ArmDefaultPosition)
        );
    }

    public void PrintPosition() {
        System.out.println("Position: " + armMotor.getSelectedSensorPosition());
    }

    public Command AlgaeIntakePressed() {
        return Commands.sequence(
            Commands.runOnce(() -> {
                System.out.println("============ AlgaeIntakePressed");
                System.out.println("lowering arm");
                System.out.println("Position1: " + armMotor.getSelectedSensorPosition());
                
                armMotor.set(ControlMode.MotionMagic,
                    m_networkTables.getDoubleValue(ConstantId.ArmIntakePosition)
                );
            }),
            Commands.waitSeconds(
                m_networkTables.getTimeValue(ConstantId.AlgaeIntakeSequenceWait).in(Units.Seconds)
            ),
            Commands.runOnce(() -> {
                System.out.println("stopping the lowering of arm");
                System.out.println("Position2: " + armMotor.getSelectedSensorPosition());
                
                rollerMotor.set(ControlMode.PercentOutput,
                    m_networkTables.getDoubleValue(ConstantId.RollerMovementAlgaeIntakeVelocity)
                );
            })
        );
    }

    public Command AlgaeIntakeReleased() {
        return Commands.sequence(
            Commands.runOnce(() -> {
                System.out.println("============ AlgaeIntakeReleased");
                armMotor.set(ControlMode.MotionMagic,
                    m_networkTables.getDoubleValue(ConstantId.ArmHoldPosition)
                );
            }),
            Commands.waitSeconds(
                m_networkTables.getTimeValue(ConstantId.AlgaeIntakeSequenceWait).in(Units.Seconds)
            ),
            Commands.runOnce(() -> {
                rollerMotor.set(ControlMode.PercentOutput, 0);
            })
        );
    }

    public Command ArmUpPressed() {
        return Commands.runOnce(() -> {
            System.out.println("============ ArmUpPressed");
            armMotor.set(ControlMode.PercentOutput, 
                m_networkTables.getDoubleValue(ConstantId.ArmUpVelocity)
            );
        });
    }

    public Command ArmUpReleased() {
        return Commands.runOnce(() -> {
            System.out.println("============ Arm stopped");
            armMotor.set(ControlMode.PercentOutput, 0);
        });
    }

    public Command ArmDownPressed() {
        return Commands.runOnce(() -> {
            System.out.println("=========== ArmDownPressed");
            armMotor.set(ControlMode.PercentOutput, 
                m_networkTables.getDoubleValue(ConstantId.ArmDownVelocity)
            );
        });
    }

    public Command ArmDownReleased() {
        return Commands.runOnce(() -> {
            System.out.println("============ Arm stopped");
            armMotor.set(ControlMode.PercentOutput, 0);
        });
    }

    public Command AutoIntakeCoral() {
        return Commands.waitUntil(() -> {
            System.out.println("Beambreak value: " + m_coralBeamBreak.get());
            return m_coralBeamBreak.get();
        });
    }

    public Command AlgaeEjectPressed() {
        return Commands.runOnce(() -> {
            System.out.println("============= AlgaeEjectPressed");
            rollerMotor.set(ControlMode.PercentOutput,
                m_networkTables.getDoubleValue(ConstantId.RollerMovementAlgaeEjectVelocity)
            );
        });
    }

    public Command AlgaeEjectReleased() {
        return Commands.runOnce(() -> {
            System.out.println("========== AlgaeEjectReleased");
            armMotor.set(ControlMode.MotionMagic,
                m_networkTables.getDoubleValue(ConstantId.ArmDefaultPosition)
            );
            rollerMotor.set(ControlMode.PercentOutput, 0);
        });
    }

    public Command CoralEjectPressed() {
        return Commands.sequence(
            Commands.runOnce(() -> {
                System.out.println("============ CoralEjectPressed\nmoving rollers forward//\n");
                rollerMotor.set(ControlMode.MotionMagic,
                    m_networkTables.getDoubleValue(ConstantId.RollerMovementCoralEjectVelocity)
                );
            }),
            Commands.waitSeconds(
                m_networkTables.getTimeValue(ConstantId.ArmCoralEjectSequenceWait).in(Units.Seconds)
            ),
            Commands.runOnce(() -> {
                armMotor.set(ControlMode.PercentOutput,
                    m_networkTables.getDoubleValue(ConstantId.ArmCoralEjectPosition)
                );
            })
        );
    }

    public Command CoralEjectReleased() {
        return Commands.runOnce(() -> {
            System.out.println("============ CoralEjectReleased");
            armMotor.set(ControlMode.MotionMagic,
                m_networkTables.getDoubleValue(ConstantId.ArmDefaultPosition)
            );
            rollerMotor.set(ControlMode.PercentOutput, 0);
        });
    }

    public Command RollerForwardPressed() {
        return Commands.runOnce(() -> {
            System.out.println("============ Rollers Forward");
            rollerMotor.set(ControlMode.PercentOutput,
                m_networkTables.getDoubleValue(ConstantId.RollerMovementForwardVelocity)
            );
        });
    }
    
    public Command RollerForwardReleased() {
        return Commands.runOnce(() -> {
            System.out.println("============ Rollers Stopped");
            rollerMotor.set(ControlMode.PercentOutput, 0);
        });
    }
    
    public Command RollerBackwardPressed() {
        return Commands.runOnce(() -> {
            System.out.println("=========== Rollers Backward");
            rollerMotor.set(ControlMode.PercentOutput,
                m_networkTables.getDoubleValue(ConstantId.RollerMovementBackwardVelocity)
            );
        });
    }

    public Command RollerBackwardReleased() {
        return Commands.runOnce(() -> {
            System.out.println("============ Rollers Stopped");
            rollerMotor.set(ControlMode.PercentOutput, 0);
        });
    }
}