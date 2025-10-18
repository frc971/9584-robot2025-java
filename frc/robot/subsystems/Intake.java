package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.*;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Constants;
import frc.robot.NetworkTablesWrapper;

public class Intake {
    private final NetworkTablesWrapper networkTables;
    private final SwerveRequests.RobotCentric robotCentricDrive;

    private final TalonSRX armMotor = new TalonSRX(Constants.Intake.ARM_MOTOR_ID);
    private final VictorSPX rollerMotor = new VictorSPX(Constants.Intake.ROLLER_MOTOR_ID);
    private final DigitalInput coralBeamBreak = new DigitalInput(Constants.Intake.CORAL_BEAM_BREAK_CHANNEL);

    public Intake(NetworkTablesWrapper networkTables, SwerveRequests.RobotCentric robotCentricDrive) {
        this.networkTables = networkTabl
        this.robotCentricDrive = robotCentricDrive;
    }

    public void robotInit() {
        armMotor.configFactoryDefault();

        // PID constants
        armMotor.config_kP(0, networkTables.getDouble(ConstantId.ArmMotorProportionalGainValue), 10);
        armMotor.config_kI(0, networkTables.getDouble(ConstantId.ArmMotorIntegralGainValue), 10);
        armMotor.config_kD(0, networkTables.getDouble(ConstantId.ArmMotorDerivativeGainValue), 10);
        armMotor.config_kF(0, networkTables.getDouble(ConstantId.ArmMotorFeedForwardGainValue), 10);

        armMotor.setSelectedSensorPosition(
            networkTables.getDouble(ConstantId.ArmSelectedSensorPosition), 0, 10);

        armMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute, 0, 10);
        armMotor.configAllowableClosedloopError(
            0, networkTables.getDouble(ConstantId.ArmMotorAllowableCloseLoopError), 10);

        armMotor.configNominalOutputForward(
            networkTables.getDouble(ConstantId.ArmMotorForwardNominalPercentOutput), 10);
        armMotor.configNominalOutputReverse(
            networkTables.getDouble(ConstantId.ArmMotorReverseNominalPercentOutput), 10);
        armMotor.configPeakOutputForward(
            networkTables.getDouble(ConstantId.ArmMotorForwardPeakPercentOutput), 10);
        armMotor.configPeakOutputReverse(
            networkTables.getDouble(ConstantId.ArmMotorReversePeakPercentOutput), 10);
        armMotor.configMotionCruiseVelocity(
            networkTables.getDouble(ConstantId.ArmMotorMagicMotionCruiseVelocity), 10);
        armMotor.configMotionAcceleration(
            networkTables.getDouble(ConstantId.ArmMotorMagicMotionAccelerationVelocity), 10);

        armMotor.setSensorPhase(false);
        armMotor.setNeutralMode(NeutralMode.Brake);
        rollerMotor.setNeutralMode(NeutralMode.Brake);
    }

    public void teleopInit() {}

    public void resetDefaultPosition() {
        System.out.println("Resetting position");
        armMotor.setSelectedSensorPosition(0, 0, 10);
        System.out.println("Position2: " + armMotor.getSelectedSensorPosition(0));
        armMotor.set(ControlMode.Position,
            networkTables.getDouble(ConstantId.ArmDefaultPosition));
    }

    public Command resetEncoderPositionCommand() {
        return Commands.runOnce(this::resetDefaultPosition);
    }

    public void autonomousInit() {
        resetDefaultPosition();
        rollerMotor.set(VictorSPXControlMode.PercentOutput, 0);
        armMotor.set(ControlMode.Position,
            networkTables.getDouble(ConstantId.ArmDefaultPosition));
    }

    public void printPosition() {
        System.out.println("Position: " + armMotor.getSelectedSensorPosition(0));
    }

    public Command algaeIntakePressed() {
        return Commands.sequence(
            Commands.runOnce(() -> {
                System.out.println("============ AlgaeIntakePressed");
                System.out.println("lowering arm");
                System.out.println("Position1: " + armMotor.getSelectedSensorPosition(0));
                armMotor.set(ControlMode.Position,
                    networkTables.getDouble(ConstantId.ArmIntakePosition));
            }),
            Commands.waitSeconds(networkTables.getTime(ConstantId.AlgaeIntakeSequenceWait)),
            Commands.runOnce(() -> {
                System.out.println("stopping the lowering of arm");
                System.out.println("Position2: " + armMotor.getSelectedSensorPosition(0));
                rollerMotor.set(VictorSPXControlMode.PercentOutput,
                    networkTables.getDouble(ConstantId.RollerMovementAlgaeIntakeVelocity));
            })
        );
    }

    public Command algaeIntakeReleased() {
        return Commands.sequence(
            Commands.runOnce(() -> {
                System.out.println("============ AlgaeIntakeReleased");
                System.out.println("raising arm");
                System.out.println("Position3: " + armMotor.getSelectedSensorPosition());
                armMotor.set(ControlMode.Position,
                    networkTables.getDouble(ConstantId.ArmHoldPosition));
            }),
            Commands.waitSeconds(networkTables.getTime(ConstantId.AlgaeIntakeSequenceWait)),
            Commands.runOnce(() -> {
                System.out.println("stopping the raising of arm");
                System.out.println("Position4: " + armMotor.getSelectedSensorPosition(0));
                rollerMotor.set(VictorSPXControlMode.PercentOutput,
                    networkTables.getDouble(ConstantId.RollerMovementHoldVelocity));
            })
        );
    }

    public Command algaeEjectPressed() {
        return Commands.runOnce(() -> {
            System.out.println("============ AlgaeEjectPressed");
            rollerMotor.set(VictorSPXControlMode.PercentOutput,
                networkTables.getDouble(ConstantId.RollerMovementAlgaeEjectVelocity));
        });
    }

    public Command algaeEjectReleased() {
        return Commands.runOnce(() -> {
            System.out.println("============ AlgaeEjectReleased");
            armMotor.set(ControlMode.Position,
                networkTables.getDouble(ConstantId.ArmDefaultPosition));
            rollerMotor.set(VictorSPXControlMode.PercentOutput, 0);
        });
    }

    public Command coralEjectPressed() {
        return Commands.sequence(
            Commands.runOnce(() -> {
                System.out.println("============ CoralEjectPressed");
                armMotor.set(ControlMode.Position,
                    networkTables.getDouble(ConstantId.ArmDefaultPosition));
                rollerMotor.set(VictorSPXControlMode.PercentOutput,
                    networkTables.getDouble(ConstantId.RollerMovementCoralEjectVelocity));
            }),
            Commands.waitUntil(() -> !coralBeamBreak.get()),
            Commands.runOnce(() -> {
                System.out.println("lowering arm");
                rollerMotor.set(VictorSPXControlMode.PercentOutput, 0);
                armMotor.set(ControlMode.Position,
                    networkTables.getDouble(ConstantId.ArmCoralEjectPosition));
            })
        ).finallyDo(() ->
            rollerMotor.set(VictorSPXControlMode.PercentOutput, 0)
        );
    }

    public Command coralEjectReleased() {
        return Commands.runOnce(() -> {
            System.out.println("============ CoralEjectReleased");
            armMotor.set(ControlMode.Position,
                networkTables.getDouble(ConstantId.ArmDefaultPosition));
        });
    }

    public Command rollerForwardPressed() {
        return Commands.runOnce(() -> {
            System.out.println("============ Rollers Forward");
            rollerMotor.set(VictorSPXControlMode.PercentOutput,
                networkTables.getDouble(ConstantId.RollerMovementForwardVelocity));
        });
    }

    public Command rollerForwardReleased() {
        return Commands.runOnce(() -> {
            System.out.println("============ Rollers Stopped");
            rollerMotor.set(VictorSPXControlMode.PercentOutput, 0);
        });
    }

    public Command rollerBackwardPressed() {
        return Commands.runOnce(() -> {
            System.out.println("============ Rollers Backward");
            rollerMotor.set(VictorSPXControlMode.PercentOutput,
                networkTables.getDouble(ConstantId.RollerMovementBackwardVelocity));
        });
    }

    public Command rollerBackwardReleased() {
        return Commands.runOnce(() -> {
            System.out.println("============ Rollers Stopped");
            rollerMotor.set(VictorSPXControlMode.PercentOutput, 0);
        });
    }

    public Command armUpPressed() {
        return Commands.runOnce(() -> {
            System.out.println("============ Arm up");
            armMotor.set(ControlMode.PercentOutput,
                networkTables.getDouble(ConstantId.ArmUpVelocity));
        });
    }

    public Command armUpReleased() {
        return Commands.runOnce(() -> {
            System.out.println("============ Arm stopped");
            armMotor.set(ControlMode.Position,
                armMotor.getSelectedSensorPosition(0));
        });
    }

    public Command armDownPressed() {
        return Commands.runOnce(() -> {
            System.out.println("============ Arm down");
            armMotor.set(ControlMode.PercentOutput,
                networkTables.getDouble(ConstantId.ArmDownVelocity));
        });
    }

    public Command armDownReleased() {
        return Commands.runOnce(() -> {
            System.out.println("============ Arm stopped");
            armMotor.set(ControlMode.Position,
                armMotor.getSelectedSensorPosition(0));
        });
    }

    public Command autoIntakeCoral() {
        return Commands.waitUntil(() -> {
            System.out.println("Beambreak value: " + coralBeamBreak.get());
            return coralBeamBreak.get();
        });
    }
}
