package frc.robot.generated;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;

import frc.robot.subsystems.CommandSwerveDrivetrain;

import static edu.wpi.first.units.Units.*;

public class TunerConstants {
    public static final Slot0Configs steerGains = new Slot0Configs()
        .withKP(100).withKI(0).withKD(0.5)
        .withKS(0.1).withKV(1.5).withKA(0)
        .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign);

    public static final Slot0Configs driveGains = new Slot0Configs()
        .withKP(0.1).withKI(0).withKD(0)
        .withKS(0).withKV(0.124).withKA(0);

    public static final SwerveModuleConstants.ClosedLoopOutputType kSteerClosedLoopOutput = SwerveModuleConstants.ClosedLoopOutputType.Voltage;
    public static final SwerveModuleConstants.ClosedLoopOutputType kDriveClosedLoopOutput = SwerveModuleConstants.ClosedLoopOutputType.Voltage;

    public static final SwerveModuleConstants.DriveMotorArrangement kDriveMotorType = SwerveModuleConstants.DriveMotorArrangement.TalonFX_Integrated;
    public static final SwerveModuleConstants.SteerMotorArrangement kSteerMotorType = SwerveModuleConstants.SteerMotorArrangement.TalonFX_Integrated;

    public static final SwerveModuleConstants.SteerFeedbackType kSteerFeedbackType = SwerveModuleConstants.SteerFeedbackType.FusedCANcoder;

    public static final double kSlipCurrentA = 300.0;

    public static final Measure<Velocity<Distance>> kSpeedAt12Volts = MetersPerSecond.of(5.5); // Adjust as per robot

    public static final double kDriveGearRatio = 5.95;
    public static final double kSteerGearRatio = 150.0 / 7.0;
    public static final double kCouplingGearRatio = 3.5714285714285716;
    public static final double kDriveWheelRadiusInches = 2;

    public static final InvertedValue kSteerMotorInverted = InvertedValue.CounterClockwise_Positive;

    public static final NeutralModeValue kDriveNeutralMode = NeutralModeValue.Brake;
    public static final NeutralModeValue kSteerNeutralMode = NeutralModeValue.Brake;

    public static final SwerveDrivetrainConstants DrivetrainConstants = new SwerveDrivetrainConstants()
        .withPigeon2Id(0)
        .withCANbusName("");

    // Define module constants for FrontLeft, FrontRight, BackLeft, BackRight
    // Example for FrontLeft (adjust IDs and values):
    public static final SwerveModuleConstants FrontLeft = new SwerveModuleConstants()
        .withLocation(new Translation2d(0.5, 0.5))
        .withDriveMotorId(1)
        .withSteerMotorId(2)
        .withCANcoderId(3)
        .withDriveMotorPinionTeeth(14)
        .withWheelRadius(Units.inchesToMeters(kDriveWheelRadiusInches))
        .withSteerMotorGains(steerGains)
        .withDriveMotorGains(driveGains)
        .withSteerMotorClosedLoopOutput(kSteerClosedLoopOutput)
        .withDriveMotorClosedLoopOutput(kDriveClosedLoopOutput)
        .withSpeedAt12VoltsMps(kSpeedAt12Volts.in(MetersPerSecond))
        .withFeedbackSource(kSteerFeedbackType)
        .withCouplingGearRatio(kCouplingGearRatio)
        .withDriveMotorInverted(InvertedValue.Clockwise_Positive)
        .withSteerMotorInverted(kSteerMotorInverted)
        .withSlipCurrent(kSlipCurrentA);

    // Similarly for other modules

    public static CommandSwerveDrivetrain createDrivetrain() {
        return new CommandSwerveDrivetrain(DrivetrainConstants, FrontLeft, FrontRight, BackLeft, BackRight);
    }
}