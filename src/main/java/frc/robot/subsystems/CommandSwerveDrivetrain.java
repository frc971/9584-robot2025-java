package frc.robot.subsystems;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj.Timer;

public class CommandSwerveDrivetrain extends SwerveDrivetrain implements Subsystem {
    private static final double kSimLoopPeriod = 0.005;
    private Notifier m_simNotifier = null;
    private static double m_lastSimTime;
    
    private final Rotation2d kBlueAlliancePerspectiveRotation = Rotation2d.fromDegrees(0);
    private final Rotation2d kRedAlliancePerspectiveRotation = Rotation2d.fromDegrees(180);
    private boolean m_hasAppliedOperatorPerspective = false;

    public CommandSwerveDrivetrain(
        SwerveDrivetrainConstants drivetrainConstants, 
            SwerveModuleConstants<?, ?, ?>... modules) {
            super(
                (deviceId, canbus) -> new TalonFX(deviceId, canbus), // driveMotorConstructor
                (deviceId, canbus) -> new TalonFX(deviceId, canbus), // steerMotorConstructor
                (deviceId, canbus) -> new CANcoder(deviceId, canbus), // steerEncoderConstructor
                drivetrainConstants,
                modules
            );
        
        configurePathPlanner();
        if (Utils.isSimulation()) {
            startSimThread();
        }
    }

    private void configurePathPlanner() {
        // Configure AutoBuilder for PathPlanner if needed
    }

    @Override
    public void periodic() {
        if (!m_hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
            DriverStation.getAlliance().ifPresent(alliance -> {
                setOperatorPerspectiveForward(
                    alliance == DriverStation.Alliance.Red ? 
                        kRedAlliancePerspectiveRotation : 
                        kBlueAlliancePerspectiveRotation
                );
                m_hasAppliedOperatorPerspective = true;
            });
        }
    }

    private void startSimThread() {
        m_lastSimTime = Timer.getFPGATimestamp();

        m_simNotifier = new Notifier(() -> {
            final double currentTime = Timer.getFPGATimestamp();
            double deltaTime = currentTime - m_lastSimTime;
            m_lastSimTime = currentTime;

            updateSimState(deltaTime, 12.0);
        });
        m_simNotifier.startPeriodic(kSimLoopPeriod);
    }

    public Command applyRequest(java.util.function.Supplier<SwerveRequest> requestSupplier) {
        return Commands.run(() -> setControl(requestSupplier.get()), this);
    }

    @FunctionalInterface
    public interface DeviceConstructor {
        BaseStatusSignal create(int deviceId, String canbus);
    }

    public void registerTelemetry(Object telemetryFunction) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'registerTelemetry'");
    }
}