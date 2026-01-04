package frc.robot.commands;

import java.util.function.DoubleSupplier;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.VisionSubsystem;
import com.ctre.phoenix6.swerve.SwerveRequest;

public class VisionAlign extends Command {
    private final CommandSwerveDrivetrain m_drivetrain;
    private final VisionSubsystem m_vision;
    private final DoubleSupplier m_throttleX, m_throttleY;

    // PID constants for rotation alignment
    private final PIDController m_controller = new PIDController(0.05, 0.0, 0.002);
    private final SwerveRequest.FieldCentric m_driveRequest = new SwerveRequest.FieldCentric();

    public VisionAlign(CommandSwerveDrivetrain drivetrain, VisionSubsystem vision,
                       DoubleSupplier throttleX, DoubleSupplier throttleY) {
        m_drivetrain = drivetrain;
        m_vision = vision;
        m_throttleX = throttleX;
        m_throttleY = throttleY;
        addRequirements(m_drivetrain);
    }

    @Override
    public void execute() {
        double rotationOutput = 0;

        if (m_vision.hasTarget()) {
            // Error is the horizontal offset from the crosshair
            rotationOutput = m_controller.calculate(m_vision.getTX(), 0);
        }

        // Apply velocities: X and Y from sticks, Rotation from PID
        m_drivetrain.setControl(m_driveRequest
                .withVelocityX(m_throttleX.getAsDouble())
                .withVelocityY(m_throttleY.getAsDouble())
                .withRotationalRate(rotationOutput));
    }
}