package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.Limelights.Constants;
import frc.robot.subsystems.Limelights.LimelightHelpers;
import frc.robot.subsystems.Led.LEDSubsystem;

public class Robot extends TimedRobot {

    private static final double kLowBatteryVoltage = 11.8;
    private static final double kLowBatteryDisabledTime = 1.5; // seconds

    private final Timer batteryTimer = new Timer();
    private boolean lowBatteryAlert = false;

    private Command m_autonomousCommand;

    private final RobotContainer m_container = new RobotContainer();
    private final LEDSubsystem m_leds = new LEDSubsystem();

    private static final boolean kUseLimelight = true;

    /**
     * Initial configuration for all Limelights
     */
    private void configureLimelights() {
        for (String limelightName : Constants.LimelightConstants.limelightNames) {
            LimelightHelpers.setLEDMode_PipelineControl(limelightName);
            LimelightHelpers.setPipelineIndex(limelightName, 0);
            System.out.println("Configured Limelight: " + limelightName);
        }
    }

    @Override
    public void robotInit() {
        m_container.robotInit();

        if (kUseLimelight) {
            configureLimelights();
        }

        if (RobotBase.isSimulation()) {
            DriverStation.silenceJoystickConnectionWarning(true);
        }

        batteryTimer.start();
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();

        // Standard Robot Telemetry
        SmartDashboard.putNumber("Voltage", RobotController.getBatteryVoltage());
        SmartDashboard.putNumber("CPU Temperature", RobotController.getCPUTemp());
        SmartDashboard.putBoolean("RSL", RobotController.getRSLState());
        SmartDashboard.putNumber("Match Time", DriverStation.getMatchTime());
        SmartDashboard.putNumber("Code Runtime (ms)", Timer.getFPGATimestamp() * 1000.0);

        // Limelight Updates
        // This handles all telemetry and MegaTag2 odometry fusion internally
        if (kUseLimelight) {
            m_container.visionSubsystem.updateOdometry(m_container.drivetrain);
        }

        // Battery Alert Logic
        if (DriverStation.isEnabled()) {
            batteryTimer.reset();
        }

        lowBatteryAlert = (RobotController.getBatteryVoltage() <= kLowBatteryVoltage
                && batteryTimer.hasElapsed(kLowBatteryDisabledTime));
        SmartDashboard.putBoolean("LowBattery", lowBatteryAlert);
    }

    @Override
    public void autonomousInit() {
        // Signal to VisionSubsystem to start the 200ms safety delay
        if (kUseLimelight) {
            m_container.visionSubsystem.resetAutoTimer();
        }

        m_autonomousCommand = m_container.getAutonomousCommand();
        if (m_autonomousCommand != null) {
            m_autonomousCommand.schedule();
        }

        m_container.autonomousInit();
        m_leds.setAutonomousMode();

        if (kUseLimelight) {
            for (String limelightName : Constants.LimelightConstants.limelightNames) {
                LimelightHelpers.setPipelineIndex(limelightName, 0);
                LimelightHelpers.setLEDMode_PipelineControl(limelightName);
            }
        }
    }

    @Override
    public void teleopInit() {
        if (m_autonomousCommand != null) {
            m_autonomousCommand.cancel();
        }

        m_container.teleopInit();
        m_leds.setTeleopMode();

        if (kUseLimelight) {
            for (String limelightName : Constants.LimelightConstants.limelightNames) {
                LimelightHelpers.setPipelineIndex(limelightName, 0);
                LimelightHelpers.setLEDMode_PipelineControl(limelightName);
            }
        }
    }

    @Override
    public void disabledInit() {
        m_leds.setDisabledMode();
    }

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    // Unused methods kept for structure
    @Override public void testPeriodic() {}
    @Override public void disabledPeriodic() {}
    @Override public void autonomousPeriodic() {}
    @Override public void teleopPeriodic() {}
    @Override public void simulationPeriodic() {}
}