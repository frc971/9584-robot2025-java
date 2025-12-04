package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.FollowPathCommand;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.AutoCommands.AutoCommands;
import frc.robot.subsystems.Climber.Climber;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.Superstructure;
import frc.robot.NetworkTables;

public class RobotContainer extends TimedRobot {
    private final NetworkTables networkTables = new NetworkTables();

    private final SwerveRequest.FieldCentric fieldCentricDrive = new SwerveRequest.FieldCentric()
        .withDeadband(networkTables.getVelocityValue(NetworkTables.ConstantId.MaxSpeed).in(MetersPerSecond) * networkTables.getDoubleValue(NetworkTables.ConstantId.ControllerDeadbandPercentage))
        .withRotationalDeadband(networkTables.getAngularRateValue(NetworkTables.ConstantId.MaxAngularRate).in(RadiansPerSecond) * networkTables.getDoubleValue(NetworkTables.ConstantId.ControllerDeadbandPercentage));

    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    private final SwerveRequest.RobotCentric robotCentricDrive = new SwerveRequest.RobotCentric();

    private double MAX_SPEED =
      TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);

    private final Telemetry logger = new Telemetry(MAX_SPEED);
    
    private final CommandXboxController controller = new CommandXboxController(0);
    private final CommandJoystick simController = new CommandJoystick(0);
    private final CommandJoystick buttonBoard = new CommandJoystick(1);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    public final Climber climber = new Climber(networkTables);
    private final Superstructure superstructure = new Superstructure(networkTables, drivetrain);
    private final AutoCommands autoCommands = new AutoCommands(superstructure, networkTables);

    private final SendableChooser<Command> autoChooser;

    private final SlewRateLimiter fieldXSlewFilter = new SlewRateLimiter(networkTables.getAccelerationValue(NetworkTables.ConstantId.SlewTranslateLimit).in(MetersPerSecondPerSecond));
    private final SlewRateLimiter fieldYSlewFilter = new SlewRateLimiter(networkTables.getAccelerationValue(NetworkTables.ConstantId.SlewTranslateLimit).in(MetersPerSecondPerSecond));
    private final SlewRateLimiter fieldRotateSlewFilter = new SlewRateLimiter(networkTables.getAngularAccelerationValue(NetworkTables.ConstantId.SlewRotateLimit).in(RadiansPerSecondPerSecond));
    private final SlewRateLimiter robotXSlewFilter = new SlewRateLimiter(networkTables.getAccelerationValue(NetworkTables.ConstantId.SlewTranslateLimit).in(MetersPerSecondPerSecond));
    private final SlewRateLimiter robotYSlewFilter = new SlewRateLimiter(networkTables.getAccelerationValue(NetworkTables.ConstantId.SlewTranslateLimit).in(MetersPerSecondPerSecond));
    private final SlewRateLimiter robotRotateSlewFilter = new SlewRateLimiter(networkTables.getAngularAccelerationValue(NetworkTables.ConstantId.SlewRotateLimit).in(RadiansPerSecondPerSecond));

    // Register autonomous commands for PathPlanner

    public RobotContainer() {
        NamedCommands.registerCommand("Eject Coral", superstructure.AutoCoral());

        autoChooser = AutoBuilder.buildAutoChooser("Tests");
        SmartDashboard.putData("Auto Mode", autoChooser);
        SmartDashboard.putData("Restore Defaults", Commands.runOnce(networkTables::RestoreDefaults));
        
        if (!RobotBase.isSimulation()){
            configureBindings();
        }
        else{
            configureSimBindings();
        }

        // Warmup PathPlanner to avoid Java pauses
        FollowPathCommand.warmupCommand().schedule();
    }

    public void robotInit() {
        
        SmartDashboard.putBoolean("IsSimulation", RobotBase.isSimulation());

        if (RobotBase.isSimulation()) {
            System.setProperty("phoenix.staleCheckingEnabled", "false"); // Disable stale checking in simulation
        }

        superstructure.RobotInit();
    }

    private void configureBindings() {
        drivetrain.setDefaultCommand(
            drivetrain.applyRequest(() -> {
                double exponentVelocity = networkTables.getDoubleValue(NetworkTables.ConstantId.ControllerVelocityCurveExponent);
                double exponentRotation = networkTables.getDoubleValue(NetworkTables.ConstantId.ControllerRotationCurveExponent);
                if (!controller.rightBumper().getAsBoolean()) {
                    double fieldX = fieldXSlewFilter.calculate(
                        networkTables.getVelocityValue(NetworkTables.ConstantId.MaxSpeed).in(MetersPerSecond) * ExponentialConvert(-controller.getLeftY(), exponentVelocity)
                    );
                    double fieldY = fieldYSlewFilter.calculate(
                        networkTables.getVelocityValue(NetworkTables.ConstantId.MaxSpeed).in(MetersPerSecond) * ExponentialConvert(-controller.getLeftX(), exponentVelocity)
                    );
                    double fieldRotate = fieldRotateSlewFilter.calculate(
                        networkTables.getAngularRateValue(NetworkTables.ConstantId.MaxAngularRate).in(RadiansPerSecond) * ExponentialConvert(-controller.getRightX(), exponentRotation)
                    );
                    return fieldCentricDrive.withVelocityX(fieldX).withVelocityY(fieldY).withRotationalRate(fieldRotate);
                } else {
                    double robotX = robotXSlewFilter.calculate(
                        networkTables.getVelocityValue(NetworkTables.ConstantId.MaxSpeed).in(MetersPerSecond) * ExponentialConvert(-controller.getLeftY(), exponentVelocity)
                    );
                    double robotY = robotYSlewFilter.calculate(
                        networkTables.getVelocityValue(NetworkTables.ConstantId.MaxSpeed).in(MetersPerSecond) * ExponentialConvert(-controller.getLeftX(), exponentVelocity)
                    );
                    double robotRotate = robotRotateSlewFilter.calculate(
                        networkTables.getAngularRateValue(NetworkTables.ConstantId.MaxAngularRate).in(RadiansPerSecond) * ExponentialConvert(-controller.getRightX(), exponentRotation)
                    );
                    return robotCentricDrive.withVelocityX(robotX).withVelocityY(robotY).withRotationalRate(robotRotate);
                }
            })
        );

        new Trigger(controller.a().whileTrue(drivetrain.applyRequest(() -> brake)));
        new Trigger(controller.b().whileTrue(drivetrain.applyRequest(() -> point.withModuleDirection(new Rotation2d(-controller.getLeftY(), -controller.getLeftX())))));

        new Trigger(controller.leftBumper().onTrue(Commands.runOnce(() -> drivetrain.seedFieldCentric())));

        // ButtonBoard bindings
        new Trigger(buttonBoard.button(networkTables.getIntValue(NetworkTables.ConstantId.ArmUpButton))
            .onTrue(superstructure.ArmUpPressed())
            .onFalse(superstructure.ArmUpReleased()));
        
        new Trigger(buttonBoard.button(networkTables.getIntValue(NetworkTables.ConstantId.ArmDownButton))
            .onTrue(superstructure.ArmDownPressed())
            .onFalse(superstructure.ArmDownReleased()));

        new Trigger(buttonBoard
            .button(networkTables.getIntValue(NetworkTables.ConstantId.RollerForwardButton))
            .onTrue(superstructure.RollerForwardPressed())
            .onFalse(superstructure.RollerForwardReleased()));

        new Trigger(buttonBoard
            .button(networkTables.getIntValue(NetworkTables.ConstantId.RollerBackwardButton))
            .onTrue(superstructure.RollerBackwardPressed())
            .onFalse(superstructure.RollerBackwardReleased()));

        new Trigger(buttonBoard.button(networkTables.getIntValue(NetworkTables.ConstantId.ClimbButton))
            .onTrue(climber.ClimbPressed())
            .onFalse(climber.ClimbReleased())); //working

        new Trigger(buttonBoard.button(networkTables.getIntValue(NetworkTables.ConstantId.UnclimbButton))
            .onTrue(climber.UnclimbPressed())
            .onFalse(climber.UnclimbReleased())); //working

        new Trigger(buttonBoard.button(networkTables.getIntValue(NetworkTables.ConstantId.ResetEncoderButton))
            .onTrue(superstructure.ResetEncoderPositionCommand()));

        new Trigger(buttonBoard
            .axisGreaterThan(
                networkTables.getIntValue(NetworkTables.ConstantId.AlgaeIntakeButtonAxis), 0.75)
            .onTrue(superstructure.AlgaeIntakePressed())
            .onFalse(superstructure.AlgaeIntakeReleased()));

        new Trigger(buttonBoard
            .axisGreaterThan(
                networkTables.getIntValue(NetworkTables.ConstantId.AlgaeEjectButtonAxis), 0.75)
            .onTrue(superstructure.AlgaeEjectPressed())
            .onFalse(superstructure.AlgaeEjectReleased()));

        new Trigger(buttonBoard.povUp().onTrue(superstructure.CoralEjectPressed()).onFalse(superstructure.CoralEjectReleased()));

        new Trigger(DriverStation::isEnabled).onTrue(climber.ClimbReleased()); //working

        for (int i = 1; i <= 11; i++) {
            buttonBoard.button(i).onTrue(Commands.print("Button " + i + " pressed"));
        }

        // Register telemetry with explicit type
        // fixed
        drivetrain.registerTelemetry(logger::telemeterize);

    }

    public void configureSimBindings(){
        drivetrain.setDefaultCommand(
            drivetrain.applyRequest(() -> {
                double exponentVelocity = networkTables.getDoubleValue(NetworkTables.ConstantId.ControllerVelocityCurveExponent);
                double exponentRotation = networkTables.getDoubleValue(NetworkTables.ConstantId.ControllerRotationCurveExponent);
                if (!simController.button(1).getAsBoolean()) {
                    double fieldX = fieldXSlewFilter.calculate(
                        networkTables.getVelocityValue(NetworkTables.ConstantId.MaxSpeed).in(MetersPerSecond) * ExponentialConvert(-controller.getRawAxis(0), exponentVelocity)
                    );
                    double fieldY = fieldYSlewFilter.calculate(
                        networkTables.getVelocityValue(NetworkTables.ConstantId.MaxSpeed).in(MetersPerSecond) * ExponentialConvert(-controller.getRawAxis(1), exponentVelocity)
                    );
                    double fieldRotate = fieldRotateSlewFilter.calculate(
                        networkTables.getAngularRateValue(NetworkTables.ConstantId.MaxAngularRate).in(RadiansPerSecond) * ExponentialConvert(-controller.getRawAxis(2), exponentRotation)
                    );
                    return fieldCentricDrive.withVelocityX(fieldX).withVelocityY(fieldY).withRotationalRate(fieldRotate);
                } else {
                    double robotX = robotXSlewFilter.calculate(
                        networkTables.getVelocityValue(NetworkTables.ConstantId.MaxSpeed).in(MetersPerSecond) * ExponentialConvert(-controller.getRawAxis(0), exponentVelocity)
                    );
                    double robotY = robotYSlewFilter.calculate(
                        networkTables.getVelocityValue(NetworkTables.ConstantId.MaxSpeed).in(MetersPerSecond) * ExponentialConvert(-controller.getRawAxis(1), exponentVelocity)
                    );
                    double robotRotate = robotRotateSlewFilter.calculate(
                        networkTables.getAngularRateValue(NetworkTables.ConstantId.MaxAngularRate).in(RadiansPerSecond) * ExponentialConvert(-controller.getRawAxis(2), exponentRotation)
                    );
                    return robotCentricDrive.withVelocityX(robotX).withVelocityY(robotY).withRotationalRate(robotRotate);
                }
            })
        );
    }

    public void teleopInit() {
        superstructure.TeleopInit();
    }

    public void autonomousInit() {
        superstructure.AutonomousInit();
    }

    public static double ExponentialConvert(double controllerValue, double exponent) {
        return Math.copySign(Math.pow(Math.abs(controllerValue), exponent), controllerValue);
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}