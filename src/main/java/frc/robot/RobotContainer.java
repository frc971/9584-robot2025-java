package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.AutoCommands;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Intake;

public class RobotContainer {
    private final NetworkTables networkTables = new NetworkTables();

    private final SwerveRequest.FieldCentric fieldCentricDrive = new SwerveRequest.FieldCentric()
        .withDeadband(networkTables.getVelocityValue(NetworkTables.ConstantId.MaxSpeed).in(MetersPerSecond) * networkTables.getDoubleValue(NetworkTables.ConstantId.ControllerDeadbandPercentage))
        .withRotationalDeadband(networkTables.getAngularRateValue(NetworkTables.ConstantId.MaxAngularRate).in(RadiansPerSecond) * networkTables.getDoubleValue(NetworkTables.ConstantId.ControllerDeadbandPercentage));

    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    private final SwerveRequest.RobotCentric robotCentricDrive = new SwerveRequest.RobotCentric();

    private final Telemetry logger = new Telemetry(networkTables.getVelocityValue(NetworkTables.ConstantId.MaxSpeed).in(MetersPerSecond));

    private final CommandXboxController controller = new CommandXboxController(0);
    private final CommandXboxController buttonBoard = new CommandXboxController(1);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    public final Climber climber = new Climber(networkTables);
    private final Intake intake = new Intake(networkTables, robotCentricDrive);
    private final AutoCommands autoCommands = new AutoCommands(intake, networkTables);

    private final SendableChooser<Command> autoChooser = AutoBuilder.buildAutoChooser("Tests");

    private final SlewRateLimiter fieldXSlewFilter = new SlewRateLimiter(networkTables.getAccelerationValue(NetworkTables.ConstantId.SlewTranslateLimit).in(MetersPerSecondPerSecond));
    private final SlewRateLimiter fieldYSlewFilter = new SlewRateLimiter(networkTables.getAccelerationValue(NetworkTables.ConstantId.SlewTranslateLimit).in(MetersPerSecondPerSecond));
    private final SlewRateLimiter fieldRotateSlewFilter = new SlewRateLimiter(networkTables.getAngularAccelerationValue(NetworkTables.ConstantId.SlewRotateLimit).in(RadiansPerSecondPerSecond));
    private final SlewRateLimiter robotXSlewFilter = new SlewRateLimiter(networkTables.getAccelerationValue(NetworkTables.ConstantId.SlewTranslateLimit).in(MetersPerSecondPerSecond));
    private final SlewRateLimiter robotYSlewFilter = new SlewRateLimiter(networkTables.getAccelerationValue(NetworkTables.ConstantId.SlewTranslateLimit).in(MetersPerSecondPerSecond));
    private final SlewRateLimiter robotRotateSlewFilter = new SlewRateLimiter(networkTables.getAngularAccelerationValue(NetworkTables.ConstantId.SlewRotateLimit).in(RadiansPerSecondPerSecond));

    // Register autonomous commands for PathPlanner
    static {
        NamedCommands.registerCommand("Eject Coral",  Commands.runOnce(() -> System.out.println("Eject Coral")));
        NamedCommands.registerCommand("Intake Coral", Commands.runOnce(() -> System.out.println("Intake Coral")));
        NamedCommands.registerCommand("Eject Algae",  Commands.runOnce(() -> System.out.println("Eject Algae")));
        NamedCommands.registerCommand("Intake Algae", Commands.runOnce(() -> System.out.println("Intake Algae")));
    }
    
    public RobotContainer() {
        SmartDashboard.putData("Auto Mode", autoChooser);
        SmartDashboard.putData("Restore Defaults", Commands.runOnce(networkTables::RestoreDefaults));

        configureBindings();
    }

    public void robotInit() {
        intake.RobotInit();
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

        controller.a().whileTrue(drivetrain.applyRequest(() -> brake));
        controller.b().whileTrue(drivetrain.applyRequest(() -> point.withModuleDirection(new Rotation2d(-controller.getLeftY(), -controller.getLeftX()))));

        controller.leftBumper().onTrue(Commands.runOnce(() -> drivetrain.seedFieldCentric()));

        buttonBoard.button(networkTables.getIntValue(NetworkTables.ConstantId.ArmUpButton))
            .onTrue(intake.ArmUpPressed())
            .onFalse(intake.ArmUpReleased());

        new Trigger(buttonBoard.povUp()).onTrue(intake.CoralEjectPressed()).onFalse(intake.CoralEjectReleased());

        new Trigger(DriverStation::isEnabled).onTrue(climber.ClimbReleased());

        for (int i = 1; i <= 11; i++) {
            buttonBoard.button(i).onTrue(Commands.print("Button " + i + " pressed"));
        }

        // Register telemetry with explicit type
        // not useable for success build
        // drivetrain.registerTelemetry(logger(Telemetry.telemeterize));

    }

    //private Consumer logger(Object telemeterize) {
        // 'todo Auto-generated method stub'
        //throw new UnsupportedOperationException("Unimplemented method 'logger'");
    //}

    public void teleopInit() {
        intake.TeleopInit();
    }

    public void autonomousInit() {
        intake.AutonomousInit();
    }

    public static double ExponentialConvert(double controllerValue, double exponent) {
        return Math.copySign(Math.pow(Math.abs(controllerValue), exponent), controllerValue);
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}