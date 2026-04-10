// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RPM;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.events.EventTrigger;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.CameraConstants;
import frc.robot.Constants.FeederSetpoints;
import frc.robot.Constants.RobotConstants;
import frc.robot.commands.AlignTargetOdometry;
import frc.robot.commands.AutoAlignAndShoot;
import frc.robot.commands.ShootCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.FeederBeltSubsystem;
import frc.robot.subsystems.FeederRollerSubsystem;
import frc.robot.subsystems.HoodSubsystem;
import frc.robot.subsystems.Intake4BarArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LimelightVision;
import frc.robot.subsystems.TripleShooterSubsystem;
import frc.robot.utils.ShootingData;

public class RobotContainer {

        /* Setting up bindings for necessary control of the swerve drive platform */
        private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
                        .withDeadband(RobotConstants.MaxSpeed * 0.1)
                        .withRotationalDeadband(RobotConstants.MaxAngularRate * 0.1) // Add a 10% deadband
                        .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive
                                                                                 // motors
        private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();

        private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

        private final SwerveRequest.FieldCentric forwardStraight = new SwerveRequest.FieldCentric()
                        .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

        private final SwerveRequest.FieldCentric forwardStraightVelocity = new SwerveRequest.FieldCentric()
                        .withDriveRequestType(DriveRequestType.Velocity);

        private final SwerveRequest.FieldCentricFacingAngle driveFacingAngle = new SwerveRequest.FieldCentricFacingAngle()
                        .withDeadband(RobotConstants.MaxSpeed * 0.1)
                        .withRotationalDeadband(RobotConstants.MaxAngularRate * 0.1) // Add a 10% deadband
                        .withHeadingPID(7, 0, 0)
                        .withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage); // Use open-loop control
                                                                                              // for drive motors

        public final CommandXboxController driver = new CommandXboxController(0);
        public final CommandXboxController codriver = new CommandXboxController(1);
        public final CommandXboxController presetdriver = new CommandXboxController(2);

        public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

        private final Telemetry logger = new Telemetry(RobotConstants.MaxSpeed, drivetrain);

        /* Path follower */
        private SendableChooser<Command> autoChooser;

        final TripleShooterSubsystem m_shooter;

        final HoodSubsystem m_hood;

        final FeederRollerSubsystem m_feederRoller;
        final FeederBeltSubsystem m_feederBelt;

        final IntakeSubsystem m_intake;

        final Intake4BarArmSubsystem m_intakeArm;

        public final LimelightVision m_llv;

        public final PowerDistribution pdh;

        private boolean logShooterData = true;
        private boolean logHoodData = true;
        private boolean logFeederBeltData = true;
        private boolean logFeederRollerData = true;
        private boolean logIntakeData = true;
        private boolean logIntakeArmData = true;
        private boolean logLLData = true;

        private Trigger collisionTrigger;

        // Presets
        // public static final double hubPresetDistance = 0.96;
        public static final double towerPresetDistance = 4.41;
        public static final double trenchPresetDistance = 3.3;
        // public static final double outpostPresetDistance = 4.84;

        public RobotContainer() {

                m_shooter = new TripleShooterSubsystem(logShooterData);
                m_hood = new HoodSubsystem(logHoodData);
                m_feederRoller = new FeederRollerSubsystem(logFeederRollerData);
                m_feederBelt = new FeederBeltSubsystem(logFeederBeltData);
                m_intake = new IntakeSubsystem(logIntakeData);
                m_intakeArm = new Intake4BarArmSubsystem(logIntakeArmData);
                m_llv = new LimelightVision(logLLData);
                pdh = new PowerDistribution(Constants.CANIDConstants.pdh, ModuleType.kRev);
                pdh.clearStickyFaults();
                registerNamedCommands();
                registerEventTriggers();
                // configurePDH();
                SmartDashboard.putData("PDH", pdh);
                m_shooter.leftMotorActive = true;
                m_shooter.middleMotorActive = true;
                m_shooter.rightMotorActive = true;

                setDefaultCommands();
                configureDriverBindings();
                configureCodriverBindings();

                configureTriggers();
                buildAutoChooser();

                SignalLogger.setPath("media/sda1/ctre-logs");

                drivetrain.registerTelemetry(logger::telemeterize);
        }

        private void setDefaultCommands() {
                drivetrain.setDefaultCommand(
                                // Drivetrain will execute this command periodically
                                drivetrain.applyRequest(() -> drive
                                                .withForwardPerspective(ForwardPerspectiveValue.OperatorPerspective)
                                                .withVelocityX(-driver.getLeftY() * RobotConstants.MaxSpeed)
                                                .withVelocityY(-driver.getLeftX() * RobotConstants.MaxSpeed)
                                                // negative X (left)
                                                .withRotationalRate(
                                                                -driver.getRightX() * RobotConstants.MaxAngularRate)));

                m_intakeArm.setDefaultCommand(m_intakeArm.positionIntakeArmCommand());

                m_hood.setDefaultCommand(m_hood.positionHoodCommand());

                // m_intake.setDefaultCommand(m_intake.stopIntakeCommand().withName("IntakeDefault")
                // .andThen(() -> {
                // }));

                // m_shooter.setDefaultCommand(m_shooter.stopAllShootersCommand().withName("ShooterDefault")
                // .andThen(() -> {
                // }));

                // m_feederRoller.setDefaultCommand(m_feederRoller.stopFeederRollerCommand().withName("RollersDefault")
                // .andThen(() -> {
                // }));
                m_feederBelt.setDefaultCommand(m_feederBelt.stopFeederBeltCommand().withName("BeltDefault")
                                .andThen(() -> {
                                }));
        }

        /**
         * Rollers and belt conditions
         * rollers and belts have stop default commands
         * a)-Feeder rollers and belts in reverse to keep fuel clear of rollers
         * b)-Feeder rollers in reverse and belt move forward and reverse to unstick
         * fuel
         * c)- Belt in reverse, until rollers at speed for shooting
         * d) Feeder rollers run shoot speed with belts in reverse
         * e) Feeder rollers abd belts run shoot speed
         * 
         * During intake do b) then a) when intaking ends
         * Shoot commands does a) initially as well as starting shooters
         * when shooters at speed, do c) rollers are started at shoot speed and
         * direction
         * when rollers at speed belt is started at shoot speed
         * 
         */

        private void configureDriverBindings() {
                // Note that X is defined as forward according to WPILib convention,
                // and Y is defined as to the left according to WPILib convention.

                // Idle while the robot is disabled. This ensures the configured
                // neutral mode is applied to the drive motors while disabled.

                final var idle = new SwerveRequest.Idle();
                RobotModeTriggers.disabled().whileTrue(
                                drivetrain.applyRequest(() -> idle).ignoringDisable(true));

                driver.leftTrigger().whileTrue(
                                new ShootCommand(m_shooter, m_hood, m_feederRoller, m_feederBelt,
                                                m_intake, drivetrain,
                                                false).withName("ShootCommand"));

                driver.rightTrigger().whileTrue(
                                Commands.parallel(
                                                m_intakeArm.intakeArmToIntakeAngleCommand(),
                                                m_intake.runIntakeAtVelocityCommand(),
                                                unstickFuelCommand()).withName("TeleopIntake"))
                                .onFalse(m_intake.startIntakeCommandSlow());

                driver.leftBumper().onTrue(
                                Commands.sequence(
                                                setForAutoShootValues(),
                                                m_shooter.runAllVelocityVoltageCommand()))
                                .whileTrue(
                                                Commands.parallel(unstickFuelCommand(),
                                                                new AlignTargetOdometry(drivetrain, m_shooter, m_hood,
                                                                                drive,
                                                                                driver, 1.75)));

                driver.rightBumper().onTrue(
                                Commands.parallel(
                                                setForAutoShootValues(),
                                                stopShootersFeedersIntake(),
                                                m_intakeArm.intakeArmToClearAngleCommand()));

                driver.y().whileTrue(unstickFuelCommand());

                driver.b().onTrue(presetShoot(trenchPresetDistance));

                driver.x().onTrue(presetShoot(towerPresetDistance));

                driver.a().whileTrue(m_intakeArm.helpShootCommand(.5, Degrees.of(2)));

                driver.povLeft().whileTrue(m_feederRoller.jogFeederRollerCommand());

                driver.povRight().whileTrue(m_feederRoller.jogReverseFeederRollerCommand());

                driver.povUp().onTrue(m_feederRoller.runFeedRollerAtVelocityCommand());

                driver.povDown().onTrue(m_feederRoller.stopFeederRollerCommand());

                driver.back().onTrue(Commands.runOnce(() -> drivetrain.getPigeon2().reset()));

                // Reset the field-centric heading
                driver.start().onTrue(
                                drivetrain.runOnce(drivetrain::seedFieldCentric));

        }

        private void configureCodriverBindings() {
                // Run SysId routines when holding back/start and X/Y.
                // Note that each routine should be run exactly once in a single log.
                // codriver.back().and(codriver.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
                // codriver.back().and(codriver.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));

                // codriver.start().and(codriver.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
                // codriver.start().and(codriver.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));
                // codriver.start().and(codriver.povRight()).onTrue(Commands.runOnce(() ->
                // SignalLogger.stop()));

                codriver.leftBumper()
                                // .whileTrue(m_intakeArm.jogIntakeArmCommand(() -> codriver.getLeftX() / 4));
                                .onTrue(Commands.deadline(
                                                m_feederBelt.unstickFuelCommand(
                                                                -.25, -.25, .25, .1),

                                                Commands.run(() -> m_feederRoller.runFeederRollerAtVelocity(-100)))
                                                .andThen(m_feederRoller.stopFeederRollerCommand()));

                codriver.rightBumper().whileTrue(m_feederRoller.jogReverseFeederRollerCommand());

                // codriver.rightBumper().and(codriver.a()).onTrue(m_intakeArm.intakeArmToIntakeAngleCommand());

                // codriver.rightBumper().and(codriver.b()).onTrue(m_intakeArm.intakeArmToMidUpAngleCommand());

                // codriver.rightBumper().and(codriver.x()).onTrue(m_intakeArm.intakeArmToMidDownAngleCommand());

                // codriver.rightBumper().and(codriver.povUp()).whileTrue(m_intakeArm.helpShootCommand(1,
                // Degrees.of(2)));

                // codriver.rightBumper().and(codriver.povDown())
                // .onTrue(clearRevStickyFaultsCommand()
                // .alongWith(clearShooterStickyFaultsCommand()));

                codriver.rightTrigger().and(codriver.povUp())
                                .whileTrue(m_feederBelt.jogFeederBeltCommand(() -> .2));

                codriver.rightTrigger().and(codriver.povDown()).onTrue(Commands.none());

                codriver.rightTrigger().and(codriver.y()).whileTrue(
                                m_shooter.setDutyCycleCommand(m_shooter.middleMotor, .5))
                                .onFalse(m_shooter.setDutyCycleCommand(m_shooter.middleMotor, .0));

                codriver.rightTrigger().and(codriver.a()).whileTrue(
                                m_shooter.setDutyCycleCommand(m_shooter.leftMotor, .5))
                                .onFalse(m_shooter.setDutyCycleCommand(m_shooter.leftMotor, .0));

                codriver.rightTrigger().and(codriver.x()).whileTrue(
                                m_shooter.setDutyCycleCommand(m_shooter.rightMotor, .5))
                                .onFalse(m_shooter.setDutyCycleCommand(m_shooter.rightMotor, .0));

                codriver.rightTrigger().and(codriver.povLeft()).whileTrue(m_hood.jogHoodUpCommand());

                codriver.rightTrigger().and(codriver.povRight()).whileTrue(m_hood.jogHoodDownCommand());

                codriver.leftTrigger().and(codriver.povLeft())
                                .onTrue(m_hood.setHoodZeroCommand().ignoringDisable(true));

                codriver.leftTrigger().and(codriver.povUp())
                                .onTrue(Commands.sequence(
                                                Commands.runOnce(() -> m_llv.useMT1 = true),
                                                (Commands.runOnce(() -> m_llv.useMT2 = false))));

                codriver.leftTrigger().and(codriver.povDown()).whileTrue(m_intake.jogIntakeCommand());

                codriver.leftTrigger().and(codriver.a()).onTrue(
                                Commands.runOnce(() -> m_llv.setCamToRobotOffset(CameraConstants.frontCamera)));

                codriver.leftTrigger().and(codriver.y()).onTrue(
                                Commands.runOnce(() -> m_llv
                                                .setCamToRobotOffsetInvertPitch(CameraConstants.frontCamera)));

        }

        private void configureTriggers() {

                collisionTrigger = new Trigger(() -> drivetrain.jerkLimitExceeded);

                // collisionTrigger.onTrue(m_intakeArm.intakeArmUpCommand());

        }

        private void configurePDH() {

                /**
                 * 0 - BRSteer
                 * 1 -
                 * 2-
                 * 3-
                 * 4 -
                 * 5- intake slide arm
                 * 6 - FRSteer
                 * 7
                 * 8
                 * 9 - FRDrive
                 * 10 - FLDrive
                 * 11-
                 * 12-
                 * 13- FLSteer
                 * 14-
                 * 15-
                 * 16
                 * 17-BLSteer
                 * 18-BRDrive
                 * 19-BLDrive
                 * 
                 */

        }

        private void buildAutoChooser() {
                // Build an auto chooser. This will use Commands.none() as the default option.
                // As an example, this will only show autos that start with "comp" while at
                // competition as defined by the programmer

                autoChooser = AutoBuilder.buildAutoChooser();

                SmartDashboard.putData("PPAutoChooser", autoChooser);
        }

        private void registerNamedCommands() {

                NamedCommands.registerCommand("ALIGN_AND_SHOOT",

                                Commands.parallel(
                                                new AutoAlignAndShoot(
                                                                drivetrain, m_shooter,
                                                                m_hood, m_intake,
                                                                m_feederRoller,
                                                                m_feederBelt, 1.75),
                                                Commands.sequence(
                                                                Commands.waitSeconds(4),
                                                                m_intakeArm.helpShootCommand(
                                                                                .75,
                                                                                Degrees.of(2))))
                                                .withTimeout(12)
                                                .andThen(stopShootersFeedersIntake()));

                NamedCommands.registerCommand("FUS",

                                unstickFuelCommand());

        }

        private void registerEventTriggers() {

                EventTrigger runIntake = new EventTrigger("RUN_INTAKE");
                runIntake.onTrue(
                                Commands.sequence(
                                                m_intakeArm.intakeArmToIntakeAngleCommand(),
                                                m_intake.startIntakeCommand(),
                                                Commands.waitSeconds(.5)));
                                               

        }

        public Command getAutonomousCommand() {
                /* Run the path selected from the auto chooser */
                return autoChooser.getSelected();
        }

        public Command clearRevStickyFaultsCommand() {
                return Commands.sequence(
                                m_feederBelt.clearFeederBeltStickyFaultsCommand(),
                                m_feederRoller.clearFeederRollerStickyFaultsCommand(),
                                m_hood.clearHoodStickyFaultsCommand(),
                                m_intake.clearIntakeStickyFaultsCommand(),
                                m_intakeArm.clearIntakeArmStickyFaultsCommand());
        }

        public Command clearShooterStickyFaultsCommand() {
                return m_shooter.clearShooterStickyFaultsCommand();
        }

        public Command presetShoot(double distance) {
                return Commands.sequence(
                                Commands.parallel(
                                                setForManualShootValues(),
                                                m_shooter.setManualTargetVelocityCommand(RPM.of(2200)),
                                                // m_shooter.setManualTargetVelocityCommand(
                                                // RPM.of(ShootingData.shooterSpeedMap.get(distance))),
                                                m_hood.setManualTargetCommand(
                                                                ShootingData.hoodAngleMap.get(distance).getDegrees()))

                                                .andThen(
                                                                Commands.parallel(
                                                                                new ShootCommand(m_shooter, m_hood,
                                                                                                m_feederRoller,
                                                                                                m_feederBelt,
                                                                                                m_intake, drivetrain,
                                                                                                true),
                                                                                Commands.sequence(
                                                                                                Commands.waitSeconds(5),
                                                                                                m_intakeArm.helpShootCommand(
                                                                                                                .75,
                                                                                                                Degrees.of(2.5))))))

                                .finallyDo((() -> stopShootersFeedersIntake()));

        }

        public Command setForManualShootValues() {
                return Commands.parallel(
                                m_shooter.setShootUsingDistanceCommand(false),
                                m_hood.setHoodUsingDistanceCommand(false));

        }

        public Command setForAutoShootValues() {
                return Commands.parallel(
                                m_shooter.setShootUsingDistanceCommand(true),
                                m_hood.setHoodUsingDistanceCommand(true));

        }

        public Command setForPresetShootValues() {
                return Commands.parallel(
                                m_shooter.setShootUsingDistanceCommand(false),
                                m_hood.setHoodUsingDistanceCommand(false));
        }

        public Command stopShootersFeedersIntake() {
                return Commands.sequence(
                                m_hood.setManualTargetCommand(HoodSubsystem.kMinPosition.in(Degrees)),
                                m_shooter.stopAllShootersCommand(),
                                m_shooter.endShootCommand(),
                                m_feederRoller.stopFeederRollerCommand(),
                                m_feederBelt.stopFeederBeltCommand(),
                                m_intake.stopIntakeCommand()).withName("StopAll");
        }

        public Command unstickFuelCommand() {
                return Commands.parallel(
                                m_feederBelt.unstickFuelCommand(25, -.25, .25, .1),
                                Commands.run(() -> m_feederRoller
                                                .runFeederRollerAtVelocity(FeederSetpoints.kRollersReverseRPM),
                                                m_feederRoller))
                                .withName("UnstickFuel");
        }

}
