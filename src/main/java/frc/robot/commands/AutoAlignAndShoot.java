// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.FeederSetpoints;
import frc.robot.Constants.IntakeSetpoints;
import frc.robot.Constants.RobotConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.FeederBeltSubsystem;
import frc.robot.subsystems.FeederRollerSubsystem;
import frc.robot.subsystems.HoodSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.TripleShooterSubsystem;
import frc.robot.utils.AllianceUtil;
import frc.robot.utils.ShootingData;

public class AutoAlignAndShoot extends Command {

  private final CommandSwerveDrivetrain m_swerve;
  private final TripleShooterSubsystem m_shooter;
  private final HoodSubsystem m_hood;
  private final IntakeSubsystem m_intake;
  private final FeederRollerSubsystem m_feederRoller;
  private final FeederBeltSubsystem m_feederBelt;
  private final double m_toleranceDegrees;
  private SwerveRequest.FieldCentric drive;
  public Pose2d targetPose = new Pose2d();
  private double rotationVal;
  private double distanceToTarget;
  private double targetDegrees;
  private double lastTargetDegrees;

  private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond);
  private int alignedCounter;
  private boolean okRunRollers;

  private int running;
  private boolean okToRunBelt;

  public AutoAlignAndShoot(CommandSwerveDrivetrain swerve, TripleShooterSubsystem shooter, HoodSubsystem hood,
      IntakeSubsystem intake, FeederRollerSubsystem feederRoller, FeederBeltSubsystem feederBelt,
      double toleranceDegrees) {

    m_swerve = swerve;
    m_shooter = shooter;
    m_hood = hood;
    m_intake = intake;
    m_feederRoller = feederRoller;
    m_feederBelt = feederBelt;
    m_toleranceDegrees = toleranceDegrees;
    addRequirements(m_swerve);
    addRequirements(m_feederBelt);
    addRequirements(m_feederRoller);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_swerve.m_alignTargetPID.reset();
    drive = new SwerveRequest.FieldCentric()
        .withDeadband(RobotConstants.MaxSpeed * 0.1)
        .withRotationalDeadband(RobotConstants.MaxAngularRate * 0.1) // Add a 10% deadband
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    targetPose = AllianceUtil.getHubPose();
    m_swerve.isAligning = true;
    alignedCounter = 0;
    okToRunBelt = false;
    okRunRollers = false;
    m_swerve.alignedToTarget = false;
    running = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if (!okToRunBelt)
      m_feederBelt.runFeederBeltAtVelocity(FeederSetpoints.kBeltReverseRPM);

    if (!okRunRollers)
      m_feederRoller.runFeederRollerAtVelocity(FeederSetpoints.kRollersReverseRPM);

    if (!m_swerve.alignedToTarget) {

      Pose2d robotPose = m_swerve.getState().Pose;

      distanceToTarget = targetPose.getTranslation()
          .getDistance(robotPose.getTranslation());

      m_shooter.setAutoSetTargetRPM(ShootingData.shooterSpeedMap.get(distanceToTarget));

      m_hood.setAutoTargetAngle(ShootingData.hoodAngleMap.get(distanceToTarget).getDegrees());

      m_shooter.runAllVelocityVoltage();

      m_intake.runIntakeMotor(IntakeSetpoints.kIntakeSlow);

      targetDegrees = getAngleDegreesToTarget(targetPose, m_swerve.getState().Pose);

      if (targetDegrees != lastTargetDegrees) {
        m_swerve.m_alignTargetPID.setSetpoint(targetDegrees);
        lastTargetDegrees = targetDegrees;
      }

      if (m_swerve.alignedToTarget || Math.abs(m_swerve.m_alignTargetPID.getError()) > m_swerve.alignIzone) {
        m_swerve.m_alignTargetPID.setIntegratorRange(0, 0);
      } else
        m_swerve.m_alignTargetPID.setIntegratorRange(-.1, .1);

      rotationVal = m_swerve.m_alignTargetPID.calculate(m_swerve.getState().Pose.getRotation().getDegrees(),
          targetDegrees);

      m_swerve.setControl(drive
          .withVelocityX(0)
          .withVelocityY(0)
          .withRotationalRate(rotationVal * MaxAngularRate));

      if (Math.abs(m_swerve.m_alignTargetPID.getError()) < m_toleranceDegrees) {
        alignedCounter++;
      } else
        alignedCounter = 0;

      m_swerve.alignedToTarget = alignedCounter >= 10;

      DogLog.log("AutoAlign/AlignedToHub", m_swerve.alignedToTarget);
      DogLog.log("AutoAlign/AlignError", m_swerve.m_alignTargetPID.getError());
      DogLog.log("AutoAlign/AlignDistance", distanceToTarget);
      DogLog.log("AutoAlign/AlignAngle", targetDegrees);
      DogLog.log("AutoAlign/AccumIntegral", m_swerve.m_alignTargetPID.getAccumulatedError());
      DogLog.log("AutoAlign/RotationVal", rotationVal);
    }

    if (m_swerve.alignedToTarget) {

      m_shooter.runAllVelocityVoltage();

       m_intake.runIntakeMotor(IntakeSetpoints.kIntakeSlow);

      if (m_shooter.allVelocityInTolerance() && m_hood.isPositionWithinTolerance()) {
        okRunRollers = true;

        if (okRunRollers) {
          m_feederRoller.runFeederRollerAtVelocity();
          if (!okToRunBelt)
            running++;

          if (Math.abs(
              m_feederRoller.feederRollerMotor.getEncoder().getVelocity()) > FeederSetpoints.rollerSpeedToStartBelt
              || running > 50)
            okToRunBelt = true;

          if (okToRunBelt)
            m_feederBelt.runFeederBeltAtVelocity(FeederSetpoints.kBeltShootRPM);
        }

      }

      DogLog.log("AutoShoot/OKTOShoot", okRunRollers);
      DogLog.log("AutoShoot/OKRunBelt", okToRunBelt);
      DogLog.log("AutoShoot/ShootersAtSpeed", m_shooter.allVelocityInTolerance());
      DogLog.log("AutoShoot/HoodAtTarget", m_hood.isPositionWithinTolerance());
      DogLog.log("AutoShoot/Aligned", m_swerve.alignedToTarget);
      DogLog.log("AutoShoot/Running", running);

    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_swerve.m_alignTargetPID.reset();
    m_swerve.isAligning = false;
    m_swerve.setControl(drive
        .withVelocityX(0)
        .withVelocityY(0)
        .withRotationalRate(0));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  public double getAngleDegreesToTarget(Pose2d targetPose, Pose2d robotPose) {
    double XDiff = targetPose.getX() - robotPose.getX();
    double YDiff = targetPose.getY() - robotPose.getY();
    return Units.radiansToDegrees(Math.atan2(YDiff, XDiff));
  }

}
