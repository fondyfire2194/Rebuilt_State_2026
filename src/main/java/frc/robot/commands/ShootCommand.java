// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import dev.doglog.DogLog;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.FeederSetpoints;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.FeederBeltSubsystem;
import frc.robot.subsystems.FeederRollerSubsystem;
import frc.robot.subsystems.HoodSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.TripleShooterSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ShootCommand extends Command {
  /** Creates a new ShootCommand. */
  private final TripleShooterSubsystem m_shooter;
  private final HoodSubsystem m_hood;
  private final FeederRollerSubsystem m_feederRoller;
  private final FeederBeltSubsystem m_feederBelt;
  private final IntakeSubsystem m_intake;
  private final CommandSwerveDrivetrain m_swerve;
  private boolean m_bypassAlign;
  private boolean okRunRollers;
  private boolean okToRunBelt;
  private int running;

  public ShootCommand(TripleShooterSubsystem shooter, HoodSubsystem hood, FeederRollerSubsystem feederRoller,
      FeederBeltSubsystem feederBelt, IntakeSubsystem intake, CommandSwerveDrivetrain swerve, boolean bypassAlign) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_shooter = shooter;
    m_hood = hood;
    m_feederRoller = feederRoller;
    m_feederBelt = feederBelt;
    m_intake = intake;
    m_swerve = swerve;
    m_bypassAlign = bypassAlign;
    addRequirements(m_feederBelt);
    addRequirements(m_feederRoller);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    okRunRollers = false;
    okToRunBelt = false;
    running = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    running++;

    m_shooter.runAllVelocityVoltage();

    m_intake.runIntakeAtVelocity(1000);

    DogLog.log("Shoot/Running", running);
    DogLog.log("Shoot/OKRunRollers", okRunRollers);
    DogLog.log("Shoot/OKToRunBelt", okToRunBelt);
    DogLog.log("Shoot/ShootersAtSpeed", m_shooter.allVelocityInTolerance());
    DogLog.log("Shoot/HoodAtTarget", m_hood.isPositionWithinTolerance());
    DogLog.log("Shoot/Aligned", m_swerve.alignedToTarget);

    m_shooter.bypassShootInterlocks = m_bypassAlign;

    if (!okToRunBelt)
      m_feederBelt.runFeederBeltAtVelocity(FeederSetpoints.kBeltReverseRPM);

    if (!okRunRollers)
      m_feederRoller.runFeederRollerAtVelocity(FeederSetpoints.kRollersReverseRPM);

    if ((m_shooter.allVelocityInTolerance() && m_hood.isPositionWithinTolerance()
        && (m_swerve.alignedToTarget || m_shooter.bypassShootInterlocks || RobotBase.isSimulation() && running > 50)))
      okRunRollers = true;

    if (okRunRollers) {
      m_feederRoller.runFeederRollerAtVelocity();

      if (Math.abs(m_feederRoller.feederRollerMotor.getEncoder().getVelocity()) > FeederSetpoints.rollerSpeedToStartBelt
          || RobotBase.isSimulation() && running > 100)
        okToRunBelt = true;

      if (okToRunBelt) {
        m_feederBelt.runFeederBeltAtVelocity();
      }
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_feederBelt.stopFeederBeltMotor();
    m_feederRoller.stopFeederRollerMotor();
    m_intake.stopIntakeMotor();
    okRunRollers = false;
    okToRunBelt = false;
    running = 0;

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
