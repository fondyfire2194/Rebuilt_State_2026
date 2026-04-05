// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import edu.wpi.first.wpilibj.event.BooleanEvent;
import edu.wpi.first.wpilibj.event.EventLoop;
import frc.robot.Constants.HoodSetpoints;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.FeederBeltSubsystem;
import frc.robot.subsystems.FeederRollerSubsystem;
import frc.robot.subsystems.HoodSubsystem;
import frc.robot.subsystems.Intake4BarArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.TripleShooterSubsystem;

/** Add your docs here. */

public class LoopEvents {
    CommandSwerveDrivetrain m_drivetrain;
    TripleShooterSubsystem m_shooter;
    FeederBeltSubsystem m_feederBelt;
    FeederRollerSubsystem m_feederRoller;
    HoodSubsystem m_hood;
    IntakeSubsystem m_intake;
    Intake4BarArmSubsystem m_intakeArm;
    EventLoop m_loop;

    public LoopEvents(CommandSwerveDrivetrain drivetrain, TripleShooterSubsystem shooter,
            FeederBeltSubsystem feederBelt, FeederRollerSubsystem feederRoller, HoodSubsystem hood,
            IntakeSubsystem intake, Intake4BarArmSubsystem intakeArm, EventLoop loop) {
        m_drivetrain = drivetrain;
        m_shooter = shooter;
        m_feederBelt = feederBelt;
        m_feederRoller = feederRoller;
        m_hood = hood;
        m_intake = intake;
        m_intakeArm = intakeArm;
        m_loop = loop;
    }

    public void init() {
        BooleanEvent shooterFaultAlert = new BooleanEvent(
                m_loop, () -> m_shooter.leftMotor.getFaultField().asSupplier().get() != 0
                        || m_shooter.middleMotor.getFaultField().asSupplier().get() != 0
                        || m_shooter.rightMotor.getFaultField().asSupplier().get() != 0);
        shooterFaultAlert.ifHigh(() -> m_shooter.shooterAlert.set(true));
        shooterFaultAlert.negate().ifHigh(() -> m_shooter.shooterAlert.set(false));

        BooleanEvent feederBeltAlert = new BooleanEvent(m_loop, () -> m_feederBelt.feederBeltMotor.hasActiveFault());
        feederBeltAlert.ifHigh(() -> m_feederBelt.feederBeltAlert.set(true));
        feederBeltAlert.negate().ifHigh(() -> m_feederBelt.feederBeltAlert.set(false));

        BooleanEvent feederRollerAlert = new BooleanEvent(m_loop,
                () -> m_feederRoller.feederRollerMotor.hasActiveFault());
        feederRollerAlert.ifHigh(() -> m_feederRoller.feederRollerAlert.set(true));
        feederRollerAlert.negate().ifHigh(() -> m_feederRoller.feederRollerAlert.set(false));

        BooleanEvent hoodAlert = new BooleanEvent(m_loop,
                () -> m_hood.hoodMotor.hasActiveFault());
        hoodAlert.ifHigh(() -> m_hood.hoodAlert.set(true));
        hoodAlert.negate().ifHigh(() -> m_hood.hoodAlert.set(false));

        BooleanEvent intakeAlert = new BooleanEvent(m_loop,
                () -> m_intake.intakeMotor.hasActiveFault());
        intakeAlert.ifHigh(() -> m_intake.intakeAlert.set(true));
        intakeAlert.negate().ifHigh(() -> m_intake.intakeAlert.set(false));

        BooleanEvent intakeArmAlert = new BooleanEvent(m_loop,
                () -> m_intakeArm.intakeArmMotor.hasActiveFault()
                        || m_intakeArm.intakeArmMotorFollower.hasActiveFault());
        intakeArmAlert.ifHigh(() -> m_intakeArm.intakeArmAlert.set(true));
        intakeArmAlert.negate().ifHigh(() -> m_intakeArm.intakeArmAlert.set(false));

    }

}
