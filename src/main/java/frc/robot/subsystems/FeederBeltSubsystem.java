// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import dev.doglog.DogLog;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants;
import frc.robot.Constants.FeederSetpoints;

public class FeederBeltSubsystem extends SubsystemBase {
  /** Creates a new FeederSubsystem. */

  public SparkMax feederBeltMotor;

  private SparkClosedLoopController closedLoopController;

  private double feederBeltPowerSim;

  private boolean showData;

  public final Alert feederBeltAlert = new Alert(
      "Feeder Belt Fault",
      AlertType.kError);

  public boolean pulse;

  public double beltStartPulseTime = 2.;
  public double beltPulseTime = .5;

  public double beltStopPulseTime = beltStartPulseTime + beltPulseTime;

  public double beltInitialShootTime = 5.;

  public FeederBeltSubsystem(boolean showData) {
    feederBeltMotor = new SparkMax(Constants.CANIDConstants.feederBeltID, MotorType.kBrushless);
    /*
     * Apply the appropriate configurations to the SPARKs.
     *
     * kResetSafeParameters is used to get the SPARK to a known state. This
     * is useful in case the SPARK is replaced.
     *
     * kPersistParameters is used to ensure the configuration is not lost when
     * the SPARK loses power. This is useful for power cycles that may occur
     * mid-operation.
     */

    feederBeltMotor.configure(
        Configs.Feeder.feederBeltConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    closedLoopController = feederBeltMotor.getClosedLoopController();

    this.showData = showData;

  }

  @Override
  public void periodic() {

    DogLog.log("Feeder/BeltRPM", feederBeltMotor.getEncoder().getVelocity());
    DogLog.log("Feeder/BeltAmps", feederBeltMotor.getOutputCurrent());
    DogLog.log("Feeder/BeltVolts", feederBeltMotor.getAppliedOutput() * RobotController.getBatteryVoltage());

  }

  // feeder belt

  public void runFeederBeltMotor(double power) {
    feederBeltMotor.set(power);
    feederBeltPowerSim = power;
  }

  public void runFeederBeltAtVelocity(double rpm) {
    closedLoopController.setSetpoint(rpm, ControlType.kVelocity, ClosedLoopSlot.kSlot0);
  }

  public void runFeederBeltAtVelocity() {
    closedLoopController.setSetpoint(FeederSetpoints.kBeltShootRPM, ControlType.kVelocity, ClosedLoopSlot.kSlot0);
  }

  public void stopFeederBeltMotor() {
    feederBeltMotor.set(0);
    feederBeltPowerSim = 0;
  }

  public Command startFeederBeltCommand() {
    return Commands.runOnce(() -> runFeederBeltMotor(FeederSetpoints.kFeedBeltSetpoint));
  }

  public Command stopFeederBeltCommand() {
    return Commands.runOnce(() -> stopFeederBeltMotor());
  }

  /**
   * Command to run the feeder belt motor. When the command is
   * interrupted, e.g. the button is released,
   * the motors will stop.
   */
  public Command jogFeederBeltCommand() {
    return this.startEnd(
        () -> {
          this.runFeederBeltMotor(Constants.FeederSetpoints.kFeedBeltJogSetpoint);
        }, () -> {
          this.runFeederBeltMotor(0.0);
        }).withName("JogFeederBelt");
  }

  /**
   * Command to reverse the feeder belt motor. When the command is
   * interrupted, e.g. the button is released, the motors will stop.
   */
  public Command jogReverseFeederBeltCommand() {
    return this.startEnd(
        () -> {
          this.runFeederBeltMotor(-Constants.FeederSetpoints.kFeedBeltSetpoint);
        }, () -> {
          this.runFeederBeltMotor(0.0);
        }).withName("FeederBeltReversing");
  }

  public double getFeederBeltAppliedOutput() {
    if (RobotBase.isReal())
      return feederBeltMotor.getAppliedOutput();
    else
      return feederBeltPowerSim;
  }

  public double getFeederBeltCurrent() {
    return feederBeltMotor.getOutputCurrent();
  }

  public boolean feederBeltRunning() {
    return Math.abs(getFeederBeltAppliedOutput()) > .1;
  }

  public Command clearFeederBeltStickyFaultsCommand() {
    return Commands.runOnce(() -> feederBeltMotor.clearFaults());
  }

}
