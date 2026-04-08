// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import org.jspecify.annotations.Nullable;

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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants;
import frc.robot.Constants.FeederSetpoints;

public class FeederBeltSubsystem extends SubsystemBase {
  /**
   * Creates a new FeederSubsystem.
   * Fuel travels from belt to belt roller to roller rollers to shooter roller
   * 
   * Feeder belt has a 3:1 reduction from a Neo motor and has 2 inch dia rollers
   * Feeder roller is 1:1 from a Neo motor and has 2 in dia rollers
   * Shooter is 1:1 from Kraken X60 and has a 4" diameter roller
   * 
   * Shooter speed typical is around 3000 rpm - 50 revs per second so 50 * PI * 4
   * or 200 PI inches per second
   * For rollers to provide 50% of that, 50 * PI * 2, rollers need to run same
   * speed as shooter or 3000 rpm
   * For belt to provide 50% of rollers, need 25 * PI * 2 or 1500 rpm but with 3:1
   * reduction means 4500 motor rpm
   * Belt 3600 rpm gives belt roller 1200 rpm so 20 * PI * 2 or 40 * PI inches per
   * second or 20% shooter speed
   * Belt moves 4 inches per roller rev = 20 * 4 or 80 inches per second
   * 
   * Speeds = 80(belt) to 120 to 314 to 628 inches per second
   * 
   * 
   */

  public SparkMax feederBeltMotor;

  private SparkClosedLoopController closedLoopController;

  private double feederBeltPowerSim;

  private boolean showData;

  public final Alert feederBeltAlert = new Alert(
      "Feeder Belt Fault",
      AlertType.kError);

  private double targetRPM;

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

    feederBeltMotor.clearFaults();

    this.showData = showData;

    feederBeltMotor.getEncoder().setPosition(0);

   // setDefaultCommand(stopFeederBeltCommand());
  }

  @Override
  public void periodic() {
    if (getCurrentCommand() != null)
      DogLog.log("Belt/CurrentCommand", getCurrentCommand().getName());

    DogLog.log("Belt/TargetRPM", targetRPM);

    DogLog.log("Belt/RPM", feederBeltMotor.getEncoder().getVelocity());
    DogLog.log("Belt/Amps", feederBeltMotor.getOutputCurrent());
    DogLog.log("Belt/Volts", feederBeltMotor.getAppliedOutput() * RobotController.getBatteryVoltage());
  }

  public void runFeederBeltMotor(double power) {
    targetRPM = power * 570;
    feederBeltMotor.set(power);
    feederBeltPowerSim = power;
  }

  public Command runFeederBeltAtVelocityCommand(double rpm) {
    return run(() -> runFeederBeltAtVelocity(rpm));
  }

  public void runFeederBeltAtVelocity(double rpm) {
    targetRPM = rpm;
    closedLoopController.setSetpoint(rpm, ControlType.kVelocity, ClosedLoopSlot.kSlot0);
  }

  public void runFeederBeltAtVelocity() {
    targetRPM = FeederSetpoints.kBeltShootRPM;
    closedLoopController.setSetpoint(FeederSetpoints.kBeltShootRPM, ControlType.kVelocity, ClosedLoopSlot.kSlot0);
  }

  public Command unstickFuelCommand(double rate1, double rate2, double moveTime, double dwell) {
    return Commands.sequence(
        jogFeederBeltCommand(() -> rate1).withTimeout(moveTime),
        Commands.waitSeconds(dwell),
        jogFeederBeltCommand(() -> rate2).withTimeout(moveTime),
        Commands.waitSeconds(dwell))
        .repeatedly()
        .andThen(jogFeederBeltCommand(() -> 0).withTimeout(.1));
  }

  public double getBeltPosition() {
    return feederBeltMotor.getEncoder().getPosition();
  }

  public void stopFeederBeltMotor() {
    targetRPM = 0;
    feederBeltMotor.set(0);
    feederBeltPowerSim = 0;
  }

  public Command startFeederBeltCommand() {
    return Commands.runOnce(() -> runFeederBeltMotor(FeederSetpoints.kFeedBeltSetpoint));
  }

  public Command stopFeederBeltCommand() {
    return runOnce(() -> stopFeederBeltMotor()).withName("StopBelt");
  }

  /**
   * Command to run the feeder belt motor. When the command is
   * interrupted, e.g. the button is released,
   * the motors will stop.
   */
  public Command jogFeederBeltCommand(DoubleSupplier rate) {
    return this.startEnd(
        () -> {
          this.runFeederBeltMotor(rate.getAsDouble());
        }, () -> {
          this.runFeederBeltMotor(0.0);
        }).withName("JogFeederBelt");
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
