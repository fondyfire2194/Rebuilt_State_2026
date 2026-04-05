// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
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

public class FeederRollerSubsystem extends SubsystemBase {
  /** Creates a new FeederSubsystem. */

  public SparkMax feederRollerMotor;

  private SparkClosedLoopController closedLoopController;
  private RelativeEncoder encoder;

  private double feederRollerPowerSim;

  private boolean showData;

  public final Alert feederRollerAlert = new Alert(
      "Feeder Fault",
      AlertType.kError);

  public boolean pulse;
  public FeederRollerSubsystem(boolean showData) {
    feederRollerMotor = new SparkMax(Constants.CANIDConstants.feederRollerID, MotorType.kBrushless);
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
    feederRollerMotor.configure(
        Configs.Feeder.feederRollerConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    closedLoopController = feederRollerMotor.getClosedLoopController();


    this.showData = showData;
  }

  @Override
  public void periodic() {

    // This method will be called once per scheduler run
    DogLog.log("Feeder/RollerRPM", feederRollerMotor.getEncoder().getVelocity());
    DogLog.log("Feeder/RollerTargetRPM", closedLoopController.getSetpoint());
    DogLog.log("Feeder/RollerAmps", feederRollerMotor.getOutputCurrent());
    DogLog.log("Feeder/RollerVolts", feederRollerMotor.getAppliedOutput() * RobotController.getBatteryVoltage());

   
  }

  public void runFeederRollerMotor(double power) {
    feederRollerMotor.set(power);
    feederRollerPowerSim = power;
  }

  public void runFeederRollerAtVelocity() {
    closedLoopController.setSetpoint(FeederSetpoints.kRollerShootRPM, ControlType.kVelocity, ClosedLoopSlot.kSlot0);
  }

  public void stopFeederRollerMotor() {
    closedLoopController.setSetpoint(0, ControlType.kVelocity, ClosedLoopSlot.kSlot0);
    feederRollerMotor.set(0);
    feederRollerPowerSim = 0;
  }

  public Command startFeederRollerCommand() {
    return Commands.runOnce(() -> runFeederRollerMotor(FeederSetpoints.kFeedRollerSetpoint));
  }

  public Command stopFeederRollerCommand() {
    return Commands.runOnce(() -> stopFeederRollerMotor());
  }

  /**
   * Command to run the feeder roller motor. When the command is
   * interrupted, e.g. the button is released,
   * the motors will stop.
   */
  public Command jogFeederRollerCommand() {
    return this.startEnd(
        () -> {
          this.runFeederRollerMotor(Constants.FeederSetpoints.kFeedRollerJogSetpoint);
        }, () -> {
          this.runFeederRollerMotor(0.0);
        }).withName("Jog FeederRoller");
  }

  /**
   * Command to reverse the feeder roller motor. When the command is
   * interrupted, e.g. the button is
   * released, the motors will stop.
   */
  public Command jogReverseFeederRollerCommand() {
    return this.startEnd(
        () -> {
          this.runFeederRollerMotor(-Constants.FeederSetpoints.kFeedRollerJogSetpoint);
        }, () -> {
          this.runFeederRollerMotor(0.0);
        }).withName("FeederRollerReversing");
  }

  public double getFeederRollerAppliedOutput() {
    if (RobotBase.isReal())
      return feederRollerMotor.getAppliedOutput();
    else
      return feederRollerPowerSim;
  }

  public double getFeederRollerCurrent() {
    return feederRollerMotor.getOutputCurrent();
  }

  public boolean feederRollerRunning() {
    return Math.abs(getFeederRollerAppliedOutput()) > .1;
  }

   public Command clearFeederRollerStickyFaultsCommand() {
    return Commands.runOnce(() -> feederRollerMotor.clearFaults());
  }


  
}
