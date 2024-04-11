// Copyright (c) 2024 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.ampbar;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.FeatureFlags;
import frc.robot.Robot;
import frc.robot.drivers.MonitoredTalonFX;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Config;
import io.github.oblarg.oblog.annotations.Log;
import org.littletonrobotics.junction.AutoLogOutput;

public class AmpBar extends SubsystemBase implements Loggable {

  private MonitoredTalonFX ampBarMotor;
  private TalonFXSimState ampBarMotorSim;
  private SingleJointedArmSim ampBarModel;

  public void DEFAULT() {}

  public AmpBar() {
    if (RobotBase.isReal()) {
      configureRealHardware();
    } else {
      configureSimHardware();
    }
  }

  private void configureRealHardware() {
    ampBarMotor = new MonitoredTalonFX(AmpBarConstants.kAmpBarMotorID);
    ampBarMotor.setNeutralMode(NeutralModeValue.Brake);
    setStatorLimits(true, 20);
  }

  private void setCurrentLimits(
      boolean enableCurrentLimit,
      double currentLimit,
      double currentThreshold,
      double currentThresholdTime) {
    var ampBarMotorConfigs = new TalonFXConfiguration();
    ampBarMotorConfigs.CurrentLimits.SupplyCurrentLimitEnable = enableCurrentLimit;
    ampBarMotorConfigs.CurrentLimits.SupplyCurrentLimit = currentLimit;
    ampBarMotorConfigs.CurrentLimits.SupplyCurrentThreshold = currentThreshold;
    ampBarMotorConfigs.CurrentLimits.SupplyTimeThreshold = currentThresholdTime;
    ampBarMotor.getConfigurator().apply(ampBarMotorConfigs);
  }

  private void setStatorLimits(boolean enableStatorLimit, double statorLimit) {
    var ampBarMotorConfigs = new TalonFXConfiguration();
    ampBarMotorConfigs.CurrentLimits.StatorCurrentLimitEnable = enableStatorLimit;
    ampBarMotorConfigs.CurrentLimits.StatorCurrentLimit = statorLimit;
    ampBarMotor.getConfigurator().apply(ampBarMotorConfigs);
  }

  private void configureSimHardware() {
    configureRealHardware();
    ampBarMotorSim = ampBarMotor.getSimState();
    ampBarMotorSim.setSupplyVoltage(12);
    ampBarModel =
        new SingleJointedArmSim(
            DCMotor.getFalcon500(AmpBarConstants.kNumAmpBarMotors),
            AmpBarConstants.kAmpBarMotorGearing,
            AmpBarConstants.jKgMetersSquared,
            AmpBarConstants.kAmpBarLength,
            Units.degreesToRadians(AmpBarConstants.kAmpBarMinAngleDeg),
            Units.degreesToRadians(AmpBarConstants.kAmpBarMaxAngleDeg),
            false,
            AmpBarConstants.kAmpBarMinAngleDeg);
  }

  public void off() {
    ampBarMotor.setControl(new NeutralOut());
    if (!Robot.isReal()) {
      ampBarModel.setInputVoltage(0);
    }
    if (Constants.FeatureFlags.kDebugEnabled) {
      System.out.println("Amp Bar off");
    }
  }

  @Config(name = "Set Amp Bar Voltage")
  public void setOutputVoltage(double voltage) {
    ampBarMotor.setVoltage(voltage);
    if (Constants.FeatureFlags.kDebugEnabled) {
      System.out.println("Amp Bar voltage set to: " + voltage);
    }
  }

  @Log.Graph(name = "AmpBar Voltage")
  @AutoLogOutput
  public double getVoltage() {
    return ampBarMotor.getMotorVoltage().getValue();
  }

  @Log.Graph(name = "AmpBar Velocity")
  @AutoLogOutput
  public double getVelocity() {
    return ampBarMotor.getVelocity().getValue();
  }

  @Log.Graph(name = "AmpBar Position")
  @AutoLogOutput
  public double getDegrees() {
    return ((ampBarMotor.getRotorPosition().getValue()) * 360);
  }

  /*
   * ZERO THE PIVOT ENCODER
   * ONLY CALL THIS WHEN YOU ARE FOR SURE THE PIVOT IS AT ITS ZERO POSITION. USE
   * THE CALIBRATION FUNCTIONALITY TO SET THIS UP!!
   */
  public void zero() {
    if (FeatureFlags.kDebugEnabled) {
      System.out.println("[AmpBar] Setting zero position to: " + getDegrees());
    }
    ampBarMotor.setPosition(0);
    if (FeatureFlags.kDebugEnabled) {
      System.out.println("[AmpBar] Zero position set. Current position: " + getDegrees());
      // Should be 0.
      // Sometimes, it's not
      // due to CAN latency.
      // It's fine, since the
      // time it takes for any
      // other command to run
      // is enough for the CAN
      // latency to be
      // resolved.
    }
  }

  // @Log.Graph(name = "AmpBar Stator Current")
  @AutoLogOutput
  public double getCurrent() {
    return ampBarMotor.getSupplyCurrent().getValue();
  }

  @Log
  @AutoLogOutput
  public boolean isCurrentSpiking() {
    return this.getCurrent() > AmpBarConstants.kAmpBarCurrentThreshold;
  }

  @AutoLogOutput
  public boolean isVelocitySpiking() {
    return Math.abs(this.getVelocity()) < AmpBarConstants.kAmpBarVelocityThreshold;
  }

  @Log
  @AutoLogOutput
  public boolean isMotorStalled() {
    return this.isCurrentSpiking()
        && this.ampBarMotor.getRotorVelocity().getValue() < AmpBarConstants.kStallVelocityThreshold;
  }

  @Override
  public void periodic() {}

  @Override
  public void simulationPeriodic() {
    ampBarModel.setInputVoltage(ampBarMotorSim.getMotorVoltage());
    ampBarModel.update(Robot.defaultPeriodSecs);
    ampBarMotorSim.setRotorVelocity(ampBarModel.getVelocityRadPerSec() / (2 * Math.PI));
    ampBarMotorSim.setRawRotorPosition(ampBarModel.getAngleRads() / (2 * Math.PI));
  }
}
