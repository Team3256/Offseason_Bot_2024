// Copyright (c) 2024 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.utils;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.drivers.MonitoredTalonFX;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;
import org.littletonrobotics.junction.AutoLogOutput;

public class SinglePositionSubsystem extends SubsystemBase implements Loggable {

  private MonitoredTalonFX positionMotor;
  private TalonFXSimState positionMotorSim;
  private SingleJointedArmSim positionModel;

  final PositionVoltage positionRequest = new PositionVoltage(0).withSlot(0);
  final MotionMagicVoltage motionMagicRequest = new MotionMagicVoltage(0).withSlot(0);

  private boolean isMotionMagic = false;
  private double currentThreshold = 0;
  private double stallVelocity = 0;

  public void DEFAULT() {}

  public SinglePositionSubsystem(
      boolean isMotionMagic, double currentThreshold, double stallVelocity) {
    this.isMotionMagic = isMotionMagic;
    this.currentThreshold = currentThreshold;
    this.stallVelocity = stallVelocity;
  }

  protected void configureRealHardware(
      int motorID,
      NeutralModeValue neutralModeValue,
      double kS,
      double kV,
      double kP,
      double kI,
      double kD,
      double velocity,
      double acceleration,
      double jerk,
      boolean enableStatorLimit,
      double statorLimit) {
    positionMotor = new MonitoredTalonFX(motorID);
    positionMotor.setNeutralMode(neutralModeValue);

    setMotorConfigs(
        kS, kV, kP, kI, kD, velocity, acceleration, jerk, enableStatorLimit, statorLimit);
  }

  // @Config(name = "Pivot Motor Slot 0 Configs")
  protected void setMotorConfigs(
      double kS,
      double kV,
      double kP,
      double kI,
      double kD,
      double velocity,
      double acceleration,
      double jerk,
      boolean enableStatorLimit,
      double statorLimit) {
    var talonFXConfiguration = new TalonFXConfiguration();
    PhoenixUtil.checkErrorAndRetry(
        () -> positionMotor.getConfigurator().refresh(talonFXConfiguration));
    talonFXConfiguration.Slot0.kS = kS;
    talonFXConfiguration.Slot0.kV = kV;
    talonFXConfiguration.Slot0.kP = kP;
    talonFXConfiguration.Slot0.kI = kI;
    talonFXConfiguration.Slot0.kD = kD;
    talonFXConfiguration.MotionMagic.MotionMagicAcceleration = velocity;
    talonFXConfiguration.MotionMagic.MotionMagicCruiseVelocity = acceleration;
    talonFXConfiguration.MotionMagic.MotionMagicJerk = jerk;
    talonFXConfiguration.CurrentLimits.StatorCurrentLimitEnable = enableStatorLimit;
    talonFXConfiguration.CurrentLimits.StatorCurrentLimit = statorLimit;
    TalonUtil.applyAndCheckConfiguration(positionMotor, talonFXConfiguration);
  }

  public void configureSimHardware(
      int kNumPositionMotors,
      double kPositionMotorGearing,
      double jKgMetersSquared,
      double kPositionLength,
      double kPositionMinAngleDeg,
      double kPositionMaxAngleDeg,
      boolean simulateGravity) {
    positionMotorSim = positionMotor.getSimState();
    positionMotorSim.setSupplyVoltage(12);
    positionModel =
        new SingleJointedArmSim(
            DCMotor.getFalcon500(kNumPositionMotors),
            kPositionMotorGearing,
            jKgMetersSquared,
            kPositionLength,
            Units.degreesToRadians(kPositionMinAngleDeg),
            Units.degreesToRadians(kPositionMaxAngleDeg),
            simulateGravity,
            kPositionMinAngleDeg);
  }

  public void off() {
    positionMotor.setControl(new NeutralOut());
    if (!Robot.isReal()) {
      positionModel.setInputVoltage(0);
    }
  }

  public void setOutputVoltage(double voltage) {
    positionMotor.setVoltage(voltage);
  }

  public void setDegrees(double degrees) {
    if (isMotionMagic) {
      positionMotor.setControl(motionMagicRequest.withPosition(degrees));
    } else {
      positionMotor.setControl(positionRequest.withPosition(degrees));
    }
  }

  public void setControlStaticBrake() {
    positionMotor.setControl(new StaticBrake());
  }

  @AutoLogOutput
  public double getVoltage() {
    return positionMotor.getMotorVoltage().getValue();
  }

  @AutoLogOutput
  public double getVelocity() {
    return positionMotor.getRotorVelocity().getValue();
  }

  public boolean getVelocityStalled() {
    return Math.abs(positionMotor.getRotorVelocity().getValue()) < 2;
  }

  @AutoLogOutput
  public double getDegrees() {
    return ((positionMotor.getRotorPosition().getValue()) * 360);
  }

  /*
   * ZERO THE PIVOT ENCODER
   * ONLY CALL THIS WHEN YOU ARE FOR SURE THE PIVOT IS AT ITS ZERO POSITION. USE
   * THE CALIBRATION FUNCTIONALITY TO SET THIS UP!!
   */
  public void zero() {
    positionMotor.setPosition(0);
  }

  // @Log.Graph(name = "PivotIntake Stator Current")
  @AutoLogOutput
  public double getCurrent() {
    return positionMotor.getStatorCurrent().getValue();
  }

  @Log
  @AutoLogOutput
  public boolean isCurrentSpiking() {
    return this.getCurrent() > currentThreshold;
  }

  @Log
  @AutoLogOutput
  public boolean isMotorStalled() {
    return this.isCurrentSpiking()
        && this.positionMotor.getRotorVelocity().getValue() < stallVelocity;
  }

  @Override
  public void periodic() {}

  @Override
  public void simulationPeriodic() {
    // positionModel.setInputVoltage(positionMotorSim.getMotorVoltage());
    // positionModel.update(Robot.defaultPeriodSecs);
    // positionMotorSim.setRotorVelocity(positionModel.getVelocityRadPerSec() / (2 *
    // Math.PI));
    // positionMotorSim.setRawRotorPosition(positionModel.getAngleRads() / (2 *
    // Math.PI));
  }
}
