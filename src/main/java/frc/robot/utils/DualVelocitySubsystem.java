// Copyright (c) 2024 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.utils;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.signals.ControlModeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.drivers.MonitoredTalonFX;
import io.github.oblarg.oblog.Loggable;
import org.littletonrobotics.junction.AutoLogOutput;

public class DualVelocitySubsystem extends SubsystemBase implements Loggable {

  final VelocityVoltage m_requestVelocityVoltageUno =
      new VelocityVoltage(0).withEnableFOC(true).withSlot(0);

  final MotionMagicVelocityVoltage m_requestMotionMagicUno =
      new MotionMagicVelocityVoltage(0).withEnableFOC(true).withSlot(0);

  final VelocityVoltage m_requestVelocityVoltageDos =
      new VelocityVoltage(0).withEnableFOC(true).withSlot(0);

  final MotionMagicVelocityVoltage m_requestMotionMagicDos =
      new MotionMagicVelocityVoltage(0).withEnableFOC(true).withSlot(0);

  // final VelocityDutyCycle m_dutyCycleOut = new
  // VelocityDutyCycle(0).withEnableFOC(true).withSlot(1);

  public MonitoredTalonFX unoMotor;

  public MonitoredTalonFX dosMotor;

  // Initializes a DigitalInput on DIO 0

  private TalonFXSimState unoMotorSim;

  public FlywheelSim flywheelSimModel;

  public boolean isMotionMagicUno = false;

  public boolean isMotionMagicDos = false;

  public double currentThresholdUno = 0;
  public double currentThresholdDos = 0;

  public double velocityThresholdUno = 0;
  public double velocityThresholdDos = 0;

  // no output for error derivative

  public DualVelocitySubsystem(
      boolean isMotionMagicUno,
      boolean isMotionMagicDos,
      double currentThresholdUno,
      double currentThresholdDos,
      double velocityThresholdUno,
      double velocityThresholdDos) {
    this.isMotionMagicUno = isMotionMagicUno;
    this.isMotionMagicDos = isMotionMagicDos;
    this.currentThresholdUno = currentThresholdUno;
    this.currentThresholdDos = currentThresholdDos;
    this.velocityThresholdUno = velocityThresholdUno;
    this.velocityThresholdDos = velocityThresholdDos;
  }

  protected void configureRealHardware(
      int motorIDUno,
      int motorIDDos,
      NeutralModeValue neutralModeUno,
      NeutralModeValue neutralModeDos,
      double kSUno,
      double kVUno,
      double kPUno,
      double kIUno,
      double kDUno,
      double kSDos,
      double kVDos,
      double kPDos,
      double kIDos,
      double kDDos,
      InvertedValue invertedUno,
      InvertedValue invertedDos) {
    unoMotor = new MonitoredTalonFX(motorIDUno, "rio"); // don't want piece to fall out

    dosMotor = new MonitoredTalonFX(motorIDDos, "rio");

    setMotorConfigsUno(kSUno, kVUno, kPUno, kIUno, kDUno, invertedUno, neutralModeUno);
    setMotorConfigsDos(kSDos, kVDos, kPDos, kIDos, kDDos, invertedDos, neutralModeDos);
  }

  // @Config(name = "PID values")
  protected void setMotorConfigsUno(
      double kS,
      double kV,
      double kP,
      double kI,
      double kD,
      InvertedValue inverted,
      NeutralModeValue neutral) {
    var motorConfig = new TalonFXConfiguration();
    PhoenixUtil.checkErrorAndRetry(() -> unoMotor.getConfigurator().refresh(motorConfig));
    motorConfig.Slot0.kS = kS; // Add 0.05 V output to overcome static friction
    motorConfig.Slot0.kV = kV; // A velocity target of 1 rps results in 0.12 V output
    motorConfig.Slot0.kP = kP; // An error of 1 rps results in 0.11 V output
    motorConfig.Slot0.kI = kI; // no output for integrated error
    motorConfig.Slot0.kD = kD;
    motorConfig.MotorOutput.Inverted = inverted;
    motorConfig.MotorOutput.NeutralMode = neutral;
    motorConfig.CurrentLimits.StatorCurrentLimit = 60;

    TalonUtil.applyAndCheckConfiguration(unoMotor, motorConfig);
  }

  protected void setMotorConfigsDos(
      double kS,
      double kV,
      double kP,
      double kI,
      double kD,
      InvertedValue inverted,
      NeutralModeValue neutral) {
    var motorConfig = new TalonFXConfiguration();
    PhoenixUtil.checkErrorAndRetry(() -> unoMotor.getConfigurator().refresh(motorConfig));
    motorConfig.Slot0.kS = kS; // Add 0.05 V output to overcome static friction
    motorConfig.Slot0.kV = kV; // A velocity target of 1 rps results in 0.12 V output
    motorConfig.Slot0.kP = kP; // An error of 1 rps results in 0.11 V output
    motorConfig.Slot0.kI = kI; // no output for integrated error
    motorConfig.Slot0.kD = kD;
    motorConfig.MotorOutput.Inverted = inverted;
    motorConfig.MotorOutput.NeutralMode = neutral;
    motorConfig.CurrentLimits.StatorCurrentLimit = 60;

    TalonUtil.applyAndCheckConfiguration(unoMotor, motorConfig);
  }

  /*
   * Motion Magic allows motion profile control of the motor. It can be set
   * dynamically during runtime as
   * well as the beginning at robot initialization.
   * These configs will not be used if UnoConstants.kUseUnoMotionMagic is
   * false
   */
  public void setMotionMagicConfigsUno(double cruiseVelocity, double acceleration, double jerk) {
    var motionMagicConfigs = new MotionMagicConfigs();
    motionMagicConfigs.MotionMagicAcceleration = acceleration;
    motionMagicConfigs.MotionMagicCruiseVelocity = cruiseVelocity;
    motionMagicConfigs.MotionMagicJerk = jerk;
    unoMotor.getConfigurator().apply(motionMagicConfigs);
  }

  public void setMotionMagicConfigsDos(double cruiseVelocity, double acceleration, double jerk) {
    var motionMagicConfigs = new MotionMagicConfigs();
    motionMagicConfigs.MotionMagicAcceleration = acceleration;
    motionMagicConfigs.MotionMagicCruiseVelocity = cruiseVelocity;
    motionMagicConfigs.MotionMagicJerk = jerk;
    dosMotor.getConfigurator().apply(motionMagicConfigs);
  }

  protected void configureSimHardware() {
    unoMotorSim = unoMotor.getSimState();
    flywheelSimModel = new FlywheelSim(DCMotor.getFalcon500(1), 1, 0.1207277 * 100);
  }

  // @Config(name = "Uno Set Voltage", defaultValueNumeric = 0)
  public void setOutputVoltageUno(double voltage) {
    unoMotor.setVoltage(voltage);
  }

  public void setOutputVoltageDos(double voltage) {
    dosMotor.setVoltage(voltage);
  }

  public void setUnoVelocity(double velocity) {
    if (isMotionMagicUno) {
      unoMotor.setControl(m_requestMotionMagicUno.withVelocity(velocity));
    } else {
      unoMotor.setControl(m_requestVelocityVoltageUno.withVelocity(velocity));
    }
  }

  public void setDosVelocity(double velocity) {
    if (isMotionMagicDos) {
      dosMotor.setControl(m_requestMotionMagicDos.withVelocity(velocity));
    } else {
      dosMotor.setControl(m_requestVelocityVoltageDos.withVelocity(velocity));
    }
  }

  // public void setDutyCycle(double velocity) {
  // unoMotor.setControl(
  // m_dutyCycleOut.withVelocity(velocity).withAcceleration(10).withFeedForward(10));
  // if (kDebugEnabled) {
  // System.out.println("uno vel set to: " + velocity);
  // }
  // }

  @AutoLogOutput
  public boolean getOff() {
    return unoMotor.getControlMode().getValue() == ControlModeValue.NeutralOut
        && dosMotor.getControlMode().getValue() == ControlModeValue.NeutralOut;
  }

  public boolean isUnoMotorCurrentSpiking() {
    return unoMotor.getSupplyCurrent().getValue() > currentThresholdUno;
  }

  public boolean isDosMotorCurrentSpiking() {
    return dosMotor.getSupplyCurrent().getValue() > currentThresholdDos;
  }

  public double getUnoVelocity() {
    return unoMotor.getVelocity().getValue();
  }

  public double getDosVelocity() {
    return dosMotor.getVelocity().getValue();
  }

  public boolean isUnoMotorVelocitySpiking() {
    return unoMotor.getVelocity().getValue() < velocityThresholdUno;
  }

  public boolean isDosMotorVelocitySpiking() {
    return dosMotor.getVelocity().getValue() < velocityThresholdDos;
  }

  public boolean isMotorSpinning() {
    return unoMotor.getVelocity().getValue() > 10;
  }

  // @Log.Graph(name = "Uno Motor Rps")
  public double getUnoRps() {
    return unoMotor.getVelocity().getValue();
  }

  public double getUnoMotorVoltage() {
    return unoMotor.getMotorVoltage().getValueAsDouble();
  }

  public double getUnoCurrent() {
    return unoMotor.getSupplyCurrent().getValueAsDouble();
  }

  public double getDosRps() {
    return dosMotor.getVelocity().getValue();
  }

  public double getDosMotorVoltage() {
    return dosMotor.getMotorVoltage().getValueAsDouble();
  }

  public double getDosCurrent() {
    return dosMotor.getSupplyCurrent().getValueAsDouble();
  }

  public void off() {
    unoMotor.setControl(new NeutralOut());
    dosMotor.setControl(new NeutralOut());
    if (!Robot.isReal()) {
      flywheelSimModel.setState(0); // mimic motor braking
    }
  }

  @Override
  public void periodic() {
    // System.out.println("TOF DISTANCE: " + getTOFDistance());
  }

  @Override
  public void simulationPeriodic() {
    flywheelSimModel.setInputVoltage(unoMotorSim.getMotorVoltage());
    flywheelSimModel.update(Robot.defaultPeriodSecs);
    unoMotorSim.setRotorVelocity(flywheelSimModel.getAngularVelocityRPM());
  }
}
