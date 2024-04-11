// Copyright (c) 2024 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.intake;

import static frc.robot.Constants.FeatureFlags.kDebugEnabled;
import static frc.robot.subsystems.intake.IntakeConstants.*;

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
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.drivers.MonitoredTalonFX;
import frc.robot.utils.PhoenixUtil;
import frc.robot.utils.TalonUtil;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Config;
import io.github.oblarg.oblog.annotations.Log;
import org.littletonrobotics.junction.AutoLogOutput;

public class Intake extends SubsystemBase implements Loggable {

  final VelocityVoltage m_requestVelocityVoltageIntake =
      new VelocityVoltage(0).withEnableFOC(true).withSlot(0);

  final MotionMagicVelocityVoltage m_requestMotionMagicIntake =
      new MotionMagicVelocityVoltage(0).withEnableFOC(true).withSlot(0);

  final VelocityVoltage m_requestVelocityVoltagePassthrough =
      new VelocityVoltage(0).withEnableFOC(true).withSlot(0);

  final MotionMagicVelocityVoltage m_requestMotionMagicPassthrough =
      new MotionMagicVelocityVoltage(0).withEnableFOC(true).withSlot(0);

  // final VelocityDutyCycle m_dutyCycleOut = new
  // VelocityDutyCycle(0).withEnableFOC(true).withSlot(1);

  @Log.MotorController(name = "Intake Motor")
  public MonitoredTalonFX intakeMotor;

  @Log.MotorController(name = "Passthrough Motor")
  public MonitoredTalonFX passthroughMotor;

  // Initializes a DigitalInput on DIO 0

  private TalonFXSimState intakeMotorSim;
  private DigitalInput beamBreakInput;

  public FlywheelSim flywheelSimModel;
  @Log private Command currentCommand = getCurrentCommand();

  // no output for error derivative

  public Intake() {
    if (RobotBase.isReal()) {
      configureRealHardware();
    } else {
      configureSimHardware();
    }

    System.out.println("Intake initialized");
  }

  private void configureRealHardware() {
    intakeMotor =
        new MonitoredTalonFX(IntakeConstants.kIntakeMotorID, "rio"); // don't want piece to fall out

    passthroughMotor = new MonitoredTalonFX(IntakeConstants.kPassthroughMotorID, "rio");
    passthroughMotor.setInverted(false);
    passthroughMotor.setNeutralMode(NeutralModeValue.Brake);

    beamBreakInput = new DigitalInput(IntakeConstants.kIntakeBeamBreakDIO);
    setMotorConfigsIntake(
        kIntakeKS,
        kIntakeKV,
        kIntakeKP,
        kIntakeKI,
        kIntakeKD,
        InvertedValue.CounterClockwise_Positive,
        NeutralModeValue.Coast);
    setMotorConfigsPassthrough(
        kPassthroughkS,
        kPassthroughkV,
        kPassthroughkP,
        kPassthroughkI,
        kPassthroughkD,
        InvertedValue.Clockwise_Positive,
        NeutralModeValue.Brake);
  }

  // @Config(name = "PID values")
  private void setMotorConfigsIntake(
      double kS,
      double kV,
      double kP,
      double kI,
      double kD,
      InvertedValue inverted,
      NeutralModeValue neutral) {
    var motorConfig = new TalonFXConfiguration();
    PhoenixUtil.checkErrorAndRetry(() -> intakeMotor.getConfigurator().refresh(motorConfig));
    motorConfig.Slot0.kS = kS; // Add 0.05 V output to overcome static friction
    motorConfig.Slot0.kV = kV; // A velocity target of 1 rps results in 0.12 V output
    motorConfig.Slot0.kP = kP; // An error of 1 rps results in 0.11 V output
    motorConfig.Slot0.kI = kI; // no output for integrated error
    motorConfig.Slot0.kD = kD;
    motorConfig.MotorOutput.Inverted = inverted;
    motorConfig.MotorOutput.NeutralMode = neutral;
    motorConfig.CurrentLimits.StatorCurrentLimit = 60;

    TalonUtil.applyAndCheckConfiguration(intakeMotor, motorConfig);
  }

  private void setMotorConfigsPassthrough(
      double kS,
      double kV,
      double kP,
      double kI,
      double kD,
      InvertedValue inverted,
      NeutralModeValue neutral) {
    var motorConfig = new TalonFXConfiguration();
    PhoenixUtil.checkErrorAndRetry(() -> intakeMotor.getConfigurator().refresh(motorConfig));
    motorConfig.Slot0.kS = kS; // Add 0.05 V output to overcome static friction
    motorConfig.Slot0.kV = kV; // A velocity target of 1 rps results in 0.12 V output
    motorConfig.Slot0.kP = kP; // An error of 1 rps results in 0.11 V output
    motorConfig.Slot0.kI = kI; // no output for integrated error
    motorConfig.Slot0.kD = kD;
    motorConfig.MotorOutput.Inverted = inverted;
    motorConfig.MotorOutput.NeutralMode = neutral;
    motorConfig.CurrentLimits.StatorCurrentLimit = 60;

    TalonUtil.applyAndCheckConfiguration(intakeMotor, motorConfig);
  }

  /*
   * Motion Magic allows motion profile control of the motor. It can be set
   * dynamically during runtime as
   * well as the beginning at robot initialization.
   * These configs will not be used if IntakeConstants.kUseIntakeMotionMagic is
   * false
   */
  public void setMotionMagicConfigsIntake(double cruiseVelocity, double acceleration, double jerk) {
    var motionMagicConfigs = new MotionMagicConfigs();
    motionMagicConfigs.MotionMagicAcceleration = acceleration;
    motionMagicConfigs.MotionMagicCruiseVelocity = cruiseVelocity;
    motionMagicConfigs.MotionMagicJerk = jerk;
    intakeMotor.getConfigurator().apply(motionMagicConfigs);
  }

  public void setMotionMagicConfigsPassthrough(
      double cruiseVelocity, double acceleration, double jerk) {
    var motionMagicConfigs = new MotionMagicConfigs();
    motionMagicConfigs.MotionMagicAcceleration = acceleration;
    motionMagicConfigs.MotionMagicCruiseVelocity = cruiseVelocity;
    motionMagicConfigs.MotionMagicJerk = jerk;
    passthroughMotor.getConfigurator().apply(motionMagicConfigs);
  }

  private void configureSimHardware() {
    configureRealHardware();
    intakeMotorSim = intakeMotor.getSimState();
    flywheelSimModel = new FlywheelSim(DCMotor.getFalcon500(1), 1, 0.1207277 * 100);
  }

  // @Config(name = "Intake Set Voltage", defaultValueNumeric = 0)
  public void setOutputVoltageIntake(double voltage) {
    intakeMotor.setVoltage(voltage);
    if (kDebugEnabled) {
      // System.out.println("Intake voltage set to: " + voltage);
    }
  }

  public void setOutputVoltagePassthrough(double voltage) {
    passthroughMotor.setVoltage(voltage);
    if (kDebugEnabled) {
      // System.out.println("Intake voltage set to: " + voltage);
    }
  }

  @Config(name = "Intake Set Velocity", defaultValueNumeric = 0)
  public void setIntakeVelocity(double velocity) {
    if (IntakeConstants.kUseIntakeMotionMagic) {
      intakeMotor.setControl(m_requestMotionMagicIntake.withVelocity(velocity));
    } else {
      intakeMotor.setControl(m_requestVelocityVoltageIntake.withVelocity(velocity));
    }
    if (kDebugEnabled) {
      System.out.println("Intake velocity set to: " + velocity);
    }
  }

  @Config(name = "Passthrough Set Velocity", defaultValueNumeric = 0)
  public void setPassthroughVelocity(double velocity) {
    if (IntakeConstants.kUsePassthroughMotionMagic) {
      passthroughMotor.setControl(m_requestMotionMagicPassthrough.withVelocity(velocity));
    } else {
      passthroughMotor.setControl(m_requestVelocityVoltagePassthrough.withVelocity(velocity));
    }
    if (kDebugEnabled) {
      System.out.println("Intake velocity set to: " + velocity);
    }
  }

  // public void setDutyCycle(double velocity) {
  // intakeMotor.setControl(
  // m_dutyCycleOut.withVelocity(velocity).withAcceleration(10).withFeedForward(10));
  // if (kDebugEnabled) {
  // System.out.println("intake vel set to: " + velocity);
  // }
  // }

  @Log.BooleanBox(name = "Intake Off")
  @AutoLogOutput
  public boolean getOff() {
    return intakeMotor.getControlMode().getValue() == ControlModeValue.NeutralOut
        && passthroughMotor.getControlMode().getValue() == ControlModeValue.NeutralOut;
  }

  @AutoLogOutput
  @Log.BooleanBox(name = "Intake Motor Current Spiking")
  public boolean isIntakeMotorCurrentSpiking() {
    return intakeMotor.getSupplyCurrent().getValue() > IntakeConstants.kIntakeCurrentThreshold;
  }

  @AutoLogOutput
  public boolean isPassthroughMotorCurrentSpiking() {
    return passthroughMotor.getSupplyCurrent().getValue() > IntakeConstants.kIntakeCurrentThreshold;
  }

  @AutoLogOutput(key = "intake velocity")
  public double getIntakeVelocity() {
    return intakeMotor.getVelocity().getValue();
  }

  @AutoLogOutput(key = "passthrough velocity")
  public double getPassthroughVelocity() {
    return passthroughMotor.getVelocity().getValue();
  }

  @AutoLogOutput(key = "intake velocity spiking")
  public boolean isIntakeMotorVelocitySpiking() {
    return intakeMotor.getVelocity().getValue() < kIntakeVelocitySpiking;
  }

  @AutoLogOutput(key = "passthrough velocity spiking")
  public boolean isPassthroughMotorVelocitySpiking() {
    return passthroughMotor.getVelocity().getValue() < kPassthroughVelocitySpiking;
  }

  @AutoLogOutput(key = "intake spinny")
  public boolean isMotorSpinning() {
    return intakeMotor.getVelocity().getValue() > 10;
  }

  @AutoLogOutput
  // @Log.Graph(name = "Intake Motor Rps")
  public double getIntakeRps() {
    return intakeMotor.getVelocity().getValue();
  }

  @AutoLogOutput
  @Log.Graph(name = "Intake Motor Voltage")
  public double getIntakeMotorVoltage() {
    return intakeMotor.getMotorVoltage().getValueAsDouble();
  }

  @AutoLogOutput
  @Log.Graph(name = "Intake Current Voltage")
  public double getIntakeCurrent() {
    return intakeMotor.getSupplyCurrent().getValueAsDouble();
  }

  @AutoLogOutput
  // @Log.Graph(name = "Intake Motor Rps")
  public double getPassthroughRps() {
    return passthroughMotor.getVelocity().getValue();
  }

  @AutoLogOutput
  @Log.Graph(name = "Intake Motor Voltage")
  public double getPassthroughMotorVoltage() {
    return passthroughMotor.getMotorVoltage().getValueAsDouble();
  }

  @AutoLogOutput
  @Log.Graph(name = "Intake Current Voltage")
  public double getPassthroughCurrent() {
    return passthroughMotor.getSupplyCurrent().getValueAsDouble();
  }

  public void off() {
    intakeMotor.setControl(new NeutralOut());
    passthroughMotor.setControl(new NeutralOut());
    if (!Robot.isReal()) {
      flywheelSimModel.setState(0); // mimic motor braking
    }
    if (kDebugEnabled) {
      System.out.println("Intake off");
    }
  }

  @AutoLogOutput
  public boolean isBeamBroken() {
    return !beamBreakInput.get();
  }

  @Override
  public void periodic() {
    currentCommand = getCurrentCommand();
    // System.out.println("TOF DISTANCE: " + getTOFDistance());
  }

  public void intakeIn() {
    setIntakeVelocity(kIntakeNoteRPM / 60);
  }

  public void intakeOut() {
    setIntakeVelocity(kOuttakeNoteRPM / 60);
  }

  @Override
  public void simulationPeriodic() {
    flywheelSimModel.setInputVoltage(intakeMotorSim.getMotorVoltage());
    flywheelSimModel.update(Robot.defaultPeriodSecs);
    intakeMotorSim.setRotorVelocity(flywheelSimModel.getAngularVelocityRPM());
  }
}
