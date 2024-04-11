// Copyright (c) 2024 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.pivot;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.StaticBrake;
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
import frc.robot.utils.PhoenixUtil;
import frc.robot.utils.TalonUtil;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Config;
import io.github.oblarg.oblog.annotations.Log;
import org.littletonrobotics.junction.AutoLogOutput;

public class PivotIntake extends SubsystemBase implements Loggable {

  private MonitoredTalonFX pivotMotor;
  private TalonFXSimState pivotMotorSim;
  private SingleJointedArmSim pivotModel;

  final PositionVoltage positionRequest =
      new PositionVoltage(0).withEnableFOC(PivotConstants.kUseFOC).withSlot(0);
  final MotionMagicVoltage motionMagicRequest =
      new MotionMagicVoltage(0).withEnableFOC(PivotConstants.kUseFOC).withSlot(0);

  public void DEFAULT() {}

  public PivotIntake() {
    if (RobotBase.isReal()) {
      configureRealHardware();
    } else {
      configureSimHardware();
    }
  }

  private void configureRealHardware() {
    pivotMotor = new MonitoredTalonFX(PivotConstants.kPivotMotorID);
    pivotMotor.setNeutralMode(NeutralModeValue.Brake);

    setMotorConfigs(
        PivotConstants.kS,
        PivotConstants.kV,
        PivotConstants.kP,
        PivotConstants.kI,
        PivotConstants.kD,
        PivotConstants.motionMagicVelocity,
        PivotConstants.motionMagicAcceleration,
        PivotConstants.motionMagicJerk,
        PivotConstants.enableStatorLimit,
        PivotConstants.statorLimit);
  }

  // @Config(name = "Pivot Motor Slot 0 Configs")
  private void setMotorConfigs(
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
        () -> pivotMotor.getConfigurator().refresh(talonFXConfiguration));
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
    TalonUtil.applyAndCheckConfiguration(pivotMotor, talonFXConfiguration);
  }

  private void configureSimHardware() {
    configureRealHardware();
    pivotMotorSim = pivotMotor.getSimState();
    pivotMotorSim.setSupplyVoltage(12);
    pivotModel =
        new SingleJointedArmSim(
            DCMotor.getFalcon500(PivotConstants.kNumPivotMotors),
            PivotConstants.kPivotMotorGearing,
            PivotConstants.jKgMetersSquared,
            PivotConstants.kPivotLength,
            Units.degreesToRadians(PivotConstants.kPivotMinAngleDeg),
            Units.degreesToRadians(PivotConstants.kPivotMaxAngleDeg),
            false,
            PivotConstants.kPivotMinAngleDeg);
  }

  public void off() {
    pivotMotor.setControl(new NeutralOut());
    if (!Robot.isReal()) {
      pivotModel.setInputVoltage(0);
    }
    if (Constants.FeatureFlags.kDebugEnabled) {
      System.out.println("Pivot off");
    }
  }

  @Config(name = "Set Pivot Voltage")
  public void setOutputVoltage(double voltage) {
    pivotMotor.setVoltage(voltage);
    if (Constants.FeatureFlags.kDebugEnabled) {
      System.out.println("Pivot voltage set to: " + voltage);
    }
  }

  @Config(name = "Set Pivot Position")
  public void setDegrees(double degrees) {
    if (Constants.FeatureFlags.kDebugEnabled) {
      System.out.println("Pivot set to: " + degrees + " deg");
    }
    if (PivotConstants.kUseMotionMagic) {
      pivotMotor.setControl(motionMagicRequest.withPosition(degrees));
    } else {
      pivotMotor.setControl(positionRequest.withPosition(degrees));
    }
  }

  public void setControlStaticBrake() {
    pivotMotor.setControl(new StaticBrake());
  }

  @Log.Graph(name = "PivotIntake Voltage")
  @AutoLogOutput
  public double getVoltage() {
    return pivotMotor.getMotorVoltage().getValue();
  }

  @Log.Graph(name = "PivotIntake Velocity")
  @AutoLogOutput
  public double getVelocity() {
    return pivotMotor.getRotorVelocity().getValue();
  }

  public boolean getVelocityStalled() {
    return Math.abs(pivotMotor.getRotorVelocity().getValue()) < 2;
  }

  @Log.Graph(name = "PivotIntake Position")
  @AutoLogOutput
  public double getDegrees() {
    return ((pivotMotor.getRotorPosition().getValue()) * 360);
  }

  /*
   * ZERO THE PIVOT ENCODER
   * ONLY CALL THIS WHEN YOU ARE FOR SURE THE PIVOT IS AT ITS ZERO POSITION. USE
   * THE CALIBRATION FUNCTIONALITY TO SET THIS UP!!
   */
  public void zero() {
    if (FeatureFlags.kDebugEnabled) {
      System.out.println("[PivotIntake] Setting zero position to: " + getDegrees());
    }
    pivotMotor.setPosition(0);
    if (FeatureFlags.kDebugEnabled) {
      System.out.println("[PivotIntake] Zero position set. Current position: " + getDegrees());
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

  // @Log.Graph(name = "PivotIntake Stator Current")
  @AutoLogOutput
  public double getCurrent() {
    return pivotMotor.getSupplyCurrent().getValue();
  }

  @Log
  @AutoLogOutput
  public boolean isCurrentSpiking() {
    return this.getCurrent() > PivotConstants.kCurrentThreshold;
  }

  @Log
  @AutoLogOutput
  public boolean isMotorStalled() {
    return this.isCurrentSpiking()
        && this.pivotMotor.getRotorVelocity().getValue() < PivotConstants.kStallVelocityThreshold;
  }

  @Override
  public void periodic() {}

  @Override
  public void simulationPeriodic() {
    pivotModel.setInputVoltage(pivotMotorSim.getMotorVoltage());
    pivotModel.update(Robot.defaultPeriodSecs);
    pivotMotorSim.setRotorVelocity(pivotModel.getVelocityRadPerSec() / (2 * Math.PI));
    pivotMotorSim.setRawRotorPosition(pivotModel.getAngleRads() / (2 * Math.PI));
  }
}
