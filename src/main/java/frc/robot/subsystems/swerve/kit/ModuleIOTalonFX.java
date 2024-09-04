// Copyright (c) 2024 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.swerve.kit;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import java.io.PrintStream;

public class ModuleIOTalonFX implements ModuleIO {
  private final TalonFX m_driveMotor;
  private final TalonFX m_steerMotor;
  private final CANcoder m_cancoder;
  private final StatusSignal<Double> m_driveVoltage;
  private final StatusSignal<Double> m_drivePosition;
  private final StatusSignal<Double> m_driveVelocity;
  private final StatusSignal<Double> m_driveStatorCurrent;
  private final StatusSignal<Double> m_driveSupplyCurrent;
  private final StatusSignal<Double> m_driveTemperature;

  private final StatusSignal<Double> m_steerVoltage;
  private final StatusSignal<Double> m_steerPosition;
  private final StatusSignal<Double> m_steerVelocity;
  private final StatusSignal<Double> m_steerStatorCurrent;
  private final StatusSignal<Double> m_steerSupplyCurrent;
  private final StatusSignal<Double> m_steerTemperature;
  private final BaseStatusSignal[] m_signals;
  private final double m_driveRotationsPerMeter;
  private final double m_couplingRatioDriveRotorToCANcoder;
  private final double m_speedAt12VoltsMps;
  private final MotionMagicVoltage m_angleVoltageSetter = new MotionMagicVoltage(0.0);
  private final MotionMagicTorqueCurrentFOC m_angleTorqueSetter =
      new MotionMagicTorqueCurrentFOC(0.0);
  private final MotionMagicExpoVoltage m_angleVoltageExpoSetter = new MotionMagicExpoVoltage(0.0);
  private final MotionMagicExpoTorqueCurrentFOC m_angleTorqueExpoSetter =
      new MotionMagicExpoTorqueCurrentFOC(0.0);
  private final VoltageOut m_voltageOpenLoopSetter = new VoltageOut(0.0);
  private final VelocityVoltage m_velocityVoltageSetter = new VelocityVoltage(0.0);
  private final VelocityTorqueCurrentFOC m_velocityTorqueSetter =
      (new VelocityTorqueCurrentFOC(0.0)).withOverrideCoastDurNeutral(true);
  private final SwerveModule.ClosedLoopOutputType m_steerClosedLoopOutput;
  private final SwerveModule.ClosedLoopOutputType m_driveClosedLoopOutput;
  private final SwerveModulePosition m_internalState = new SwerveModulePosition();
  private SwerveModuleState m_targetState = new SwerveModuleState();

  public ModuleIOTalonFX(SwerveModuleConstants constants, String canbusName) {
    this.m_driveMotor = new TalonFX(constants.DriveMotorId, canbusName);
    this.m_steerMotor = new TalonFX(constants.SteerMotorId, canbusName);
    this.m_cancoder = new CANcoder(constants.CANcoderId, canbusName);
    TalonFXConfiguration talonConfigs = new TalonFXConfiguration();
    talonConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    talonConfigs.Slot0 = constants.DriveMotorGains;
    talonConfigs.TorqueCurrent.PeakForwardTorqueCurrent = constants.SlipCurrent;
    talonConfigs.TorqueCurrent.PeakReverseTorqueCurrent = -constants.SlipCurrent;
    talonConfigs.CurrentLimits.StatorCurrentLimit = constants.SlipCurrent;
    talonConfigs.CurrentLimits.StatorCurrentLimitEnable = true;
    talonConfigs.MotorOutput.Inverted =
        constants.DriveMotorInverted
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;
    StatusCode response = this.m_driveMotor.getConfigurator().apply(talonConfigs);
    PrintStream var10000;
    int var10001;
    if (!response.isOK()) {
      var10000 = System.out;
      var10001 = this.m_driveMotor.getDeviceID();
      var10000.println(
          "TalonFX ID " + var10001 + " failed config with error " + response.toString());
    }

    talonConfigs.TorqueCurrent = new TorqueCurrentConfigs();
    talonConfigs.CurrentLimits = new CurrentLimitsConfigs();
    talonConfigs.Slot0 = constants.SteerMotorGains;
    talonConfigs.Feedback.FeedbackRemoteSensorID = constants.CANcoderId;
    switch (constants.FeedbackSource) {
      case RemoteCANcoder:
        talonConfigs.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
        break;
      case FusedCANcoder:
        talonConfigs.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
        break;
      case SyncCANcoder:
        talonConfigs.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.SyncCANcoder;
    }

    talonConfigs.Feedback.RotorToSensorRatio = constants.SteerMotorGearRatio;
    talonConfigs.MotionMagic.MotionMagicCruiseVelocity = 100.0 / constants.SteerMotorGearRatio;
    talonConfigs.MotionMagic.MotionMagicAcceleration =
        talonConfigs.MotionMagic.MotionMagicCruiseVelocity / 0.1;
    talonConfigs.MotionMagic.MotionMagicExpo_kV = 0.12 * constants.SteerMotorGearRatio;
    talonConfigs.MotionMagic.MotionMagicExpo_kA = 0.1;
    talonConfigs.ClosedLoopGeneral.ContinuousWrap = true;
    talonConfigs.MotorOutput.Inverted =
        constants.SteerMotorInverted
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;
    response = this.m_steerMotor.getConfigurator().apply(talonConfigs);
    if (!response.isOK()) {
      var10000 = System.out;
      var10001 = this.m_steerMotor.getDeviceID();
      var10000.println(
          "TalonFX ID " + var10001 + " failed config with error " + response.toString());
    }

    CANcoderConfiguration cancoderConfigs = new CANcoderConfiguration();
    cancoderConfigs.MagnetSensor.MagnetOffset = constants.CANcoderOffset;
    response = this.m_cancoder.getConfigurator().apply(cancoderConfigs);
    if (!response.isOK()) {
      var10000 = System.out;
      var10001 = this.m_cancoder.getDeviceID();
      var10000.println(
          "CANcoder ID " + var10001 + " failed config with error " + response.toString());
    }

    this.m_driveVoltage = this.m_driveMotor.getMotorVoltage().clone();
    this.m_drivePosition = this.m_driveMotor.getPosition().clone();
    this.m_driveVelocity = this.m_driveMotor.getVelocity().clone();
    this.m_driveStatorCurrent = this.m_driveMotor.getStatorCurrent().clone();
    this.m_driveSupplyCurrent = this.m_driveMotor.getSupplyCurrent().clone();
    this.m_driveTemperature = this.m_driveMotor.getDeviceTemp().clone();

    this.m_steerVoltage = this.m_steerMotor.getMotorVoltage().clone();
    this.m_steerPosition = this.m_steerMotor.getPosition().clone();
    this.m_steerVelocity = this.m_steerMotor.getVelocity().clone();
    this.m_steerStatorCurrent = this.m_steerMotor.getStatorCurrent().clone();
    this.m_steerSupplyCurrent = this.m_steerMotor.getSupplyCurrent().clone();
    this.m_steerTemperature = this.m_steerMotor.getDeviceTemp().clone();
    this.m_signals = new BaseStatusSignal[4];
    this.m_signals[0] = this.m_drivePosition;
    this.m_signals[1] = this.m_driveVelocity;
    this.m_signals[2] = this.m_steerPosition;
    this.m_signals[3] = this.m_steerVelocity;
    double rotationsPerWheelRotation = constants.DriveMotorGearRatio;
    double metersPerWheelRotation = 6.283185307179586 * Units.inchesToMeters(constants.WheelRadius);
    this.m_driveRotationsPerMeter = rotationsPerWheelRotation / metersPerWheelRotation;
    this.m_couplingRatioDriveRotorToCANcoder = constants.CouplingGearRatio;
    this.m_angleVoltageSetter.UpdateFreqHz = 0.0;
    this.m_angleTorqueSetter.UpdateFreqHz = 0.0;
    this.m_angleVoltageExpoSetter.UpdateFreqHz = 0.0;
    this.m_angleTorqueExpoSetter.UpdateFreqHz = 0.0;
    this.m_velocityTorqueSetter.UpdateFreqHz = 0.0;
    this.m_velocityVoltageSetter.UpdateFreqHz = 0.0;
    this.m_voltageOpenLoopSetter.UpdateFreqHz = 0.0;
    this.m_steerClosedLoopOutput = constants.SteerMotorClosedLoopOutput;
    this.m_driveClosedLoopOutput = constants.DriveMotorClosedLoopOutput;
    this.m_speedAt12VoltsMps = constants.SpeedAt12VoltsMps;
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        m_driveVoltage,
        m_drivePosition,
        m_driveVelocity,
        m_driveStatorCurrent,
        m_driveSupplyCurrent,
        m_driveTemperature,
        m_steerVoltage,
        m_steerPosition,
        m_steerVelocity,
        m_steerStatorCurrent,
        m_steerSupplyCurrent,
        m_steerTemperature);
    inputs.driveMotorVoltage = m_driveVoltage.getValueAsDouble();
    inputs.driveMotorPosition = m_drivePosition.getValueAsDouble();
    inputs.driveMotorVelocity = m_driveVelocity.getValueAsDouble();
    inputs.driveMotorStatorCurrent = m_driveStatorCurrent.getValueAsDouble();
    inputs.driveMotorSupplyCurrent = m_driveSupplyCurrent.getValueAsDouble();
    inputs.driveMotorTemperature = m_driveTemperature.getValueAsDouble();

    inputs.angleMotorVoltage = m_steerVoltage.getValueAsDouble();
    inputs.angleMotorPosition = m_steerPosition.getValueAsDouble();
    inputs.angleMotorVelocity = m_steerVelocity.getValueAsDouble();
    inputs.angleMotorStatorCurrent = m_steerStatorCurrent.getValueAsDouble();
    inputs.angleMotorSupplyCurrent = m_steerSupplyCurrent.getValueAsDouble();
    inputs.angleMotorTemperature = m_steerTemperature.getValueAsDouble();

    inputs.currentState = getCurrentState();
    inputs.currentPosition = getPosition(false);
  }

  @Override
  public SwerveModulePosition getPosition(boolean refresh) {
    if (refresh) {
      this.m_drivePosition.refresh();
      this.m_driveVelocity.refresh();
      this.m_steerPosition.refresh();
      this.m_steerVelocity.refresh();
    }

    double drive_rot =
        BaseStatusSignal.getLatencyCompensatedValue(this.m_drivePosition, this.m_driveVelocity);
    double angle_rot =
        BaseStatusSignal.getLatencyCompensatedValue(this.m_steerPosition, this.m_steerVelocity);
    drive_rot -= angle_rot * this.m_couplingRatioDriveRotorToCANcoder;
    this.m_internalState.distanceMeters = drive_rot / this.m_driveRotationsPerMeter;
    this.m_internalState.angle = Rotation2d.fromRotations(angle_rot);
    return this.m_internalState;
  }

  public void apply(SwerveModuleState state, SwerveModule.DriveRequestType driveRequestType) {
    this.apply(state, driveRequestType, SwerveModule.SteerRequestType.MotionMagic);
  }

  @Override
  public void apply(
      SwerveModuleState state,
      SwerveModule.DriveRequestType driveRequestType,
      SwerveModule.SteerRequestType steerRequestType) {
    SwerveModuleState optimized;
    double angleToSetDeg;
    optimized = SwerveModuleState.optimize(state, this.m_internalState.angle);
    this.m_targetState = optimized;
    angleToSetDeg = optimized.angle.getRotations();
    label35:
    switch (steerRequestType) {
      case MotionMagic:
        switch (this.m_steerClosedLoopOutput) {
          case Voltage:
            this.m_steerMotor.setControl(this.m_angleVoltageSetter.withPosition(angleToSetDeg));
            break label35;
          case TorqueCurrentFOC:
            this.m_steerMotor.setControl(this.m_angleTorqueSetter.withPosition(angleToSetDeg));
          default:
            break label35;
        }
      case MotionMagicExpo:
        switch (this.m_steerClosedLoopOutput) {
          case Voltage:
            this.m_steerMotor.setControl(this.m_angleVoltageExpoSetter.withPosition(angleToSetDeg));
            break;
          case TorqueCurrentFOC:
            this.m_steerMotor.setControl(this.m_angleTorqueExpoSetter.withPosition(angleToSetDeg));
        }
    }

    double velocityToSet = optimized.speedMetersPerSecond * this.m_driveRotationsPerMeter;
    double steerMotorError = angleToSetDeg - (Double) this.m_steerPosition.getValue();
    double cosineScalar = Math.cos(Units.rotationsToRadians(steerMotorError));
    if (cosineScalar < 0.0) {
      cosineScalar = 0.0;
    }

    velocityToSet *= cosineScalar;
    double azimuthTurnRps = (Double) this.m_steerVelocity.getValue();
    double driveRateBackOut = azimuthTurnRps * this.m_couplingRatioDriveRotorToCANcoder;
    velocityToSet += driveRateBackOut;
    switch (driveRequestType) {
      case OpenLoopVoltage:
        velocityToSet /= this.m_driveRotationsPerMeter;
        this.m_driveMotor.setControl(
            this.m_voltageOpenLoopSetter.withOutput(
                velocityToSet / this.m_speedAt12VoltsMps * 12.0));
        break;
      case Velocity:
        switch (this.m_driveClosedLoopOutput) {
          case Voltage:
            this.m_driveMotor.setControl(this.m_velocityVoltageSetter.withVelocity(velocityToSet));
            break;
          case TorqueCurrentFOC:
            this.m_driveMotor.setControl(this.m_velocityTorqueSetter.withVelocity(velocityToSet));
        }
    }
  }

  @Override
  public void applyCharacterization(Rotation2d steerTarget, VoltageOut driveRequest) {
    double angleToSetDeg = steerTarget.getRotations();
    switch (this.m_steerClosedLoopOutput) {
      case Voltage:
        this.m_steerMotor.setControl(this.m_angleVoltageSetter.withPosition(angleToSetDeg));
        break;
      case TorqueCurrentFOC:
        this.m_steerMotor.setControl(this.m_angleTorqueSetter.withPosition(angleToSetDeg));
    }

    this.m_driveMotor.setControl(driveRequest);
  }

  @Override
  public void applyCharacterization(Rotation2d steerTarget, TorqueCurrentFOC driveRequest) {
    double angleToSetDeg = steerTarget.getRotations();
    switch (this.m_steerClosedLoopOutput) {
      case Voltage:
        this.m_steerMotor.setControl(this.m_angleVoltageSetter.withPosition(angleToSetDeg));
        break;
      case TorqueCurrentFOC:
        this.m_steerMotor.setControl(this.m_angleTorqueSetter.withPosition(angleToSetDeg));
    }

    this.m_driveMotor.setControl(driveRequest);
  }

  @Override
  public StatusCode configNeutralMode(NeutralModeValue neutralMode) {
    MotorOutputConfigs configs = new MotorOutputConfigs();
    StatusCode status = this.m_driveMotor.getConfigurator().refresh(configs);
    if (status.isOK()) {
      configs.NeutralMode = neutralMode;
      status = this.m_driveMotor.getConfigurator().apply(configs);
    }

    if (!status.isOK()) {
      PrintStream var10000 = System.out;
      int var10001 = this.m_driveMotor.getDeviceID();
      var10000.println(
          "TalonFX ID " + var10001 + " failed config neutral mode with error " + status.toString());
    }

    return status;
  }

  @Override
  public SwerveModulePosition getCachedPosition() {
    return this.m_internalState;
  }

  @Override
  public SwerveModuleState getCurrentState() {
    return new SwerveModuleState(
        (Double) this.m_driveVelocity.getValue() / this.m_driveRotationsPerMeter,
        Rotation2d.fromRotations((Double) this.m_steerPosition.getValue()));
  }

  @Override
  public SwerveModuleState getTargetState() {
    return this.m_targetState;
  }

  @Override
  public BaseStatusSignal[] getSignals() {
    return this.m_signals;
  }

  @Override
  public void resetPosition() {
    this.m_driveMotor.setPosition(0.0);
  }

  @Override
  public TalonFX getDriveMotor() {
    return this.m_driveMotor;
  }

  @Override
  public TalonFX getSteerMotor() {
    return this.m_steerMotor;
  }

  @Override
  public CANcoder getCANcoder() {
    return this.m_cancoder;
  }

  public static enum DriveRequestType {
    OpenLoopVoltage,
    Velocity;

    private DriveRequestType() {}
  }

  public static enum SteerRequestType {
    MotionMagic,
    MotionMagicExpo;

    private SteerRequestType() {}
  }

  public static enum ClosedLoopOutputType {
    Voltage,
    TorqueCurrentFOC;

    private ClosedLoopOutputType() {}
  }
}
