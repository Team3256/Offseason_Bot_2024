// Copyright (c) 2024 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.drivers.MonitoredTalonFX;
import frc.robot.utils.PhoenixUtil;
import frc.robot.utils.TalonUtil;

public class ShooterIOTalonFX implements ShooterIO {
  private final MonitoredTalonFX shooterMotor = new MonitoredTalonFX(ShooterConstants.kShooterMotorID);
  final VelocityVoltage velocityRequest = new VelocityVoltage(0).withSlot(0);
  final MotionMagicVelocityVoltage motionMagicRequest = new MotionMagicVelocityVoltage(0).withSlot(0);
  private final VoltageOut voltageReq = new VoltageOut(0);

  private final StatusSignal<Double> shooterMotorVoltage = shooterMotor.getMotorVoltage();
  private final StatusSignal<Double> shooterMotorVelocity = shooterMotor.getVelocity();
  private final StatusSignal<Double> shooterMotorStatorCurrent = shooterMotor.getStatorCurrent();
  private final StatusSignal<Double> shooterMotorSupplyCurrent = shooterMotor.getSupplyCurrent();
  private final StatusSignal<Double> shooterMotorTemperature = shooterMotor.getDeviceTemp();
  private final StatusSignal<Double> shooterMotorReferenceSlope = shooterMotor.getClosedLoopReferenceSlope();

  private final MonitoredTalonFX shooterMotorFollower = new MonitoredTalonFX(ShooterConstants.kShooterMotorFollowerID);
  final VelocityVoltage velocityRequestFollower = new VelocityVoltage(0).withSlot(0);
  final MotionMagicVelocityVoltage motionMagicRequestFollower = new MotionMagicVelocityVoltage(0).withSlot(0);

  private final StatusSignal<Double> shooterMotorFollowerVoltage = shooterMotorFollower.getMotorVoltage();
  private final StatusSignal<Double> shooterMotorFollowerVelocity = shooterMotorFollower.getVelocity();
  private final StatusSignal<Double> shooterMotorFollowerStatorCurrent = shooterMotorFollower.getStatorCurrent();
  private final StatusSignal<Double> shooterMotorFollowerSupplyCurrent = shooterMotorFollower.getSupplyCurrent();
  private final StatusSignal<Double> shooterMotorFollowerTemperature = shooterMotorFollower.getDeviceTemp();
  private final StatusSignal<Double> shooterMotorFollowerReferenceSlope = shooterMotorFollower
      .getClosedLoopReferenceSlope();

  public ShooterIOTalonFX() {
    var motorConfig = ShooterConstants.motorConfigs;
    PhoenixUtil.checkErrorAndRetry(() -> shooterMotor.getConfigurator().refresh(motorConfig));
    TalonUtil.applyAndCheckConfiguration(shooterMotor, motorConfig);

    var motorConfigFollower = ShooterConstants.followerMotorConfigs;
    PhoenixUtil.checkErrorAndRetry(
        () -> shooterMotorFollower.getConfigurator().refresh(motorConfigFollower));
    TalonUtil.applyAndCheckConfiguration(shooterMotorFollower, motorConfigFollower);

    BaseStatusSignal.setUpdateFrequencyForAll(
        ShooterConstants.updateFrequency,
        shooterMotorVoltage,
        shooterMotorVelocity,
        shooterMotorStatorCurrent,
        shooterMotorSupplyCurrent,
        shooterMotorTemperature,
        shooterMotorReferenceSlope,
        shooterMotorFollowerVoltage,
        shooterMotorFollowerVelocity,
        shooterMotorFollowerStatorCurrent,
        shooterMotorFollowerSupplyCurrent,
        shooterMotorFollowerTemperature,
        shooterMotorFollowerReferenceSlope);
    shooterMotor.optimizeBusUtilization();
    shooterMotorFollower.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        shooterMotorVoltage,
        shooterMotorVelocity,
        shooterMotorStatorCurrent,
        shooterMotorSupplyCurrent,
        shooterMotorTemperature,
        shooterMotorReferenceSlope,
        shooterMotorFollowerVoltage,
        shooterMotorFollowerVelocity,
        shooterMotorFollowerStatorCurrent,
        shooterMotorFollowerSupplyCurrent,
        shooterMotorFollowerTemperature,
        shooterMotorFollowerReferenceSlope);
    inputs.shooterMotorVoltage = shooterMotorVoltage.getValueAsDouble();
    inputs.shooterMotorVelocity = shooterMotorVelocity.getValueAsDouble();
    inputs.shooterMotorStatorCurrent = shooterMotorStatorCurrent.getValueAsDouble();
    inputs.shooterMotorSupplyCurrent = shooterMotorSupplyCurrent.getValueAsDouble();
    inputs.shooterMotorTemperature = shooterMotorTemperature.getValueAsDouble();
    inputs.shooterMotorReferenceSlope = shooterMotorReferenceSlope.getValueAsDouble();

    inputs.shooterMotorFollowerVoltage = shooterMotorFollowerVoltage.getValueAsDouble();
    inputs.shooterMotorFollowerVelocity = shooterMotorFollowerVelocity.getValueAsDouble();
    inputs.shooterMotorFollowerStatorCurrent = shooterMotorFollowerStatorCurrent.getValueAsDouble();
    inputs.shooterMotorFollowerSupplyCurrent = shooterMotorFollowerSupplyCurrent.getValueAsDouble();
    inputs.shooterMotorFollowerTemperature = shooterMotorFollowerTemperature.getValueAsDouble();
    inputs.shooterMotorFollowerReferenceSlope = shooterMotorFollowerReferenceSlope.getValueAsDouble();
  }

  @Override
  public void setShooterVoltage(double voltage) {
    shooterMotor.setVoltage(voltage);
  }

  @Override
  public void setShooterVelocity(double velocity) {
    if (ShooterConstants.kUseMotionMagic) {
      shooterMotor.setControl(motionMagicRequest.withVelocity(velocity));
    } else {
      shooterMotor.setControl(velocityRequest.withVelocity(velocity));
    }
  }

  @Override
  public void setShooterFollowerVoltage(double voltage) {
    shooterMotorFollower.setVoltage(voltage);
  }

  @Override
  public void setShooterFollowerVelocity(double velocity) {
    if (ShooterConstants.kUseMotionMagic) {
      shooterMotorFollower.setControl(motionMagicRequestFollower.withVelocity(velocity));
    } else {
      shooterMotorFollower.setControl(velocityRequestFollower.withVelocity(velocity));
    }
  }

  @Override
  public void regenerativeBrake() {
    shooterMotor.setControl(new VelocityTorqueCurrentFOC(0).withSlot(1).withVelocity(0));
    shooterMotorFollower.setControl(new VelocityTorqueCurrentFOC(0).withSlot(1).withVelocity(0));
  }

  @Override
  public void off() {
    shooterMotor.setControl(new NeutralOut());
    shooterMotorFollower.setControl(new NeutralOut());
  }

  @Override
  public MonitoredTalonFX getMotor() {
    return shooterMotor;
  }

  @Override
  public VoltageOut getVoltageRequest() {
    return voltageReq;
  }
}
