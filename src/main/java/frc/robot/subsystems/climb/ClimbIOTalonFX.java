// Copyright (c) 2024 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.climb;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.utils.PhoenixUtil;
import frc.robot.utils.TalonUtil;

public class ClimbIOTalonFX implements ClimbIO {

  private final TalonFX leftClimbMotor = new TalonFX(ClimbConstants.kLeftClimbMotorID);
  private final PositionVoltage positionRequest = new PositionVoltage(0).withSlot(0);
  private final MotionMagicVoltage motionMagicRequest = new MotionMagicVoltage(0).withSlot(0);
  private final VoltageOut voltageReq = new VoltageOut(0);

  private final StatusSignal<Double> leftClimbMotorVoltage = leftClimbMotor.getMotorVoltage();
  private final StatusSignal<Double> leftClimbMotorVelocity = leftClimbMotor.getVelocity();
  private final StatusSignal<Double> leftClimbMotorPosition = leftClimbMotor.getPosition();
  private final StatusSignal<Double> leftClimbMotorStatorCurrent =
      leftClimbMotor.getStatorCurrent();
  private final StatusSignal<Double> leftClimbMotorSupplyCurrent =
      leftClimbMotor.getSupplyCurrent();
  private final StatusSignal<Double> leftClimbMotorTemperature = leftClimbMotor.getDeviceTemp();
  private final StatusSignal<Double> leftClimbMotorReferenceSlope =
      leftClimbMotor.getClosedLoopReferenceSlope();

  private final StatusSignal<Double> rightClimbMotorVoltage = leftClimbMotor.getMotorVoltage();
  private final StatusSignal<Double> rightClimbMotorVelocity = leftClimbMotor.getVelocity();
  private final StatusSignal<Double> rightClimbMotorPosition = leftClimbMotor.getPosition();
  private final StatusSignal<Double> rightClimbMotorStatorCurrent =
      leftClimbMotor.getStatorCurrent();
  private final StatusSignal<Double> rightClimbMotorSupplyCurrent =
      leftClimbMotor.getSupplyCurrent();
  private final StatusSignal<Double> rightClimbMotorTemperature = leftClimbMotor.getDeviceTemp();
  private final StatusSignal<Double> rightClimbMotorReferenceSlope =
      leftClimbMotor.getClosedLoopReferenceSlope();

  private final TalonFX rightClimbMotor = new TalonFX(ClimbConstants.kRightClimbMotorID);

  // private final Follower rightClimbFollowReq = new
  // Follower(leftClimbMotor.getDeviceID(), false);

  public ClimbIOTalonFX() {
    var leftClimbConfig = ClimbConstants.leftClimbConfig;
    PhoenixUtil.checkErrorAndRetry(() -> leftClimbMotor.getConfigurator().refresh(leftClimbConfig));
    TalonUtil.applyAndCheckConfiguration(leftClimbMotor, leftClimbConfig);

    var rightClimbConfig = ClimbConstants.rightClimbConfig;
    PhoenixUtil.checkErrorAndRetry(
        () -> rightClimbMotor.getConfigurator().refresh(rightClimbConfig));
    TalonUtil.applyAndCheckConfiguration(rightClimbMotor, rightClimbConfig);

    BaseStatusSignal.setUpdateFrequencyForAll(
        ClimbConstants.updateFrequency,
        leftClimbMotorVoltage,
        rightClimbMotorVoltage,
        leftClimbMotorVelocity,
        rightClimbMotorVelocity,
        leftClimbMotorPosition,
        rightClimbMotorPosition,
        leftClimbMotorStatorCurrent,
        rightClimbMotorStatorCurrent,
        leftClimbMotorSupplyCurrent,
        rightClimbMotorSupplyCurrent,
        leftClimbMotorTemperature,
        rightClimbMotorTemperature,
        leftClimbMotorReferenceSlope,
        rightClimbMotorReferenceSlope);
    leftClimbMotor.optimizeBusUtilization();
    rightClimbMotor.optimizeBusUtilization();
    // rightClimbMotor.setControl(rightClimbFollowReq);
  }

  @Override
  public void updateInputs(ClimbIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        leftClimbMotorVoltage,
        rightClimbMotorVoltage,
        leftClimbMotorVelocity,
        rightClimbMotorVelocity,
        leftClimbMotorPosition,
        rightClimbMotorPosition,
        leftClimbMotorStatorCurrent,
        rightClimbMotorStatorCurrent,
        leftClimbMotorSupplyCurrent,
        rightClimbMotorSupplyCurrent,
        leftClimbMotorTemperature,
        rightClimbMotorTemperature,
        leftClimbMotorReferenceSlope,
        rightClimbMotorReferenceSlope);
    inputs.leftClimbMotorVoltage = leftClimbMotorVoltage.getValueAsDouble();
    inputs.leftClimbMotorVelocity = leftClimbMotorVelocity.getValueAsDouble();
    inputs.leftClimbMotorPosition = leftClimbMotorPosition.getValueAsDouble();
    inputs.leftClimbMotorStatorCurrent = leftClimbMotorStatorCurrent.getValueAsDouble();
    inputs.leftClimbMotorSupplyCurrent = leftClimbMotorSupplyCurrent.getValueAsDouble();
    inputs.leftClimbMotorTemperature = leftClimbMotorTemperature.getValueAsDouble();
    inputs.leftClimbMotorReferenceSlope = leftClimbMotorReferenceSlope.getValueAsDouble();

    inputs.rightClimbMotorVoltage = rightClimbMotorVoltage.getValueAsDouble();
    inputs.rightClimbMotorVelocity = rightClimbMotorVelocity.getValueAsDouble();
    inputs.rightClimbMotorPosition = rightClimbMotorPosition.getValueAsDouble();
    inputs.rightClimbMotorStatorCurrent = rightClimbMotorStatorCurrent.getValueAsDouble();
    inputs.rightClimbMotorSupplyCurrent = rightClimbMotorSupplyCurrent.getValueAsDouble();
    inputs.rightClimbMotorTemperature = rightClimbMotorTemperature.getValueAsDouble();
    inputs.rightClimbMotorReferenceSlope = rightClimbMotorReferenceSlope.getValueAsDouble();
  }

  // @Override
  // public void setPosition(double position) {
  // if (ClimbConstants.kUseMotionMagic) {
  // leftClimbMotor.setControl(motionMagicRequest.withPosition(position));
  // rightClimbMotor.setControl(motionMagicRequest.withPosition(position));
  // } else {
  // leftClimbMotor.setControl(positionRequest.withPosition(position));
  // rightClimbMotor.setControl(positionRequest.withPosition(position));
  // }
  // }

  @Override
  public void setVoltageLeft(double voltage) {
    leftClimbMotor.setVoltage(voltage);
  }

  @Override
  public void setVoltageRight(double voltage) {
    rightClimbMotor.setVoltage(voltage);
  }

  @Override
  public void off() {
    leftClimbMotor.setControl(new StaticBrake());
    rightClimbMotor.setControl(new StaticBrake());
  }

  @Override
  public void zero() {
    leftClimbMotor.setPosition(0);
    rightClimbMotor.setPosition(0);
  }

  @Override
  public void goToZeroLeft() {
    if (ClimbConstants.kUseMotionMagic) {
      leftClimbMotor.setControl(motionMagicRequest.withPosition(0));
      // rightClimbMotor.setControl(motionMagicRequest.withPosition(0));
    } else {
      leftClimbMotor.setControl(positionRequest.withPosition(0));
      // rightClimbMotor.setControl(positionRequest.withPosition(0));
    }
  }

  @Override
  public void goToZeroRight() {
    if (ClimbConstants.kUseMotionMagic) {
      // leftClimbMotor.setControl(motionMagicRequest.withPosition(0));
      rightClimbMotor.setControl(motionMagicRequest.withPosition(0));
    } else {
      // leftClimbMotor.setControl(positionRequest.withPosition(0));
      rightClimbMotor.setControl(positionRequest.withPosition(0));
    }
  }

  @Override
  public TalonFX getMotor() {
    return leftClimbMotor;
  }

  @Override
  public VoltageOut getVoltageRequest() {
    return voltageReq;
  }
}
