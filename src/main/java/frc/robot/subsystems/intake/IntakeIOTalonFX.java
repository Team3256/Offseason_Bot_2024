// Copyright (c) 2024 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.intake;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.drivers.MonitoredTalonFX;
import frc.robot.utils.PhoenixUtil;
import frc.robot.utils.TalonUtil;

public class IntakeIOTalonFX implements IntakeIO {
  private final MonitoredTalonFX intakeMotor = new MonitoredTalonFX(IntakeConstants.kIntakeMotorID);
  final VelocityVoltage intakeRequest = new VelocityVoltage(0).withSlot(0);
  final MotionMagicVelocityVoltage motionMagicIntakeRequest =
      new MotionMagicVelocityVoltage(0).withSlot(0);
  private final VoltageOut intakeVoltageReq = new VoltageOut(0);

  private final StatusSignal<Double> intakeMotorVoltage = intakeMotor.getMotorVoltage();
  private final StatusSignal<Double> intakeMotorVelocity = intakeMotor.getVelocity();
  private final StatusSignal<Double> intakeMotorStatorCurrent = intakeMotor.getStatorCurrent();
  private final StatusSignal<Double> intakeMotorSupplyCurrent = intakeMotor.getSupplyCurrent();
  private final StatusSignal<Double> intakeMotorTemperature = intakeMotor.getDeviceTemp();
  private final StatusSignal<Double> intakeMotorReferenceSlope =
      intakeMotor.getClosedLoopReferenceSlope();

  private final MonitoredTalonFX passthroughMotor =
      new MonitoredTalonFX(IntakeConstants.kPassthroughMotorID);
  final VelocityVoltage passthroughRequest = new VelocityVoltage(0).withSlot(0);
  final MotionMagicVelocityVoltage motionMagicPassthroughRequest =
      new MotionMagicVelocityVoltage(0).withSlot(0);
  private final VoltageOut passthroughVoltageReq = new VoltageOut(0);

  private final StatusSignal<Double> passthroughMotorVoltage = passthroughMotor.getMotorVoltage();
  private final StatusSignal<Double> passthroughMotorVelocity = passthroughMotor.getVelocity();
  private final StatusSignal<Double> passthroughMotorStatorCurrent =
      passthroughMotor.getStatorCurrent();
  private final StatusSignal<Double> passthroughMotorSupplyCurrent =
      passthroughMotor.getSupplyCurrent();
  private final StatusSignal<Double> passthroughMotorTemperature = passthroughMotor.getDeviceTemp();
  private final StatusSignal<Double> passthroughMotorReferenceSlope =
      passthroughMotor.getClosedLoopReferenceSlope();

  private DigitalInput beamBreakInput = new DigitalInput(IntakeConstants.kIntakeBeamBreakDIO);

  public IntakeIOTalonFX() {
    var motorConfig = IntakeConstants.intakeMotorConfig;
    PhoenixUtil.checkErrorAndRetry(() -> intakeMotor.getConfigurator().refresh(motorConfig));
    TalonUtil.applyAndCheckConfiguration(intakeMotor, motorConfig);

    var passthroughConfig = IntakeConstants.passthroughMotorConfig;
    PhoenixUtil.checkErrorAndRetry(
        () -> passthroughMotor.getConfigurator().refresh(passthroughConfig));
    TalonUtil.applyAndCheckConfiguration(passthroughMotor, passthroughConfig);

    BaseStatusSignal.setUpdateFrequencyForAll(
        IntakeConstants.updateFrequency,
        intakeMotorVoltage,
        intakeMotorVelocity,
        intakeMotorStatorCurrent,
        intakeMotorSupplyCurrent,
        intakeMotorTemperature,
        intakeMotorReferenceSlope,
        passthroughMotorVoltage,
        passthroughMotorVelocity,
        passthroughMotorStatorCurrent,
        passthroughMotorSupplyCurrent,
        passthroughMotorTemperature,
        passthroughMotorReferenceSlope);
    intakeMotor.optimizeBusUtilization();
    passthroughMotor.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        intakeMotorVoltage,
        intakeMotorVelocity,
        intakeMotorStatorCurrent,
        intakeMotorSupplyCurrent,
        intakeMotorTemperature,
        intakeMotorReferenceSlope,
        passthroughMotorVoltage,
        passthroughMotorVelocity,
        passthroughMotorStatorCurrent,
        passthroughMotorSupplyCurrent,
        passthroughMotorTemperature,
        passthroughMotorReferenceSlope);
    inputs.intakeMotorVoltage = intakeMotorVoltage.getValueAsDouble();
    inputs.intakeMotorVelocity = intakeMotorVelocity.getValueAsDouble();
    inputs.intakeMotorStatorCurrent = intakeMotorStatorCurrent.getValueAsDouble();
    inputs.intakeMotorSupplyCurrent = intakeMotorSupplyCurrent.getValueAsDouble();
    inputs.intakeMotorTemperature = intakeMotorTemperature.getValueAsDouble();
    inputs.intakeMotorReferenceSlope = intakeMotorReferenceSlope.getValueAsDouble();

    inputs.passthroughMotorVoltage = passthroughMotorVoltage.getValueAsDouble();
    inputs.passthroughMotorVelocity = passthroughMotorVelocity.getValueAsDouble();
    inputs.passthroughMotorStatorCurrent = passthroughMotorStatorCurrent.getValueAsDouble();
    inputs.passthroughMotorSupplyCurrent = passthroughMotorSupplyCurrent.getValueAsDouble();
    inputs.passthroughMotorTemperature = passthroughMotorTemperature.getValueAsDouble();
    inputs.passthroughMotorReferenceSlope = passthroughMotorReferenceSlope.getValueAsDouble();

    inputs.isBeamBroken = !beamBreakInput.get();
  }

  @Override
  public void setIntakeVoltage(double voltage) {
    intakeMotor.setVoltage(voltage);
  }

  @Override
  public void setIntakeVelocity(double velocity) {
    if (IntakeConstants.kIntakeMotionMagic) {
      intakeMotor.setControl(motionMagicIntakeRequest.withVelocity(velocity));
    } else {
      intakeMotor.setControl(intakeRequest.withVelocity(velocity));
    }
  }

  @Override
  public void setPassthroughVoltage(double voltage) {
    passthroughMotor.setVoltage(voltage);
  }

  @Override
  public void setPassthroughVelocity(double velocity) {
    if (IntakeConstants.kPassthroughMotionMagic) {
      passthroughMotor.setControl(motionMagicPassthroughRequest.withVelocity(velocity));
    } else {
      passthroughMotor.setControl(passthroughRequest.withVelocity(velocity));
    }
  }

  @Override
  public void off() {
    intakeMotor.setControl(new NeutralOut());
    passthroughMotor.setControl(new NeutralOut());
  }

  @Override
  public boolean isBeamBroken() {
    return !beamBreakInput.get();
  }

  @Override
  public MonitoredTalonFX getIntakeMotor() {
    return intakeMotor;
  }

  @Override
  public VoltageOut getIntakeVoltageRequest() {
    return intakeVoltageReq;
  }

  @Override
  public MonitoredTalonFX getPassthroughMotor() {
    return passthroughMotor;
  }

  @Override
  public VoltageOut getPassthroughVoltageRequest() {
    return passthroughVoltageReq;
  }
}
