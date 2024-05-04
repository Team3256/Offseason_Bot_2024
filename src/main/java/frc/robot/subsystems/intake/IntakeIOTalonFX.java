// Copyright (c) 2024 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.intake;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.drivers.MonitoredTalonFX;
import frc.robot.utils.PhoenixUtil;
import frc.robot.utils.TalonUtil;

public class IntakeIOTalonFX implements IntakeIO {
  private final MonitoredTalonFX intakeMotor = new MonitoredTalonFX(IntakeConstants.kIntakeMotorID);
  final VelocityVoltage intakeRequest = new VelocityVoltage(0).withSlot(0);
  final MotionMagicVelocityVoltage motionMagicIntakeRequest =
      new MotionMagicVelocityVoltage(0).withSlot(0);

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

  private final StatusSignal<Double> passthroughMotorVoltage = passthroughMotor.getMotorVoltage();
  private final StatusSignal<Double> passthroughMotorVelocity = passthroughMotor.getVelocity();
  private final StatusSignal<Double> passthroughMotorStatorCurrent =
      passthroughMotor.getStatorCurrent();
  private final StatusSignal<Double> passthroughMotorSupplyCurrent =
      passthroughMotor.getSupplyCurrent();
  private final StatusSignal<Double> passthroughMotorTemperature = passthroughMotor.getDeviceTemp();
  private final StatusSignal<Double> passthroughMotorReferenceSlope =
      passthroughMotor.getClosedLoopReferenceSlope();

  private DigitalInput beamBreakInput = new DigitalInput(IntakeConstants.kIntakeBeamBreakDIONew);

  public IntakeIOTalonFX() {
    var motorConfig = new TalonFXConfiguration();
    PhoenixUtil.checkErrorAndRetry(() -> intakeMotor.getConfigurator().refresh(motorConfig));
    motorConfig.Slot0.kS = IntakeConstants.kIntakeKS;
    motorConfig.Slot0.kV = IntakeConstants.kIntakeKV;
    motorConfig.Slot0.kP = IntakeConstants.kIntakeKP;
    motorConfig.Slot0.kI = IntakeConstants.kIntakeKI;
    motorConfig.Slot0.kD = IntakeConstants.kIntakeKD;
    motorConfig.MotorOutput.Inverted = IntakeConstants.intakeInverted;
    motorConfig.MotorOutput.NeutralMode = IntakeConstants.intakeNeutralMode;
    motorConfig.MotionMagic.MotionMagicAcceleration =
        IntakeConstants.kIntakeMotionMagicAcceleration;
    motorConfig.MotionMagic.MotionMagicCruiseVelocity = IntakeConstants.kIntakeMotionMagicVelocity;
    motorConfig.MotionMagic.MotionMagicJerk = IntakeConstants.kIntakeMotionMagicJerk;
    motorConfig.CurrentLimits.StatorCurrentLimitEnable = IntakeConstants.kIntakeCurrentLimitEnable;
    motorConfig.CurrentLimits.StatorCurrentLimit = IntakeConstants.kIntakeCurrentLimit;
    TalonUtil.applyAndCheckConfiguration(intakeMotor, motorConfig);

    var passthroughConfig = new TalonFXConfiguration();
    PhoenixUtil.checkErrorAndRetry(
        () -> passthroughMotor.getConfigurator().refresh(passthroughConfig));
    passthroughConfig.Slot0.kS = IntakeConstants.kPassthroughkS;
    passthroughConfig.Slot0.kV = IntakeConstants.kPassthroughkV;
    passthroughConfig.Slot0.kP = IntakeConstants.kPassthroughkP;
    passthroughConfig.Slot0.kI = IntakeConstants.kPassthroughkI;
    passthroughConfig.Slot0.kD = IntakeConstants.kPassthroughkD;
    passthroughConfig.MotorOutput.Inverted = IntakeConstants.passthroughInverted;
    passthroughConfig.MotorOutput.NeutralMode = IntakeConstants.passthroughNeutralMode;
    motorConfig.MotionMagic.MotionMagicAcceleration =
        IntakeConstants.kPassthroughMotionMagicAcceleration;
    motorConfig.MotionMagic.MotionMagicCruiseVelocity =
        IntakeConstants.kPassthroughMotionMagicVelocity;
    motorConfig.MotionMagic.MotionMagicJerk = IntakeConstants.kPassthroughMotionMagicJerk;
    motorConfig.CurrentLimits.StatorCurrentLimitEnable =
        IntakeConstants.kPassthroughCurrentLimitEnable;
    motorConfig.CurrentLimits.StatorCurrentLimit = IntakeConstants.kPassthroughCurrentLimit;
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
}
