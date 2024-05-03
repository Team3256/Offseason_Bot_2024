package frc.robot.subsystems.pivotshooter;

import frc.robot.drivers.MonitoredTalonFX;
import frc.robot.limelight.LimelightHelpers;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionVoltage;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import frc.robot.utils.PhoenixUtil;
import frc.robot.utils.TalonUtil;

public class PivotShooterIOTalonFX implements PivotShooterIO {

    private final InterpolatingDoubleTreeMap aprilTagMap = new InterpolatingDoubleTreeMap(){{
        put(0.0, 0.0);
        put(1.0, 1.0);
    }};
  private final MonitoredTalonFX pivotShooterMotor =
      new MonitoredTalonFX(PivotShooterConstants.kPivotMotorID);
  final PositionVoltage positionRequest = new PositionVoltage(0).withSlot(0);
  final MotionMagicVoltage motionMagicRequest = new MotionMagicVoltage(0).withSlot(0);

  private final StatusSignal<Double> pivotShooterMotorVoltage = pivotShooterMotor.getMotorVoltage();
  private final StatusSignal<Double> pivotShooterMotorPosition = pivotShooterMotor.getPosition();
  private final StatusSignal<Double> pivotShooterMotorStatorCurrent =
      pivotShooterMotor.getStatorCurrent();
  private final StatusSignal<Double> pivotShooterMotorSupplyCurrent =
      pivotShooterMotor.getSupplyCurrent();
  private final StatusSignal<Double> pivotShooterMotorTemperature = pivotShooterMotor.getDeviceTemp();

  public PivotShooterIOTalonFX() {
    var motorConfig = new TalonFXConfiguration();
    PhoenixUtil.checkErrorAndRetry(() -> pivotShooterMotor.getConfigurator().refresh(motorConfig));
    motorConfig.Slot0.kS = PivotShooterConstants.kS;
    motorConfig.Slot0.kV = PivotShooterConstants.kV;
    motorConfig.Slot0.kP = PivotShooterConstants.kP;
    motorConfig.Slot0.kI = PivotShooterConstants.kI;
    motorConfig.Slot0.kD = PivotShooterConstants.kD;
    motorConfig.MotionMagic.MotionMagicAcceleration = PivotShooterConstants.motionMagicVelocity;
    motorConfig.MotionMagic.MotionMagicCruiseVelocity =
        PivotShooterConstants.motionMagicAcceleration;
    motorConfig.MotionMagic.MotionMagicJerk = PivotShooterConstants.motionMagicJerk;
    motorConfig.CurrentLimits.StatorCurrentLimitEnable = PivotShooterConstants.enableStatorLimit;
    motorConfig.CurrentLimits.StatorCurrentLimit = PivotShooterConstants.statorLimit;
    TalonUtil.applyAndCheckConfiguration(pivotShooterMotor, motorConfig);

    BaseStatusSignal.setUpdateFrequencyForAll(
        PivotShooterConstants.updateFrequency,
        pivotShooterMotorVoltage,
        pivotShooterMotorPosition,
        pivotShooterMotorStatorCurrent,
        pivotShooterMotorSupplyCurrent,
        pivotShooterMotorTemperature);
    pivotShooterMotor.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(PivotShooterIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        pivotShooterMotorVoltage,
        pivotShooterMotorPosition,
        pivotShooterMotorStatorCurrent,
        pivotShooterMotorSupplyCurrent,
        pivotShooterMotorTemperature);
    inputs.pivotShooterMotorVoltage = pivotShooterMotorVoltage.getValueAsDouble();
    inputs.pivotShooterMotorPosition = pivotShooterMotorPosition.getValueAsDouble();
    inputs.pivotShooterMotorDegrees =
        (inputs.pivotShooterMotorPosition / PivotShooterConstants.kPivotMotorGearing) * 360;
    inputs.pivotShooterMotorStatorCurrent = pivotShooterMotorStatorCurrent.getValueAsDouble();
    inputs.pivotShooterMotorSupplyCurrent = pivotShooterMotorSupplyCurrent.getValueAsDouble();
    inputs.pivotShooterMotorTemperature = pivotShooterMotorTemperature.getValueAsDouble();
    inputs.lastLastCenterLimelightTY = inputs.lastCenterLimelightTY;
    inputs.lastCenterLimelightTY = inputs.currentCenterLimelightTY;
    inputs.centerLimelightTYOffset = (inputs.lastCenterLimelightTY - inputs.lastLastCenterLimelightTY) + inputs.currentCenterLimelightTY;
    inputs.currentCenterLimelightTY = LimelightHelpers.getTY("limelight");
    inputs.interpolatedPivotPosition = aprilTagMap.get(inputs.centerLimelightTYOffset);
  }

  @Override
  public void setPosition(double position) {
    if (PivotShooterConstants.kUseMotionMagic) {
      pivotShooterMotor.setControl(motionMagicRequest.withPosition(position));
    } else {
      pivotShooterMotor.setControl(positionRequest.withPosition(position));
    }
  }

  @Override
  public void setVoltage(double voltage) {
    pivotShooterMotor.setVoltage(voltage);
  }

  @Override
  public void off() {
    pivotShooterMotor.setControl(new NeutralOut());
  }

  @Override
  public void zero() {
    pivotShooterMotor.setPosition(0);
  }
}
