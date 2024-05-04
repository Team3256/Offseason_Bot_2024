package frc.robot.subsystems.ampbar;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;

import frc.robot.drivers.MonitoredTalonFX;
import frc.robot.utils.PhoenixUtil;
import frc.robot.utils.TalonUtil;

public class AmpBarIOTalonFX implements AmpBarIO {
    

    private final MonitoredTalonFX ampBarMotor = new MonitoredTalonFX(AmpBarConstants.kAmpBarMotorID);

    private final StatusSignal<Double> ampBarMotorVoltage = ampBarMotor.getMotorVoltage();
    private final StatusSignal<Double> ampBarMotorVelocity = ampBarMotor.getVelocity();
    private final StatusSignal<Double> ampBarMotorPosition = ampBarMotor.getPosition();
    private final StatusSignal<Double> ampBarMotorStatorCurrent = ampBarMotor.getStatorCurrent();
    private final StatusSignal<Double> ampBarMotorSupplyCurrent = ampBarMotor.getSupplyCurrent();
    private final StatusSignal<Double> ampBarMotorTemperature = ampBarMotor.getDeviceTemp();

    public AmpBarIOTalonFX() {
        var motorConfig = new TalonFXConfiguration();
        PhoenixUtil.checkErrorAndRetry(() -> ampBarMotor.getConfigurator().refresh(motorConfig));
        motorConfig.MotorOutput.NeutralMode = AmpBarConstants.neutralMode;
        motorConfig.MotorOutput.Inverted = AmpBarConstants.ampBarInverted;
        motorConfig.CurrentLimits.StatorCurrentLimitEnable = AmpBarConstants.enableStatorLimit;
        motorConfig.CurrentLimits.StatorCurrentLimit = AmpBarConstants.statorLimit;
        TalonUtil.applyAndCheckConfiguration(ampBarMotor, motorConfig);

        BaseStatusSignal.setUpdateFrequencyForAll(
            AmpBarConstants.updateFrequency,
            ampBarMotorVoltage,
            ampBarMotorVelocity,
            ampBarMotorPosition,
            ampBarMotorStatorCurrent,
            ampBarMotorSupplyCurrent,
            ampBarMotorTemperature);
        ampBarMotor.optimizeBusUtilization();
    }

    @Override
    public void updateInputs(AmpBarIOInputs inputs) {
        BaseStatusSignal.refreshAll( ampBarMotorVoltage, ampBarMotorVelocity, ampBarMotorPosition,
                ampBarMotorStatorCurrent, ampBarMotorSupplyCurrent, ampBarMotorTemperature);
        inputs.ampBarMotorVoltage = ampBarMotorVoltage.getValueAsDouble();
        inputs.ampBarMotorVelocity = ampBarMotorVelocity.getValueAsDouble();
        inputs.ampBarMotorPosition = ampBarMotorPosition.getValueAsDouble();
        inputs.ampBarMotorStatorCurrent = ampBarMotorStatorCurrent.getValueAsDouble();
        inputs.ampBarMotorSupplyCurrent = ampBarMotorSupplyCurrent.getValueAsDouble();
        inputs.ampBarMotorTemperature = ampBarMotorTemperature.getValueAsDouble();
    }

    @Override
    public void setVoltage(double voltage) {
        ampBarMotor.setVoltage(voltage);
    }

    @Override
    public void off() {
        ampBarMotor.setControl(new NeutralOut());
    }

    @Override
    public boolean isCurrentSpiking() {
        return ampBarMotor.getStatorCurrent().getValueAsDouble() > AmpBarConstants.kAmpBarCurrentThreshold;
    }
}
