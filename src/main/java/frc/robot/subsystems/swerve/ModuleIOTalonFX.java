package frc.robot.subsystems.swerve;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import frc.robot.Robot;
import frc.robot.drivers.MonitoredTalonFX;
import frc.robot.helpers.Conversions;
import frc.robot.subsystems.swerve.helpers.SwerveModuleConstants;
import frc.robot.utils.TalonUtil;

public class ModuleIOTalonFX implements ModuleIO {
    public int moduleNumber;
    private Rotation2d angleOffset;

    private MonitoredTalonFX mAngleMotor;
    private MonitoredTalonFX mDriveMotor;
    private CANcoder angleEncoder;

    private final SimpleMotorFeedforward driveFeedForward =
        new SimpleMotorFeedforward(
            SwerveConstants.driveKS, SwerveConstants.driveKV, SwerveConstants.driveKA);

    private final DutyCycleOut driveDutyCycle = new DutyCycleOut(0).withEnableFOC(true);
    private final VelocityVoltage driveVelocity = new VelocityVoltage(0).withEnableFOC(true);

    private final PositionVoltage anglePosition = new PositionVoltage(0).withEnableFOC(true);

    private final StatusSignal<Double> angleMotorVoltage;
    private final StatusSignal<Double> angleMotorPosition;
    private final StatusSignal<Double> angleMotorStatorCurrent ;
    private final StatusSignal<Double> angleMotorSupplyCurrent ;
    private final StatusSignal<Double> angleMotorTemperature;
    private final StatusSignal<Double> angleMotorReferenceSlope ;

    private final StatusSignal<Double> driveMotorVoltage ;
    private final StatusSignal<Double> driveMotorVelocity ;
    private final StatusSignal<Double> driveMotorPosition ;
    private final StatusSignal<Double> driveMotorStatorCurrent ;
    private final StatusSignal<Double> driveMotorSupplyCurrent ;
    private final StatusSignal<Double> driveMotorTemperature ;
    private final StatusSignal<Double> driveMotorReferenceSlope ;

    public ModuleIOTalonFX(int moduleNumber, SwerveModuleConstants moduleConstants) {
        this.moduleNumber = moduleNumber;
        this.angleOffset = moduleConstants.angleOffset;

        angleEncoder = new CANcoder(moduleConstants.cancoderID, "mani");
        angleEncoder.getConfigurator().apply(Robot.ctreConfigs.swerveCANcoderConfig);
        
        mAngleMotor = new MonitoredTalonFX(moduleConstants.angleMotorID, "mani");
        TalonUtil.applyAndCheckConfiguration(mAngleMotor, Robot.ctreConfigs.swerveAngleFXConfig);
        resetToAbsolute();


        mDriveMotor = new MonitoredTalonFX(moduleConstants.driveMotorID, "mani");
        TalonUtil.applyAndCheckConfiguration(mDriveMotor, Robot.ctreConfigs.swerveDriveFXConfig);
        mDriveMotor.getConfigurator().setPosition(0.0);

        angleMotorVoltage = mAngleMotor.getMotorVoltage();
        angleMotorPosition = mAngleMotor.getPosition();
        angleMotorStatorCurrent = mAngleMotor.getStatorCurrent();
        angleMotorSupplyCurrent = mAngleMotor.getSupplyCurrent();
        angleMotorTemperature = mAngleMotor.getDeviceTemp();
        angleMotorReferenceSlope = mAngleMotor.getClosedLoopReferenceSlope();

        driveMotorVoltage = mDriveMotor.getMotorVoltage();
        driveMotorVelocity = mDriveMotor.getVelocity();
        driveMotorPosition = mDriveMotor.getPosition();
        driveMotorStatorCurrent = mDriveMotor.getStatorCurrent(); 
        driveMotorSupplyCurrent = mDriveMotor.getSupplyCurrent();
        driveMotorTemperature = mDriveMotor.getDeviceTemp();
        driveMotorReferenceSlope = mDriveMotor.getClosedLoopReferenceSlope();

        BaseStatusSignal.setUpdateFrequencyForAll(SwerveConstants.updateFrequency, 
            angleMotorVoltage, 
            angleMotorPosition, 
            angleMotorStatorCurrent, 
            angleMotorSupplyCurrent, 
            angleMotorTemperature, 
            angleMotorReferenceSlope,
            driveMotorVoltage, 
            driveMotorVelocity, 
            driveMotorPosition, 
            driveMotorStatorCurrent, 
            driveMotorSupplyCurrent, 
            driveMotorTemperature, 
            driveMotorReferenceSlope);

        mAngleMotor.optimizeBusUtilization();
        mDriveMotor.optimizeBusUtilization();
    }

    @Override
    public void updateInputs(ModuleIOInputs inputs) {

        BaseStatusSignal.refreshAll(angleMotorVoltage, angleMotorPosition, angleMotorStatorCurrent, angleMotorSupplyCurrent, angleMotorTemperature, angleMotorReferenceSlope, driveMotorVoltage, driveMotorVelocity, driveMotorPosition, driveMotorStatorCurrent, driveMotorSupplyCurrent, driveMotorTemperature, driveMotorReferenceSlope);
        inputs.angleMotorVoltage = angleMotorVoltage.getValueAsDouble();
        inputs.angleMotorPosition = angleMotorPosition.getValueAsDouble();
        inputs.angleMotorStatorCurrent = angleMotorStatorCurrent.getValueAsDouble();
        inputs.angleMotorSupplyCurrent = angleMotorSupplyCurrent.getValueAsDouble();
        inputs.angleMotorTemperature = angleMotorTemperature.getValueAsDouble();
        inputs.angleMotorReferenceSlope = angleMotorReferenceSlope.getValueAsDouble();

        inputs.driveMotorVoltage = driveMotorVoltage.getValueAsDouble();
        inputs.driveMotorVelocity = driveMotorVelocity.getValueAsDouble();
        inputs.driveMotorPosition = driveMotorPosition.getValueAsDouble();
        inputs.driveMotorStatorCurrent = driveMotorStatorCurrent.getValueAsDouble();
        inputs.driveMotorSupplyCurrent = driveMotorSupplyCurrent.getValueAsDouble();
        inputs.driveMotorTemperature = driveMotorTemperature.getValueAsDouble();
        inputs.driveMotorReferenceSlope = driveMotorReferenceSlope.getValueAsDouble();

        inputs.absolutePosition = getCANcoder();
        inputs.currentState = getState();
        inputs.currentPosition = getPosition();
    }

    @Override
    public void setDriveVelocity(double velocity, boolean isOpenLoop) { // meters per second
        if (isOpenLoop) {
            driveDutyCycle.Output = velocity / SwerveConstants.maxTranslationalVelocity;
            mDriveMotor.setControl(driveDutyCycle);
        } else {
            driveVelocity.Velocity =
                Conversions.MPSToRPS(
                    velocity, SwerveConstants.wheelCircumference);
            driveVelocity.FeedForward = driveFeedForward.calculate(velocity);
            mDriveMotor.setControl(driveVelocity);
        }
    }

    @Override
    public void setAnglePosition(double position) {
        anglePosition.Position = position;
        mAngleMotor.setControl(anglePosition);
    }
    @Override
    public Rotation2d getCANcoder() {
        return Rotation2d.fromDegrees(angleEncoder.getAbsolutePosition().getValue());
    
    }

    @Override
    public void resetToAbsolute() {
        double absolutePosition = getCANcoder().getRotations() - angleOffset.getRotations();
        mAngleMotor.setPosition(absolutePosition);
    }

    @Override
    public SwerveModuleState getState() {
    return new SwerveModuleState(
        Conversions.RPSToMPS(
            mDriveMotor.getVelocity().getValue(), SwerveConstants.wheelCircumference),
        Rotation2d.fromRotations(mAngleMotor.getPosition().getValue()));
    }

    @Override
    public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(
        Conversions.rotationsToMeters(
            mDriveMotor.getPosition().getValue(), SwerveConstants.wheelCircumference),
        Rotation2d.fromRotations(mAngleMotor.getPosition().getValue()));
  }

  

}
