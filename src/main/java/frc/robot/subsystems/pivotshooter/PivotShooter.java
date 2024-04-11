// Copyright (c) 2024 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.pivotshooter;

import static frc.robot.subsystems.pivotshooter.PivotingShooterConstants.kSubWooferPreset;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.drivers.MonitoredTalonFX;
import frc.robot.limelight.Limelight;
import frc.robot.utils.PhoenixUtil;
import frc.robot.utils.TalonUtil;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Config;
import io.github.oblarg.oblog.annotations.Log;
import java.util.Optional;
import org.littletonrobotics.junction.AutoLogOutput;

public class PivotShooter extends SubsystemBase implements Loggable {

  private MonitoredTalonFX pivotMotor;
  private TalonFXSimState pivotMotorSim;
  private SingleJointedArmSim pivotModel;

  public InterpolatingDoubleTreeMap pivotMotorData = new InterpolatingDoubleTreeMap();
  public InterpolatingDoubleTreeMap pivotMotorDataNotGlobalPose = new InterpolatingDoubleTreeMap();
  public InterpolatingDoubleTreeMap pivotMotorDataNotGlobalPose2 =
      new InterpolatingDoubleTreeMap(); // for second april
  // tag

  final PositionVoltage positionRequest =
      new PositionVoltage(0).withEnableFOC(PivotingShooterConstants.kUseFOC).withSlot(0);
  final MotionMagicVoltage motionMagicRequest =
      new MotionMagicVoltage(0).withEnableFOC(PivotingShooterConstants.kUseFOC).withSlot(0);

  public void DEFAULT() {}

  public PivotShooter() {
    if (RobotBase.isReal()) {
      configureRealHardware();
    } else {
      configureSimHardware();
    }
  }

  private void configureRealHardware() {
    pivotMotor = new MonitoredTalonFX(PivotingShooterConstants.kPivotMotorID);
    pivotMotor.setNeutralMode(NeutralModeValue.Brake);
    pivotMotor.setInverted(true);

    setMotorConfigs(
        PivotingShooterConstants.kS,
        PivotingShooterConstants.kV,
        PivotingShooterConstants.kP,
        PivotingShooterConstants.kI,
        PivotingShooterConstants.kD,
        PivotingShooterConstants.motionMagicVelocity,
        PivotingShooterConstants.motionMagicAcceleration,
        PivotingShooterConstants.motionMagicJerk,
        PivotingShooterConstants.enableStatorLimit,
        PivotingShooterConstants.statorLimit);
    setupPivotShooterData();
  }

  private void setupPivotShooterData() {

    pivotMotorData.put(0.051, kSubWooferPreset);
    pivotMotorData.put(1.4, 6.25 / 138.333);
    pivotMotorData.put(0.945, 5.8 / 138.333);
    pivotMotorData.put(1.69, 6.5 / 138.333);
    pivotMotorData.put(0.485, 5.25 / 138.333);
    //    pivotMotorData.put(3.19, 5.6/138.333); // DO NOT use to interpolate for now
    // distance to speaker and then angle
  }

  private void setupPivotShooterDataNotGlobalPose() {
    pivotMotorDataNotGlobalPose.put(69.2, 2.1); // distance to speaker and then angle
    pivotMotorDataNotGlobalPose2.put(69.1, 69.2); // distance to speaker and then angle
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

  private void setStatorLimits(boolean enableStatorLimit, double statorLimit) {
    var pivotMotorConfigs = new TalonFXConfiguration();
    pivotMotorConfigs.CurrentLimits.StatorCurrentLimitEnable = enableStatorLimit;
    pivotMotorConfigs.CurrentLimits.StatorCurrentLimit = statorLimit;
    pivotMotor.getConfigurator().apply(pivotMotorConfigs);
  }

  private void configureSimHardware() {
    configureRealHardware();
    pivotMotorSim = pivotMotor.getSimState();
    pivotMotorSim.setSupplyVoltage(12);
    pivotModel =
        new SingleJointedArmSim(
            DCMotor.getFalcon500(PivotingShooterConstants.kNumPivotMotors),
            PivotingShooterConstants.kPivotMotorGearing,
            PivotingShooterConstants.jKgMetersSquared,
            PivotingShooterConstants.kPivotLength,
            Units.degreesToRadians(PivotingShooterConstants.kPivotMinAngleDeg),
            Units.degreesToRadians(PivotingShooterConstants.kPivotMaxAngleDeg),
            false,
            PivotingShooterConstants.kPivotMinAngleDeg);
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
    if (PivotingShooterConstants.kUseMotionMagic) {
      pivotMotor.setControl(motionMagicRequest.withPosition(degrees));
    } else {
      pivotMotor.setControl(positionRequest.withPosition(degrees));
    }
  }

  @Log.Graph(name = "PivotShooter Voltage")
  @AutoLogOutput
  public double getVoltage() {
    return pivotMotor.getMotorVoltage().getValue();
  }

  @Log.Graph(name = "PivotShooter Velocity")
  @AutoLogOutput
  public double getVelocity() {
    return pivotMotor.getRotorVelocity().getValue();
  }

  @Log.Graph(name = "PivotShooter Position")
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
    if (Constants.FeatureFlags.kDebugEnabled) {
      System.out.println("[PivotShooter] Setting zero position to: " + getDegrees());
    }
    pivotMotor.setPosition(0);
    if (Constants.FeatureFlags.kDebugEnabled) {
      System.out.println("[PivotShooter] Zero position set. Current position: " + getDegrees());
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
    return pivotMotor.getStatorCurrent().getValue();
  }

  @Log
  @AutoLogOutput
  public boolean isCurrentSpiking() {
    return this.getCurrent() > PivotingShooterConstants.kCurrentThreshold;
  }

  @AutoLogOutput
  public boolean isSpeakerTag() {
    int speakerId;
    Optional<DriverStation.Alliance> ally = DriverStation.getAlliance();
    if (ally.isPresent() && ally.get() == DriverStation.Alliance.Red) {
      speakerId = PivotingShooterConstants.kSpeakerAprilTagRed;
    } else if (ally.isPresent()) {
      speakerId = PivotingShooterConstants.kSpeakerAprilTagBlue;
    } else {
      speakerId = PivotingShooterConstants.kSpeakerAprilTagRed;
    }
    return (Limelight.getFiducialID("limelight") == speakerId);
  }

  @Log
  @AutoLogOutput
  public boolean isMotorStalled() {
    return this.isCurrentSpiking()
        && this.pivotMotor.getRotorVelocity().getValue()
            < PivotingShooterConstants.kStallVelocityThreshold;
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
