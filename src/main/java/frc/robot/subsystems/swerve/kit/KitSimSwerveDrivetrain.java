// Copyright (c) 2024 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.swerve.kit;

//
// Source code recreated from a .class file by IntelliJ IDEA
// (powered by FernFlower decompiler)
//

import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.sim.CANcoderSimState;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.Pigeon2SimState;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class KitSimSwerveDrivetrain {
  public final Pigeon2SimState PigeonSim;
  protected final SimSwerveModule[] m_modules;
  protected final int ModuleCount;
  public final SwerveDriveKinematics Kinem;
  public Rotation2d LastAngle = new Rotation2d();

  public KitSimSwerveDrivetrain(
      Translation2d[] wheelLocations,
      Pigeon2 pigeon,
      SwerveDrivetrainConstants driveConstants,
      SwerveModuleConstants... moduleConstants) {
    this.PigeonSim = pigeon.getSimState();
    this.ModuleCount = moduleConstants.length;
    this.m_modules = new SimSwerveModule[this.ModuleCount];

    for (int i = 0; i < this.ModuleCount; ++i) {
      this.m_modules[i] =
          new SimSwerveModule(
              moduleConstants[i].SteerMotorGearRatio,
              moduleConstants[i].SteerInertia,
              moduleConstants[i].SteerFrictionVoltage,
              moduleConstants[i].SteerMotorInverted,
              moduleConstants[i].DriveMotorGearRatio,
              moduleConstants[i].DriveInertia,
              moduleConstants[i].DriveFrictionVoltage,
              moduleConstants[i].DriveMotorInverted);
    }

    this.Kinem = new SwerveDriveKinematics(wheelLocations);
  }

  public void update(double dtSeconds, double supplyVoltage, ModuleIO... modulesToApply) {
    if (this.m_modules.length == this.ModuleCount) {
      SwerveModuleState[] states = new SwerveModuleState[this.ModuleCount];

      for (int i = 0; i < this.ModuleCount; ++i) {
        TalonFXSimState steerMotor = modulesToApply[i].getSteerMotor().getSimState();
        TalonFXSimState driveMotor = modulesToApply[i].getDriveMotor().getSimState();
        CANcoderSimState cancoder = modulesToApply[i].getCANcoder().getSimState();
        steerMotor.Orientation =
            this.m_modules[i].SteerMotorInverted
                ? ChassisReference.Clockwise_Positive
                : ChassisReference.CounterClockwise_Positive;
        driveMotor.Orientation =
            this.m_modules[i].DriveMotorInverted
                ? ChassisReference.Clockwise_Positive
                : ChassisReference.CounterClockwise_Positive;
        steerMotor.setSupplyVoltage(supplyVoltage);
        driveMotor.setSupplyVoltage(supplyVoltage);
        cancoder.setSupplyVoltage(supplyVoltage);
        this.m_modules[i].SteerMotor.setInputVoltage(
            this.addFriction(steerMotor.getMotorVoltage(), this.m_modules[i].SteerFrictionVoltage));
        this.m_modules[i].DriveMotor.setInputVoltage(
            this.addFriction(driveMotor.getMotorVoltage(), this.m_modules[i].DriveFrictionVoltage));
        this.m_modules[i].SteerMotor.update(dtSeconds);
        this.m_modules[i].DriveMotor.update(dtSeconds);
        steerMotor.setRawRotorPosition(
            this.m_modules[i].SteerMotor.getAngularPositionRotations()
                * this.m_modules[i].SteerGearing);
        steerMotor.setRotorVelocity(
            this.m_modules[i].SteerMotor.getAngularVelocityRPM()
                / 60.0
                * this.m_modules[i].SteerGearing);
        cancoder.setRawPosition(this.m_modules[i].SteerMotor.getAngularPositionRotations());
        cancoder.setVelocity(this.m_modules[i].SteerMotor.getAngularVelocityRPM() / 60.0);
        driveMotor.setRawRotorPosition(
            this.m_modules[i].DriveMotor.getAngularPositionRotations()
                * this.m_modules[i].DriveGearing);
        driveMotor.setRotorVelocity(
            this.m_modules[i].DriveMotor.getAngularVelocityRPM()
                / 60.0
                * this.m_modules[i].DriveGearing);
        states[i] = modulesToApply[i].getCurrentState();
      }

      double angleChange = this.Kinem.toChassisSpeeds(states).omegaRadiansPerSecond * dtSeconds;
      this.LastAngle = this.LastAngle.plus(Rotation2d.fromRadians(angleChange));
      this.PigeonSim.setRawYaw(this.LastAngle.getDegrees());
    }
  }

  protected double addFriction(double motorVoltage, double frictionVoltage) {
    if (Math.abs(motorVoltage) < frictionVoltage) {
      motorVoltage = 0.0;
    } else if (motorVoltage > 0.0) {
      motorVoltage -= frictionVoltage;
    } else {
      motorVoltage += frictionVoltage;
    }

    return motorVoltage;
  }

  public class SimSwerveModule {
    public final DCMotorSim SteerMotor;
    public final DCMotorSim DriveMotor;
    public final double SteerGearing;
    public final double DriveGearing;
    public final double SteerFrictionVoltage;
    public final double DriveFrictionVoltage;
    public final boolean SteerMotorInverted;
    public final boolean DriveMotorInverted;

    public SimSwerveModule(
        double steerGearing,
        double steerInertia,
        double steerFrictionVoltage,
        boolean steerMotorInverted,
        double driveGearing,
        double driveInertia,
        double driveFrictionVoltage,
        boolean driveMotorInverted) {
      this.SteerMotor = new DCMotorSim(DCMotor.getFalcon500(1), steerGearing, steerInertia);
      this.DriveMotor = new DCMotorSim(DCMotor.getFalcon500(1), driveGearing, driveInertia);
      this.SteerGearing = steerGearing;
      this.DriveGearing = driveGearing;
      this.SteerFrictionVoltage = steerFrictionVoltage;
      this.DriveFrictionVoltage = driveFrictionVoltage;
      this.SteerMotorInverted = steerMotorInverted;
      this.DriveMotorInverted = driveMotorInverted;
    }
  }
}
