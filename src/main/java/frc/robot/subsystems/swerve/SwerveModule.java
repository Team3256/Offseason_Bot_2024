// Copyright (c) 2024 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.helpers.Conversions;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class SwerveModule {

  private final ModuleIO io;
  private final ModuleIOInputsAutoLogged inputs = new ModuleIOInputsAutoLogged();
  public final int moduleNumber;

  public SwerveModule(ModuleIO io, int index) {

    this.io = io;
    this.moduleNumber = index;
  }

  public void updateInputs() {
    io.updateInputs(inputs);
  }

  public void periodic() {
    Logger.processInputs("Swerve/Module" + Integer.toString(moduleNumber), inputs);
  }

  public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {
    desiredState = SwerveModuleState.optimize(desiredState, getState().angle);
    io.setAnglePosition(desiredState.angle.getRotations());
    setSpeed(desiredState, isOpenLoop);
  }

  private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop) {
    io.setDriveVelocity(desiredState.speedMetersPerSecond, isOpenLoop);
  }

  @AutoLogOutput
  public SwerveModuleState getState() {
    return new SwerveModuleState(
        Conversions.RPSToMPS(inputs.driveMotorVelocity, SwerveConstants.wheelCircumference),
        Rotation2d.fromRotations(inputs.angleMotorPosition));
  }

  @AutoLogOutput
  public SwerveModulePosition getPosition() {

    return new SwerveModulePosition(
        Conversions.rotationsToMeters(
            inputs.driveMotorPosition, SwerveConstants.wheelCircumference),
        Rotation2d.fromRotations(inputs.angleMotorPosition));
  }

  public void resetToAbsolute() {
    io.resetToAbsolute();
  }

  public Rotation2d getCANcoder() {
    return io.getCANcoder();
  }

  public void off() {}
}
