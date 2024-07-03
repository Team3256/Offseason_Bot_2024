// Copyright (c) 2024 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.pivotshooter;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.subsystems.vision.Vision;
import org.littletonrobotics.junction.Logger;

public class PivotShooter extends SubsystemBase {

  private final PivotShooterIO pivotShooterIO;
  private final PivotShooterIOInputsAutoLogged pivotShooterIOAutoLogged =
      new PivotShooterIOInputsAutoLogged();

  private final InterpolatingDoubleTreeMap aprilTagMap =
      new InterpolatingDoubleTreeMap() {
        {
          put(0.0, 0.0);
          put(1.0, 1.0);
        }
      };

  public PivotShooter(PivotShooterIO pivotShooterIO) {
    this.pivotShooterIO = pivotShooterIO;
  }

  @Override
  public void periodic() {
    pivotShooterIO.updateInputs(pivotShooterIOAutoLogged);
    Logger.processInputs(this.getClass().getName(), pivotShooterIOAutoLogged);
  }

  public Command setPosition(double position) {
    return this.run(() -> pivotShooterIO.setPosition(position));
  }

  public Command setVoltage(double voltage) {
    return this.run(() -> pivotShooterIO.setVoltage(voltage)).finallyDo(pivotShooterIO::off);
  }

  public Command off() {

    return this.runOnce(pivotShooterIO::off);
  }

  public Command slamZero() {
    return this.run(() -> pivotShooterIO.setVoltage(PivotShooterConstants.kPivotSlamShooterVoltage))
        .until(
            () ->
                pivotShooterIOAutoLogged.pivotShooterMotorStatorCurrent
                    > PivotShooterConstants.kPivotSlamStallCurrent)
        .andThen(this.zero());
  }

  public Command slamAndPID() {

    return Commands.sequence(this.setPosition(0), this.slamZero());
  }

  public Command zero() {
    return this.runOnce(pivotShooterIO::zero);
  }

  public Command bruh(Vision vision) {
    return this.run(
        () -> {
          pivotShooterIO.setPosition(
              aprilTagMap.get(
                      (vision.getLastCenterLimelightY() - vision.getLastLastCenterLimelightY())
                          + vision.getCenterLimelightY())
                  * PivotShooterConstants.kPivotMotorGearing);
        });
  }
}
