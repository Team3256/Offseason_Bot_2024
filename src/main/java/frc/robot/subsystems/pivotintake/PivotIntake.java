// Copyright (c) 2024 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.pivotintake;

import edu.wpi.first.wpilibj2.command.*;
import org.littletonrobotics.junction.Logger;

public class PivotIntake extends SubsystemBase {

  private final PivotIntakeIO pivotIntakeIO;
  private final PivotIntakeIOInputsAutoLogged pivotIntakeIOAutoLogged =
      new PivotIntakeIOInputsAutoLogged();

  public PivotIntake(PivotIntakeIO pivotIntakeIO) {
    this.pivotIntakeIO = pivotIntakeIO;
  }

  @Override
  public void periodic() {
    pivotIntakeIO.updateInputs(pivotIntakeIOAutoLogged);
    Logger.processInputs(this.getClass().getName(), pivotIntakeIOAutoLogged);
  }

  public Command setPosition(double position) {
    return this.run(() -> pivotIntakeIO.setPosition(position));
  }

  public Command setVoltage(double voltage) {
    return this.run(() -> pivotIntakeIO.setVoltage(voltage)).finallyDo(pivotIntakeIO::off);
  }

  public Command off() {
    return this.runOnce(pivotIntakeIO::off);
  }

  public Command slamZero() {
    return this.run(()->pivotIntakeIO.setVoltage(PivotIntakeConstants.kPivotSlamShooterVoltage)).until(()->pivotIntakeIOAutoLogged.pivotIntakeMotorStatorCurrent > PivotIntakeConstants.kPivotSlamStallCurrent).andThen(this.zero());
  }

  public Command slamAndPID() {

    return Commands.sequence(this.setPosition(0), this.slamZero());
  }

  public Command zero() {
    return this.runOnce(pivotIntakeIO::zero);
  }
}
