// Copyright (c) 2024 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.pivotintake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
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
    Logger.processInputs(getName(), pivotIntakeIOAutoLogged);
  }
}
