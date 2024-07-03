// Copyright (c) 2024 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.ampbar;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class AmpBar extends SubsystemBase {

  private final AmpBarIO ampBarIO;
  private final AmpBarIOInputsAutoLogged ampBarIOAutoLogged = new AmpBarIOInputsAutoLogged();

  public AmpBar(AmpBarIO ampBarIO) {
    this.ampBarIO = ampBarIO;
  }

  @Override
  public void periodic() {
    ampBarIO.updateInputs(ampBarIOAutoLogged);
    Logger.processInputs(this.getClass().getName(), ampBarIOAutoLogged);
  }

  public Command setVoltage(double voltage) {
    return this.run(() -> ampBarIO.setVoltage(voltage)).finallyDo(ampBarIO::off);
  }

  public Command setAmpPosition() {
    return this.run(() -> ampBarIO.setVoltage(AmpBarConstants.kAmpBarAmpVoltage))
        .until(ampBarIO::isCurrentSpiking)
        .andThen(this.off());
  }

  public Command setStowPosition() {
    return this.run(() -> ampBarIO.setVoltage(AmpBarConstants.kAmpBarStowVoltage))
        .until(ampBarIO::isCurrentSpiking)
        .andThen(this.off());
  }

  public Command off() {

    return this.runOnce(ampBarIO::off);
  }
}
