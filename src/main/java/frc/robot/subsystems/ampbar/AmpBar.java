// Copyright (c) 2024 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.ampbar;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.ampbar.AmpBarIO.AmpBarIOInputs;

public class AmpBar extends SubsystemBase {

  private final AmpBarIO ampBarIO;
  private final AmpBarIOInputsAutoLogged ampBarIOAutoLogged = new AmpBarIOInputsAutoLogged();

  public AmpBar(AmpBarIO ampBarIO) {
    this.ampBarIO = ampBarIO;
  }

  @Override
  public void periodic() {
    ampBarIO.updateInputs(ampBarIOAutoLogged);
    Logger.processInputs(getName(), ampBarIOAutoLogged);
  }

  public Command setVoltage(double voltage) {
    return new StartEndCommand(
        () -> ampBarIO.setVoltage(voltage), () -> ampBarIO.off(), this);
  }

  public Command setAmpPosition() {
    return new Command() {
      @Override
      public void initialize() {
        ampBarIO.setVoltage(AmpBarConstants.kAmpBarAmpVoltage);
      }

      @Override
      public void end(boolean interrupted) {
        ampBarIO.off();
      }

      @Override
      public boolean isFinished() {
        return ampBarIO.isCurrentSpiking();
      }
    };
  }
  public Command setStowPosition() {
    return new Command() {
      @Override
      public void initialize() {
        ampBarIO.setVoltage(AmpBarConstants.kAmpBarStowVoltage);
      }

      @Override
      public void end(boolean interrupted) {
        ampBarIO.off();
      }

      @Override
      public boolean isFinished() {
        return ampBarIO.isCurrentSpiking();
      }
    };
  }

  public Command off() {
    return new StartEndCommand(() -> ampBarIO.off(), () -> {}, this);
  }
}
