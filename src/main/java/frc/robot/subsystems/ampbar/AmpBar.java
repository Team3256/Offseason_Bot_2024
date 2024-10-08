// Copyright (c) 2024 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.ampbar;

import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.utils.DisableSubsystem;
import org.littletonrobotics.junction.Logger;

public class AmpBar extends DisableSubsystem {

  private final AmpBarIO ampBarIO;
  private final AmpBarIOInputsAutoLogged ampBarIOAutoLogged = new AmpBarIOInputsAutoLogged();

  private final SysIdRoutine m_sysIdRoutine;

  public AmpBar(boolean disabled, AmpBarIO ampBarIO) {
    super(disabled);
    this.ampBarIO = ampBarIO;
    m_sysIdRoutine =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                Volts.of(0.2).per(Seconds.of(1)), // Use default ramp rate (1 V/s)
                Volts.of(6), // Reduce dynamic step voltage to 4 to prevent brownout
                null, // Use default timeout (10 s)
                // Log state with Phoenix SignalLogger class
                (state) -> SignalLogger.writeString("state", state.toString())),
            new SysIdRoutine.Mechanism(
                (volts) ->
                    ampBarIO
                        .getMotor()
                        .setControl(ampBarIO.getVoltageRequest().withOutput(volts.in(Volts))),
                null,
                this));
  }

  @Override
  public void periodic() {
    super.periodic();
    ampBarIO.updateInputs(ampBarIOAutoLogged);
    Logger.processInputs(this.getClass().getSimpleName(), ampBarIOAutoLogged);
  }

  public Command setVoltage(double voltage) {
    return this.run(() -> ampBarIO.setVoltage(voltage)).finallyDo(ampBarIO::off);
  }

  public Command setAmpPosition() {
    return setPosition(AmpBarConstants.kAmpBarAmpPosition * AmpBarConstants.kAmpBarGearing);
  }

  public Command setStowPosition() {
    return setPosition(AmpBarConstants.kAmpBarStowPosition * AmpBarConstants.kAmpBarGearing);
  }

  public Command setPosition(double position) {
    return this.run(() -> ampBarIO.setPosition(position)).finallyDo(ampBarIO::off);
  }

  public Command off() {

    return this.runOnce(ampBarIO::off);
  }
}
