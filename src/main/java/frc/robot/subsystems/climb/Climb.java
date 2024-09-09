// Copyright (c) 2024 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.climb;

import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.utils.DisableSubsystem;
import org.littletonrobotics.junction.Logger;

public class Climb extends DisableSubsystem {

  private final ClimbIO climbIO;
  private final ClimbIOInputsAutoLogged climbIOAutoLogged = new ClimbIOInputsAutoLogged();
  private final SysIdRoutine m_sysIdRoutine;

  public Climb(boolean disabled, ClimbIO climbIO) {
    super(disabled);
    this.climbIO = climbIO;
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
                    climbIO
                        .getMotor()
                        .setControl(climbIO.getVoltageRequest().withOutput(volts.in(Volts))),
                null,
                this));
  }

  @Override
  public void periodic() {
    super.periodic();
    climbIO.updateInputs(climbIOAutoLogged);
    Logger.processInputs(this.getClass().getSimpleName(), climbIOAutoLogged);
  }

  public Command setPosition(double position) {
    return this.run(() -> climbIO.setPosition(position * ClimbConstants.gearRatio));
  }

  public Command setVoltage(double voltage) {
    return this.run(() -> climbIO.setVoltage(voltage)).finallyDo(climbIO::off);
  }

  public Command off() {

    return this.runOnce(climbIO::off);
  }

  public Command zero() {

    return this.runOnce(climbIO::zero);
  }

  public Command extendClimber() {
    return setPosition(ClimbConstants.kClimbUpPosition);
  }

  public Command retractClimber() {
    return setPosition(ClimbConstants.kClimbDownPosition);
  }

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutine.quasistatic(direction);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutine.dynamic(direction);
  }
}
