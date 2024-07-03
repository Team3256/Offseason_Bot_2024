// Copyright (c) 2024 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {

  private final ShooterIO shooterIO;
  private final ShooterIOInputsAutoLogged shooterIOAutoLogged = new ShooterIOInputsAutoLogged();

  private final SysIdRoutine m_sysIdRoutine;

  public Shooter(ShooterIO shooterIO) {
    this.shooterIO = shooterIO;
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
                    shooterIO
                        .getMotor()
                        .setControl(shooterIO.getVoltageRequest().withOutput(volts.in(Volts))),
                null,
                this));
  }

  @Override
  public void periodic() {
    shooterIO.updateInputs(shooterIOAutoLogged);
    Logger.processInputs(this.getClass().getName(), shooterIOAutoLogged);
  }

  public Command setVoltage(double voltage, double followerVoltage) {
    return new StartEndCommand(
        () -> {
          shooterIO.setShooterVoltage(voltage);
          shooterIO.setShooterFollowerVoltage(followerVoltage);
        },
        () -> shooterIO.off(),
        this);
  }

  public Command setVelocity(double velocity, double followerVelocity) {
    return new StartEndCommand(
        () -> {
          shooterIO.setShooterVelocity(velocity);
          shooterIO.setShooterFollowerVelocity(followerVelocity);
        },
        () -> shooterIO.off(),
        this);
  }

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutine.quasistatic(direction);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutine.dynamic(direction);
  }

  public Command off() {
    return new StartEndCommand(() -> shooterIO.off(), () -> {}, this);
  }
}
