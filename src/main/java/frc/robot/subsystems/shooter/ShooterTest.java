package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLogOutput;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.Constants;
import frc.robot.utils.DualVelocitySubsystem;

public class ShooterTest extends DualVelocitySubsystem {
    public ShooterTest() {
        super(ShooterConstants.kUseShooterMotionMagic, ShooterConstants.kUseShooterMotionMagic, 69.0, 69.0, 69.0, 69.0);
        super.configureRealHardware(ShooterConstants.kShooterMotorID, ShooterConstants.kShooterMotorFollowerID,
                NeutralModeValue.Coast, NeutralModeValue.Brake, ShooterConstants.kShooterKS,
                ShooterConstants.kShooterKV, ShooterConstants.kShooterKP, ShooterConstants.kShooterKI,
                ShooterConstants.kShooterKD, ShooterConstants.kShooterKS, ShooterConstants.kShooterKV,
                ShooterConstants.kShooterKP, ShooterConstants.kShooterKI, ShooterConstants.kShooterKD,
                InvertedValue.CounterClockwise_Positive, InvertedValue.Clockwise_Positive);
        if (!RobotBase.isReal()) {
            super.configureSimHardware();
        }
    }

    public void setInputShooterVoltage(double voltage) {
        super.setOutputVoltageUno(voltage);
        if (Constants.FeatureFlags.kDebugEnabled) {
            System.out.println("Shooter voltage set to: " + voltage);
        }
    }

    public void setInputShooterFollowerVoltage(double voltage) {
        super.setOutputVoltageDos(voltage);
        if (Constants.FeatureFlags.kDebugEnabled) {
            System.out.println("Shooter follower voltage set to: " + voltage);
        }
    }

    public void setShooterRps(double shooterRps) {
        super.setUnoVelocity(shooterRps);
        if (Constants.FeatureFlags.kDebugEnabled) {
            System.out.println("Shooter velocity set to: " + shooterRps);
        }
    }

    public void setShooterFollowerRps(double shooterRps) {
        super.setDosVelocity(shooterRps);
        if (Constants.FeatureFlags.kDebugEnabled) {
            System.out.println("Shooter velocity set to: " + shooterRps);
        }
    }

    public void off() {
        super.off();
        if (Constants.FeatureFlags.kDebugEnabled) {
            System.out.println("Shooter off");
        }
    }

    @AutoLogOutput
    public boolean getOff() {
        return super.getOff();
    }

    // getters
    @AutoLogOutput
    public double getShooterRps() {
        return super.getUnoVelocity();
    }

    @AutoLogOutput
    public double getShooterCurrent() {
        return super.getUnoCurrent();
    }

    @AutoLogOutput
    public double getShooterFollowerCurrent() {
        return super.getDosCurrent();
    }

    @AutoLogOutput
    public double getShooterMotorVoltage() {
        return super.getUnoMotorVoltage();
    }

    // @Log.Graph(name = "Shooter Motor Stator Voltage")
    // @AutoLogOutput
    // public double getShooterStatorVoltage() {
    // return shooterMotor.getStatorCurrent().getValue();
    // }

    // @Log.Graph(name = "Feeder Motor Rps"

    /* workers */

    // @Log.Graph(name = "Feeder Stator Voltage")

    // @Log.Graph(name = "Follower Motor Rps")
    @AutoLogOutput
    public double getShooterFollowerRps() {
        return super.getDosVelocity();
    }

    // @Log.Graph(name = "Follower Motor Voltage")
    @AutoLogOutput
    public double getShooterMotorFollowerVoltage() {
        return super.getDosMotorVoltage();
    }

    // @Log.Graph(name = "Follower Stator Voltage")
    @AutoLogOutput
    public double getShooterFollowerSupplyVoltage() {
        return super.getDosCurrent();
    }

    // workers

}
