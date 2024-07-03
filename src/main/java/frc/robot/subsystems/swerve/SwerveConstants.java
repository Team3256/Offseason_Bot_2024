package frc.robot.subsystems.swerve;

import com.ctre.phoenix6.mechanisms.swerve.utility.PhoenixPIDController;
import com.pathplanner.lib.util.PIDConstants;

public final class SwerveConstants {
    public static final PhoenixPIDController azimuthController = new PhoenixPIDController(1,0,0); // you shouldn't have a I or D value TODO: tune
    public static final PIDConstants autoRotationalController = new PIDConstants(azimuthController.getP(), azimuthController.getI(), azimuthController.getD()); // should be the same
    public static final PIDConstants autoTranslationalController = new PIDConstants(1,0,0); // you shouldn't have a I or D value TODO: tune

}
