package frc.robot.subsystems.swerve;

import static frc.robot.subsystems.swerve.AzimuthConstants.aziSourceRed;

import edu.wpi.first.math.geometry.Rotation2d;

public final class AzimuthConstants {
    /* Angles (raw) */

    // TODO: TUNE ALL OF THESE ON PRAC DAY
    // front of the robot is the intake

    public static final Rotation2d aziAmpRed = Rotation2d.fromDegrees(90);

    public static final Rotation2d aziAmpBlue = Rotation2d.fromDegrees(-aziAmpRed.getDegrees());

    public static final Rotation2d aziSubwooferFront = Rotation2d.fromDegrees(0);

    public static final Rotation2d aziSubwooferLeft = Rotation2d.fromDegrees(-30);

    public static final Rotation2d aziSubwooferRight = Rotation2d.fromDegrees(aziSubwooferLeft.getDegrees());

    public static final Rotation2d aziSourceRed = Rotation2d.fromDegrees(60);

    public static final Rotation2d aziSourceBlue = Rotation2d.fromDegrees(-aziSourceRed.getDegrees());

    public static final Rotation2d feederRed = Rotation2d.fromDegrees(45);

    public static final Rotation2d feederBlue = Rotation2d.fromDegrees(-feederRed.getDegrees());

    public static final Rotation2d cleanUp = Rotation2d.fromDegrees(180);

    /* Timeout */
    public static final double aziCommandTimeOut = 1.5;
}