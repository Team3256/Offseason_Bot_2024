package frc.robot.subsystems.vision;

import frc.robot.limelight.Limelight;

public class VisionIOLimelight implements VisionIO {

    public VisionIOLimelight() {
    }
    @Override
    public void updateInputs(VisionIOInputs inputs) {
        inputs.noteLimelightX = Limelight.getTX(VisionConstants.noteDetectionLimelight);
        inputs.noteLimelightY = Limelight.getTY(VisionConstants.noteDetectionLimelight);

        inputs.centerLimelightX = Limelight.getTX(VisionConstants.centerLimelight);
        inputs.centerLimelightY = Limelight.getTY(VisionConstants.centerLimelight);
    }
}
