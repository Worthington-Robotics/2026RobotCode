// Copyright (c) 2024 FRC 4145
// https://github.com/Worthington-Robotics
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.WorBots.subsystems.vision.apriltags;

import edu.wpi.first.math.geometry.Pose3d;

public interface TagVisionIO {
    public static class TagVisionIOInputs {
        public double[][] frames = new double[][] {};
        public double timestamps[] = new double[] {};
        public Pose3d[] covariances = new Pose3d[] {};
        public double fps = 0.0;
        public boolean isConnected = false;
    }

  /** Modes for filtering tag detections */
  public static enum DetectionMode {
    // Detect all tags
    MultiTag,
    // Not currently implemented in the tag vision code
    LargestTag,
    // Detect only a specific tag, specified somewhere else
    SpecificTag,
  }

  /**
   * Updates the inputs of the tag vision subsystem.
   *
   * @param inputs The inputs to be updated.
   */
  public default void updateInputs(TagVisionIOInputs inputs) {}

  /**
   * Sets the detection configuration for the tag vision
   *
   * @param mode The detection mode for filtering tags
   * @param specificTag The specific tag for specific tag mode, can be -1 if not using that mode
   */
  public default void setDetectionConfig(DetectionMode mode, int specificTag) {}    


}
