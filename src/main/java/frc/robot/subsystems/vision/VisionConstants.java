// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems.vision;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;

public class VisionConstants {
  // AprilTag layout
  public static AprilTagFieldLayout aprilTagLayout =
      AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

  // Camera names, must match names configured on coprocessor
  public static String camera0Name = "limelight";
  public static String camera1Name = "camera_1";

  // Robot to camera transforms
  // (Not used by Limelight, configure in web UI instead)

  // from Cybercats code reefscape:
  // private final LimeLight limeLightFront = new LimeLight("limelight-front", 0.037, -0.236, 0.565,
  // -91.5, -1, 2.5);
  // private final LimeLight limeLightFrontRight = new LimeLight("limelight-right", .127, 0.029,
  // 0.395, 5.8, -21, 31.2);

  //   front-left camera
  public static Transform3d robotToCamera0 =
      //   new Transform3d(0.2, 0.0, 0.2, new Rotation3d(0.0, -0.4, 0.0));
      new Transform3d(
          0.37,
          -0.236,
          0.565,
          new Rotation3d(
              Radians.convertFrom(-91.5, Degrees),
              Radians.convertFrom(-1, Degrees),
              Radians.convertFrom(2.5, Degrees)));

  // front-right camera
  public static Transform3d robotToCamera1 =
      //   new Transform3d(-0.2, 0.0, 0.2, new Rotation3d(0.0, -0.4, Math.PI));
      new Transform3d(
          0.127,
          0.029,
          0.395,
          new Rotation3d(
              Radians.convertFrom(5.8, Degrees),
              Radians.convertFrom(-21, Degrees),
              Radians.convertFrom(31.2, Degrees)));

  // Basic filtering thresholds
  public static double maxAmbiguity = 0.3;
  public static double maxZError = 0.75;

  // Standard deviation baselines, for 1 meter distance and 1 tag
  // (Adjusted automatically based on distance and # of tags)
  public static double linearStdDevBaseline = 0.02; // Meters
  public static double angularStdDevBaseline = 0.06; // Radians

  // Standard deviation multipliers for each camera
  // (Adjust to trust some cameras more than others)
  public static double[] cameraStdDevFactors =
      new double[] {
        1.0, // Camera 0
        1.0 // Camera 1
      };

  // Multipliers to apply for MegaTag 2 observations
  public static double linearStdDevMegatag2Factor = 0.5; // More stable than full 3D solve
  public static double angularStdDevMegatag2Factor =
      Double.POSITIVE_INFINITY; // No rotation data available
}
