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

import static frc.robot.subsystems.vision.VisionConstants.*;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.vision.VisionIO.PoseObservationType;
import frc.robot.subsystems.vision.VisionIO.VisionIOInputs;

import java.util.Arrays;
import java.util.LinkedList;
import java.util.List;
import org.littletonrobotics.junction.Logger;

public class Vision extends SubsystemBase {
  private final VisionConsumer consumer;
  private final VisionIO[] io;
  private final VisionIOInputsAutoLogged[] inputs;
  private final Alert[] disconnectedAlerts;
  private final CommandSwerveDrivetrain drivetrain;
  private final Object lock = new Object();

  // Data that highFrequencyPeriodic produces and periodic logs:
  private Pose3d[][] latestTagPosesPerCamera;
  private Pose3d[][] latestRobotPosesPerCamera;
  private Pose3d[][] latestRobotPosesAcceptedPerCamera;
  private Pose3d[][] latestRobotPosesRejectedPerCamera;
  private CyberVision[] latestCyberVisions;
  private long highFrequencyPeriodicCount = 0;
  private long periodicCount = 0;
  
  public Vision(VisionConsumer consumer, CommandSwerveDrivetrain drivetrain, VisionIO... io) {
    this.consumer = consumer;
    this.io = io;
    this.drivetrain = drivetrain;

    // Initialize inputs
    this.inputs = new VisionIOInputsAutoLogged[io.length];
    for (int i = 0; i < inputs.length; i++) {
      inputs[i] = new VisionIOInputsAutoLogged();
    }

    // Initialize disconnected alerts
    this.disconnectedAlerts = new Alert[io.length];
    for (int i = 0; i < inputs.length; i++) {
      disconnectedAlerts[i] =
          new Alert(
              "Vision camera " + Integer.toString(i) + " is disconnected.", AlertType.kWarning);
    }

    SendableRegistry.addLW(this, this.getClass().getSimpleName(), this.getClass().getSimpleName());
    Notifier fastNotifier = new Notifier(this::highFrequencyPeriodic);
    fastNotifier.startPeriodic(VisionConstants.visionProcessingPeriod); // 10 ms
  }

  /**
   * Returns the X angle to the best target, which can be used for simple servoing with vision.
   *
   * @param cameraIndex The index of the camera to use.
   */
  public Rotation2d getTargetX(int cameraIndex) {
    return inputs[cameraIndex].latestTargetObservation.tx();
  }

  @Override
  public void periodic() {

    // Setup variables to copy data from highFrequencyPeriodic
    Pose3d[][] tagPosesPerCamera;
    Pose3d[][] robotPosesPerCamera;
    Pose3d[][] robotPosesAcceptedPerCamera;
    Pose3d[][] robotPosesRejectedPerCamera;
    CyberVision[] cyberVisions;
    long hfCount;

    synchronized (lock) {
      // local copies so we don't hold the lock while calling Logger
      tagPosesPerCamera = latestTagPosesPerCamera;
      robotPosesPerCamera = latestRobotPosesPerCamera;
      robotPosesAcceptedPerCamera = latestRobotPosesAcceptedPerCamera;
      robotPosesRejectedPerCamera = latestRobotPosesRejectedPerCamera;
      cyberVisions = latestCyberVisions;
      hfCount = highFrequencyPeriodicCount;
    }

    Logger.recordOutput("Periodic Count", periodicCount++);
    Logger.recordOutput("Periodic Count High Freq", hfCount);

    if (tagPosesPerCamera == null) {
      // Notifier hasn’t populated anything yet
      return;
    }

    // Aggregate summaries
    List<Pose3d> allTagPoses = new LinkedList<>();
    List<Pose3d> allRobotPoses = new LinkedList<>();
    List<Pose3d> allRobotPosesAccepted = new LinkedList<>();
    List<Pose3d> allRobotPosesRejected = new LinkedList<>();

    for (int cameraIndex = 0; cameraIndex < io.length; cameraIndex++) {
      // Log auto-logged inputs once per loop
      Logger.processInputs("Vision/Camera" + Integer.toString(cameraIndex), inputs[cameraIndex]);

      Pose3d[] tagPoses = tagPosesPerCamera[cameraIndex];
      Pose3d[] robotPoses = robotPosesPerCamera[cameraIndex];
      Pose3d[] robotPosesAccepted = robotPosesAcceptedPerCamera[cameraIndex];
      Pose3d[] robotPosesRejected = robotPosesRejectedPerCamera[cameraIndex];

      if (tagPoses == null) continue; // safety

      Logger.recordOutput(
          "Vision/Camera" + cameraIndex + "/TagPoses", tagPoses);
      Logger.recordOutput(
          "Vision/Camera" + cameraIndex + "/RobotPoses", robotPoses);
      Logger.recordOutput(
          "Vision/Camera" + cameraIndex + "/RobotPosesAccepted",
          robotPosesAccepted);
      Logger.recordOutput(
          "Vision/Camera" + cameraIndex + "/RobotPosesRejected",
          robotPosesRejected);

      allTagPoses.addAll(List.of(tagPoses));
      allRobotPoses.addAll(List.of(robotPoses));
      allRobotPosesAccepted.addAll(List.of(robotPosesAccepted));
      allRobotPosesRejected.addAll(List.of(robotPosesRejected));

    }

    // Log summary data
    Logger.recordOutput(
        "Vision/Summary/TagPoses", allTagPoses.toArray(new Pose3d[allTagPoses.size()]));
    Logger.recordOutput(
        "Vision/Summary/RobotPoses", allRobotPoses.toArray(new Pose3d[allRobotPoses.size()]));
    Logger.recordOutput(
        "Vision/Summary/RobotPosesAccepted",
        allRobotPosesAccepted.toArray(new Pose3d[allRobotPosesAccepted.size()]));
    Logger.recordOutput(
        "Vision/Summary/RobotPosesRejected",
        allRobotPosesRejected.toArray(new Pose3d[allRobotPosesRejected.size()]));

    if (cyberVisions != null){
      Logger.recordOutput("CyberVisions", cyberVisions);
    }

  }

  public void highFrequencyPeriodic() {
    // This is built to run on a separate thread at 100 Hz.
    // No logging calls allowed in this method, due to thread safety.


    // Local containers built on this thread
    Pose3d[][] tagPosesPerCamera = new Pose3d[io.length][];
    Pose3d[][] robotPosesPerCamera = new Pose3d[io.length][];
    Pose3d[][] robotPosesAcceptedPerCamera = new Pose3d[io.length][];
    Pose3d[][] robotPosesRejectedPerCamera = new Pose3d[io.length][];
    List<CyberVision> allCyberVisions = new LinkedList<>();
    

    // Update the robot pose in VisionIOInputs
    // Pose2d currentRobotPose = drivetrain.getState().Pose; // Get the current robot pose from drivetrain

    // Loop over cameras (no logging)
    for (int cameraIndex = 0; cameraIndex < io.length; cameraIndex++) {
      // Update inputs
      inputs[cameraIndex].estimatedPose2d = drivetrain.getState().Pose; // Set the current robot pose
      inputs[cameraIndex].cycleCount = highFrequencyPeriodicCount;
      io[cameraIndex].updateInputs(inputs[cameraIndex]);

      // Initialize per-camer logging values
      List<Pose3d> tagPoses = new LinkedList<>();
      List<Pose3d> robotPoses = new LinkedList<>();
      List<Pose3d> robotPosesAccepted = new LinkedList<>();
      List<Pose3d> robotPosesRejected = new LinkedList<>();
      List<CyberVision> cyberVisions = new LinkedList<>();

      // Update disconnected alert – ideally this stays on main thread,
      // but if your alert code is simple/set-only you *can* do it here.
      disconnectedAlerts[cameraIndex].set(!inputs[cameraIndex].connected);


      // Add tag poses
      for (int tagId : inputs[cameraIndex].tagIds) {
        var tagPose = aprilTagLayout.getTagPose(tagId);
        if (tagPose.isPresent()) {
          tagPoses.add(tagPose.get());
        }
      }

      // Loop over pose observations
      for (var observation : inputs[cameraIndex].poseObservations) {
        // Check whether to reject pose
        boolean rejectPose =
            observation.tagCount() == 0 // Must have at least one tag
                || (observation.tagCount() == 1
                    && observation.ambiguity() > maxAmbiguity) // Cannot be high ambiguity
                || Math.abs(observation.pose().getZ())
                    > maxZError // Must have realistic Z coordinate

                // Must be within the field boundaries
                || observation.pose().getX() < 0.0
                || observation.pose().getX() > aprilTagLayout.getFieldLength()
                || observation.pose().getY() < 0.0
                || observation.pose().getY() > aprilTagLayout.getFieldWidth();

        // Add pose to log
        robotPoses.add(observation.pose());
        if (rejectPose) {
          robotPosesRejected.add(observation.pose());
        } else {
          robotPosesAccepted.add(observation.pose());
        }

        // Skip if rejected
        if (rejectPose) {
          continue;
        }

        // Calculate standard deviations
        double stdDevFactor =
            Math.pow(observation.averageTagDistance(), 2.0) / observation.tagCount();
        double linearStdDev = linearStdDevBaseline * stdDevFactor;
        double angularStdDev = angularStdDevBaseline * stdDevFactor;
        if (observation.type() == PoseObservationType.MEGATAG_2) {
          linearStdDev *= linearStdDevMegatag2Factor;
          Logger.recordOutput("Megatag2", observation.pose().toPose2d());
          System.out.println(observation.pose());
          angularStdDev *= angularStdDevMegatag2Factor;
        }
        if (cameraIndex < cameraStdDevFactors.length) {
          linearStdDev *= cameraStdDevFactors[cameraIndex];
          angularStdDev *= cameraStdDevFactors[cameraIndex];
        }

        // Send vision observation
        consumer.accept(
            observation.pose().toPose2d(),
            observation.timestamp(),
            VecBuilder.fill(linearStdDev, linearStdDev, angularStdDev));

        System.out.println("TagIds: " + Arrays.toString(inputs[cameraIndex].tagIds));
        cyberVisions.add(new CyberVision(
          cameraIndex,
          observation.timestamp(), 
          observation.pose().toPose2d().getTranslation(),
          observation.pose().toPose2d().getRotation().getDegrees(),
          observation.ambiguity(),
          observation.tagCount(),
          observation.averageTagDistance(),
          observation.type(),
          observation.visionToRobotDistanceError(),
          linearStdDev, 
          angularStdDev, 
          !rejectPose,
          observation.cycleCount()  //dummy value
          ));
      }

      // Convert per-camera lists to arrays for handoff
      tagPosesPerCamera[cameraIndex] = tagPoses.toArray(new Pose3d[tagPoses.size()]);
      robotPosesPerCamera[cameraIndex] = robotPoses.toArray(new Pose3d[robotPoses.size()]);
      robotPosesAcceptedPerCamera[cameraIndex] = robotPosesAccepted.toArray(new Pose3d[robotPosesAccepted.size()]);
      robotPosesRejectedPerCamera[cameraIndex] = robotPosesRejected.toArray(new Pose3d[robotPosesRejected.size()]);
      allCyberVisions.addAll(cyberVisions);
    }

    // Now publish all new data atomically
    synchronized (lock) {
      highFrequencyPeriodicCount++;
      latestTagPosesPerCamera = tagPosesPerCamera;
      latestRobotPosesPerCamera = robotPosesPerCamera;
      latestRobotPosesAcceptedPerCamera = robotPosesAcceptedPerCamera;
      latestRobotPosesRejectedPerCamera = robotPosesRejectedPerCamera;
      latestCyberVisions =
          allCyberVisions.toArray(new CyberVision[allCyberVisions.size()]);
    }
      
      //
  }

  @FunctionalInterface
  public static interface VisionConsumer {
    public void accept(
        Pose2d visionRobotPoseMeters,
        double timestampSeconds,
        Matrix<N3, N1> visionMeasurementStdDevs);
  }
}
