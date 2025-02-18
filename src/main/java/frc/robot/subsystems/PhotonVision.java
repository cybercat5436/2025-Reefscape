// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.photonvision.PhotonCamera;

public class PhotonVision extends SubsystemBase {
  /** Creates a new PhotonVision. */
  private PhotonCamera camera = new PhotonCamera("photonvision");

  public PhotonVision() {
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
     // Read in relevant data from the Camera
     boolean targetVisible = false;
     double targetYaw = 0.0;
     var results = camera.getAllUnreadResults();
     if (!results.isEmpty()) {
         // Camera processed a new frame since last
         // Get the last one in the list.
         var result = results.get(results.size() - 1);
         if (result.hasTargets()) {
             // At least one AprilTag was seen by the camera
             for (var target : result.getTargets()) {
                 if (target.getFiducialId() == 7) {
                     // Found Tag 7, record its information
                     targetYaw = target.getYaw();
                     targetVisible = true;
                 }
             }
         }
     }
     SmartDashboard.putNumber("target Yaw", targetYaw);
     SmartDashboard.putBoolean("target Visible", targetVisible);
  }
}
