// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.List;

import org.photonvision.PhotonCamera;

public class PhotonVision extends SubsystemBase {
  /** Creates a new PhotonVision. */
  public double observedYaw;//y value
  public double observedArea;//x value
  private PhotonCamera camera = new PhotonCamera("photonvision");
  public PhotonVision() {
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
     // Read in relevant data from the Camera
    var results = camera.getAllUnreadResults();
     boolean targetVisible = false;
     if (results.isEmpty() == false) {
         // Camera processed a new frame since last
         // Get the last one in the list.
         System.out.println("-----------------------//////Entered Photonvision periodic/////////---------------------");
         var result = results.get(results.size() - 1);
         if (result.hasTargets()) {
             // At least one AprilTag was seen by the camera
             for (var target : result.getTargets()) {
                 if (target.getFiducialId() == 20) {
                     // Found Tag 20, record its information
                     observedYaw = target.getYaw();
                     targetVisible = true;
                     observedArea = target.getArea();
                 }
                 if (target.getFiducialId() == 21) {
                    // Found Tag 20, record its information
                    observedYaw = target.getYaw();
                    targetVisible = true;
                    observedArea = target.getArea();
                }
                observedYaw = target.getYaw();
                SmartDashboard.putNumber(getName(), observedArea);
             }
         }
     }
  

    

     

//     //  System.out.println("*********Observed Yaw*********" + observedYaw);
     SmartDashboard.putNumber("observed area", observedArea);
     SmartDashboard.putNumber("observed Yaw", observedYaw);
     SmartDashboard.putBoolean("target Visible", targetVisible);
    }
}
