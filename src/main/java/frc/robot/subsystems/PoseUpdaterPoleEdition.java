// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;

import com.ctre.phoenix6.Utils;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.generated.TunerConstants;

public class PoseUpdaterPoleEdition extends SubsystemBase {
  /** Creates a new PoseUpdater. */
  private final LimeLight limeLight;
  private final ReefController reefController;
  private final CommandSwerveDrivetrain commandSwerveDrivetrain;

  private double tx;
  private double ta;
  private boolean isTargetVisible;
  public NetworkTableEntry txLocal;
  public NetworkTableEntry tyLocal;
  public NetworkTableEntry taLocal;
  public double yError = 0.0;
  public double distanceEstimate = 0.0;
  public boolean isEnabled = false;   // can be to prevent updates during certain periods in auton
  public boolean isLockedOut = false;   // odometry updates can be locked out for a period of time after updating
  public int cyclesSinceLocked = 0;
  public final int CYCLE_LOCKOUT = 20;  // 50 cycles per second
  public double totalAdjustment = 0;
  public int numAdjustments = 0;
  public int poseCount;
  public int ambiguityCount;
  public int distanceCount;
  public boolean isMT2 = true;

  public PoseUpdaterPoleEdition(LimeLight limeLight, ReefController reefController, CommandSwerveDrivetrain swerveSubsystem) {
    this.limeLight = limeLight;
    this.commandSwerveDrivetrain = swerveSubsystem;
    this.reefController = reefController;
    txLocal = limeLight.txLocal;
    taLocal = limeLight.taLocal;
    cyclesSinceLocked = 0;

    //Register the sendables
    SendableRegistry.addLW(this, this.getClass().getSimpleName(), this.getClass().getSimpleName());
    SmartDashboard.putData(this);
  }
  
  /** Calculate straight line distance from camera to target */
  public double getDistanceEstimate() {
    // use vision to provide a distance estimate
    
    ta = taLocal.getDouble(0.0);  // read the value from the network table

    double yMax = 2.0;  // set max allowable y distance
    
    // TODO: DEFINE THIS
    // Set the robot at known distances from the target (using robot centerX and offset in Y to align with elevator)
    // Record at least 4 points of (distance, area)
    // Use tool like Google Sheets to perform a polynomial fit of the data in the format: y = k0 + k1(d) + k2(d^2) + k3(d^3)

    // 3rd order poly fit from Sheets
    //4.34 + -3.09x + 0.875x^2 + -0.0807x^3
    double k0=4.34, k1=-3.09, k2=0.875, k3=-0.0807;
    double d = k0 + k1 * ta + k2 * Math.pow(ta, 2) + k3 * Math.pow(ta, 3);

    
    // Bound distance 0 <= d <= y1
    d = Math.min(yMax, d);  // don't let number exceed y0
    d = Math.max(TunerConstants.kBotOffset, d); // don't let be less than 0

    return d;

  }

  /** Convert the straight line distance to Translation2d that locates the center of the robot using camera location */
  private Translation2d getTranslationToBotCenter(double distance){
    Translation2d robotOffset = LimelightHelpers.getCameraPose3d_RobotSpace(limeLight.limelightName).toPose2d().getTranslation();
    Translation2d visionTranslation2d = new Translation2d(distance, Rotation2d.kZero);
    return visionTranslation2d.minus(robotOffset);
  }

  /** Uses target's position, robot heading and translationToBotCenter to estimate field position */
  private Pose2d getEstimatedFieldPose(Translation2d translationToBotCenter){
    // 1) get the target's position
    // 2) calculate the relative angle of the bot
    // 3) perform vector math to determine field position

    Pose2d targetPose = reefController.getTargetRobotPose();


    return null;
  }

  public double calculateYError(double ty, double distanceEstimate) {
    // this calculates and returns yError but doesn't set the instance variable

    // New method
    // ------------------------
    // use Camera Angle over the distance from the target to determine yOffset
    double cameraHeight = 0.5;   // camera height off ground
    double sightDist = Math.sqrt(Math.pow(distanceEstimate,2) + Math.pow(cameraHeight, 2));
    // yError = sightDist * tan(tx)
    double yOffset = sightDist * Math.tan(Math.toRadians(ty));
    return yOffset;
  }

  private Pose2d calculatePose2d(ReefController.ReefPosition reefPosition){
    // Here's the steps to estimate the robot position
    // 1) Get the distances from camera to target
    // 2) Estimate distance to center of robot by offseting for camera position in robot
    // 3) Use the pose of the target and the heading of the robot to estimate field position
    // First get a distance error based on ta, this represents the hypotenuse
    // Next get the angle error, use it to calculate the robot-centric error components
    
    // guard if no target visible
    if(!hasTarget()) return new Pose2d();

    double distance = getDistanceEstimate();
    Translation2d translationToBotCenter = getTranslationToBotCenter(distance);
    Pose2d estimatedFieldPose = getEstimatedFieldPose(translationToBotCenter); 
    
    return estimatedFieldPose;
  }

  public void enable() {
    isEnabled = true;
  }
  public void disable() {
    isEnabled = false;
  }

  public void resetPoseCount(){
    poseCount = 0;
  }

  private boolean hasTarget(){
    return LimelightHelpers.getTV(limeLight.limelightName);
  }


  @Override
  public void periodic() {
    if (!isEnabled || commandSwerveDrivetrain.getState().Pose == null) return;

    // ~~~~~~~~~~~~~~~~~   Estimate pose based on visible area and angle    ~~~~~~~~~~~~~~~~~
    
    boolean doRejectUpdate = false;
    // verify the robot yaw is within target band
    double yawErrorMax = 2.0;
    Rotation2d targetYaw = Rotation2d.fromDegrees(-60);
    Rotation2d robotYaw = commandSwerveDrivetrain.getState().Pose.getRotation();
    Rotation2d yawError = robotYaw.minus(targetYaw);
    if(Math.abs(yawError.getDegrees()) > yawErrorMax){
      doRejectUpdate = true;
    }



  }

  // private void updatePoseEstimatorMT2(LimeLight limeLight, double robotYaw){
    
  //   isTargetVisible = limeLight.getVisionTargetStatus();
  //   if (LimelightHelpers.getTargetCount(limeLight.limelightName) > 0) {
      
  //     boolean doRejectUpdate = false;  // initialize rough filter

  //     // only accept vision measurements that are within 1m of current pose
  //     double distVisionToCurrent = commandSwerveDrivetrain.getState().Pose.getTranslation().getDistance(limelightMeasurementMT2.pose.getTranslation());
  //     if(distVisionToCurrent > 1.0){
  //       doRejectUpdate = true;
  //     }

  //     // only accept vision measurements when not rotating rapidly (CTRE recommendation)
  //     double omegaRps = Units.radiansToRotations(commandSwerveDrivetrain.getState().Speeds.omegaRadiansPerSecond);
  //     if(Math.abs(omegaRps) >= 2.0){
  //       doRejectUpdate = true;
  //     }


  //     // TODO: Update stddev based on number of tags observed (not used yet)
  //     double positionStdDev = 0.7;
  //     // if (limelightMeasurementMT2.tagCount > 1){
  //     //   positionStdDev = 0.5;
  //     // }


  //     if(!doRejectUpdate){
  //       // TODO: Update stddev numbers as a function of distance from target (distVisionToCurrent)
  //       commandSwerveDrivetrain.setVisionMeasurementStdDevs(VecBuilder.fill(.7, .7, 99999999));
  //       poseCount++;  // count how many vision targets are passed to poseestimator
  //       commandSwerveDrivetrain.addVisionMeasurement(
  //           limelightMeasurementMT2.pose,
  //           limelightMeasurementMT2.timestampSeconds
  //       );
  //     }
  //   }
  // }


  
  @Override
  public void simulationPeriodic() {
    // TODO Auto-generated method stub
    super.simulationPeriodic();
    if(DriverStation.isDisabled()){
      var visionPose = new Pose2d(7.5, 5.5, Rotation2d.k180deg);
      double distVisionToCurrent = commandSwerveDrivetrain.getState().Pose.getTranslation().getDistance(visionPose.getTranslation());
      System.out.println(String.format("Distance Vision to Current %.2f", distVisionToCurrent));
      
      // without this distance filter you can get diverging pose corrections (swirls off map)
      if(distVisionToCurrent <= 1.0){

      }
      
    }
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    // TODO Auto-generated method stub
    super.initSendable(builder);

  }

}
