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

public class PoseUpdater extends SubsystemBase {
  /** Creates a new PoseUpdater. */
  private final LimeLight limeLight;
  private final LimeLight limeLightFrontRight;
  private List<LimeLight> limeLights = new ArrayList<>();
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

  public PoseUpdater(LimeLight limeLightFront, LimeLight limeLightFrontRight, CommandSwerveDrivetrain swerveSubsystem) {
    this.limeLight = limeLightFront;
    this.limeLightFrontRight = limeLightFrontRight;
    this.commandSwerveDrivetrain = swerveSubsystem;
    txLocal = limeLightFront.txLocal;
    taLocal = limeLightFront.taLocal;
    cyclesSinceLocked = 0;
    limeLights.clear();
    limeLights.add(limeLightFront);
    limeLights.add(limeLightFrontRight);

    //Register the sendables
    SendableRegistry.addLW(this, this.getClass().getSimpleName(), this.getClass().getSimpleName());
    SmartDashboard.putData(this);
  }

  public double getDistanceEstimate(double ta) {
    // 3m: Ta = 0.5
    // 1.5m: Ta = 1.4
    // .75m: Ta = 4.7
   
    double x0=0.5, x1=1.4, x2=4.7,y0=3.0, y1=1.5, y2=0.75;
    // double m = (y2-y1) / (x2-x1);  //slope
    // double d = (ta-x1) * m + y1;
    
    // 3rd order poly fit from Sheets
    //4.34 + -3.09x + 0.875x^2 + -0.0807x^3
    double k0=4.34, k1=-3.09, k2=0.875, k3=-0.0807;
    double d = k0 + k1 * ta + k2 * Math.pow(ta, 2) + k3 * Math.pow(ta, 3);
    
    // Bound distance 0 <= d <= y1
    d = Math.min(y0, d);  // don't let number exceed y0
    d = Math.max(0, d); // don't let be less than 0
    return d;
    // return -0.013*ta + .682; //equation from testing
  }

  public double calculateYError(double tx, double distanceEstimate) {
    // this calculates and returns yError but doesn't set the instance variable

    // Calibration data
    // ------------------------
    // Robot Side(left = -, right = +)
    // 109Cm side, 3m away: Tx = 16.6
    // 104Cm side, 1.5m away: Tx = 30.7
    // 55Cm side, .75m away: Tx = 27.5

    // Old method
    // ------------------------
    // double s1=104/30.7, s2=55/27.5, d1=1.5, d2=0.75;
    // double mSens = (s2-s1) / (d2-d1);  // sensitity slope as function of distance
    // double sensitivity = mSens * (distanceEstimate - d1);
    // bound sensitivity 1 < s < 2.25
    // sensitivity = Math.min(2.25, sensitivity);
    // sensitivity = Math.max(1.0, sensitivity);
    // double yOffset = (tx * sensitivity) / 100;
    // ------------------------
    
    // New method
    // ------------------------
    // use Camera Angle over the distance from the target to determine yOffset
    double cameraHeight = 0.5;   // camera height off ground
    double sightDist = Math.sqrt(Math.pow(distanceEstimate,2) + Math.pow(cameraHeight, 2));
    // yError = sightDist * tan(tx)
    double yOffset = sightDist * Math.tan(Math.toRadians(tx));
    return yOffset;
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

  public void setMT1(){
    isMT2 = false;
  }

  public void setMT2(){
    isMT2 = true;
  }

  @Override
  public void periodic() {
    if (!isEnabled || commandSwerveDrivetrain.getState().Pose == null) return;

    // ~~~~~~~~~~~~~~~~~   Pick MT1 or MT2    ~~~~~~~~~~~~~~~~~
    double robotYaw = commandSwerveDrivetrain.getState().Pose.getRotation().getDegrees();
    for(LimeLight ll: limeLights){
      if (isMT2){
        updatePoseEstimatorMT2(ll, robotYaw);
      }else{
        updatePoseEstimatorMT1(ll);
      }
    }
  }

  private void updatePoseEstimatorMT2(LimeLight limeLight, double robotYaw){
    
    isTargetVisible = limeLight.getVisionTargetStatus();
    if (LimelightHelpers.getTargetCount(limeLight.limelightName) > 0) {
      
      boolean doRejectUpdate = false;  // initialize rough filter

      // Send gyro and get MT2 field estimate
      LimelightHelpers.SetRobotOrientation(limeLight.limelightName, robotYaw, 0.0, 0.0, 0.0, 0.0, 0.0);
      LimelightHelpers.PoseEstimate limelightMeasurementMT2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limeLight.limelightName);

      // only accept vision measurements that are within 1m of current pose
      double distVisionToCurrent = commandSwerveDrivetrain.getState().Pose.getTranslation().getDistance(limelightMeasurementMT2.pose.getTranslation());
      if(distVisionToCurrent > 1.0){
        doRejectUpdate = true;
      }

      // only accept vision measurements when not rotating rapidly (CTRE recommendation)
      double omegaRps = Units.radiansToRotations(commandSwerveDrivetrain.getState().Speeds.omegaRadiansPerSecond);
      if(Math.abs(omegaRps) >= 2.0){
        doRejectUpdate = true;
      }


      // TODO: Update stddev based on number of tags observed (not used yet)
      double positionStdDev = 0.7;
      // if (limelightMeasurementMT2.tagCount > 1){
      //   positionStdDev = 0.5;
      // }

      var myTags = limelightMeasurementMT2.rawFiducials;
      // TODO: Reject if ambiguity too high (not implemented yet)
      // TODO: Adjust stdev based on max tag distance (not implemented yet)
      for(int i = 0; i < myTags.length; i++){
        if (myTags[i].ambiguity > 0.7){
          ambiguityCount++;
          // doRejectUpdate = true;
        } else if(myTags[i].distToCamera > 4.5){
          distanceCount++;
          // doRejectUpdate = true;
        } else if(myTags[i].distToCamera > 3.0){
          distanceCount++;
          // positionStdDev *= 2.0;
        }
      }

      if(!doRejectUpdate){
        // TODO: Update stddev numbers as a function of distance from target (distVisionToCurrent)
        commandSwerveDrivetrain.setVisionMeasurementStdDevs(VecBuilder.fill(.7, .7, 99999999));
        poseCount++;  // count how many vision targets are passed to poseestimator
        commandSwerveDrivetrain.addVisionMeasurement(
            limelightMeasurementMT2.pose,
            limelightMeasurementMT2.timestampSeconds
        );
      }
    }
  }

  private void updatePoseEstimatorMT1(LimeLight limeLight){
    // ~~~~~~~~~~~~~~~~~ updates pose with vision using MegaTag1 ~~~~~~~~~~~~~~~~~
    LimelightHelpers.PoseEstimate limelightMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue(limeLight.limelightName);
    if (limelightMeasurement == null) return;

    boolean doRejectUpdate = false;

    //SmartDashboard.putNumber("Limelight Measured Heading", limelightMeasurement.pose.getRotation().getDegrees());
    if(limelightMeasurement.tagCount == 1 && limelightMeasurement.rawFiducials.length == 1) {
      if(limelightMeasurement.rawFiducials[0].ambiguity > .7)
      {
        ambiguityCount++;
        doRejectUpdate = true;
      }
      if(limelightMeasurement.rawFiducials[0].distToCamera > 3)
      {
        doRejectUpdate = true;
      }
    }
    if(limelightMeasurement.tagCount == 0)
    {
      doRejectUpdate = true;
    }

    // only accept vision measurements that are within 1m of current pose
    if(commandSwerveDrivetrain.getState().Pose.getTranslation().getDistance(limelightMeasurement.pose.getTranslation()) > 1.0){
      doRejectUpdate = true;
    }

    if(!doRejectUpdate){
      // commandSwerveDrivetrain.setVisionMeasurementStdDevs(VecBuilder.fill(.5,.5,.25));
      // commandSwerveDrivetrain.resetPose(limelightMeasurement.pose);
      commandSwerveDrivetrain.setVisionMeasurementStdDevs(VecBuilder.fill(.5,.5,9999999));
      poseCount ++;
      commandSwerveDrivetrain.addVisionMeasurement(
        limelightMeasurement.pose,
        limelightMeasurement.timestampSeconds
      );
    }
  }
  

  @Override
  public void simulationPeriodic() {
    // TODO Auto-generated method stub
    super.simulationPeriodic();
    if(DriverStation.isDisabled()){
      var visionPose = new Pose2d(7.5, 5.5, Rotation2d.k180deg);
      double distVisionToCurrent = commandSwerveDrivetrain.getState().Pose.getTranslation().getDistance(visionPose.getTranslation());
      // System.out.println(String.format("Distance Vision to Current %.2f", distVisionToCurrent));
      
      // without this distance filter you can get diverging pose corrections (swirls off map)
      if(distVisionToCurrent <= 1.0){
        for(LimeLight ll: limeLights){
          commandSwerveDrivetrain.setVisionMeasurementStdDevs(VecBuilder.fill(.5,.5,9999999));
          poseCount++;
          commandSwerveDrivetrain.addVisionMeasurement(
            visionPose,
            Utils.fpgaToCurrentTime(Timer.getFPGATimestamp())
          );
        }
      }
    }
  }


  public void startLockoutPeriod() {
    isLockedOut = true;
    cyclesSinceLocked = 0;
  }

  public void endLockoutPeriod(){
    isLockedOut = false;
  }

  public double calculateYError(){
    // this sets yError instance variable
    ta = taLocal.getDouble(0);
    tx = txLocal.getDouble(0);
    distanceEstimate = getDistanceEstimate(ta);
    yError = calculateYError(tx, distanceEstimate);
    return yError;
  }

  private void printInfo(Pose2d currentPose, Pose2d newPose){
    double adjustment = currentPose.getY() - newPose.getY();
    System.out.println(String.format("Adjusting by %.2f",adjustment));
    System.out.println(String.format("PoseUpdater:: tx: %.2f ta: %.2f with yError: %.2f and distance: %.2f", tx, ta, yError, distanceEstimate));
    System.out.println(String.format("Updating from: %s\nUpdating to:   %s", currentPose.toString(), newPose.toString()));
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    // TODO Auto-generated method stub
    super.initSendable(builder);
    // builder.addDoubleArrayProperty("tx-ta", () -> new double[] {tx, ta}, null);
    builder.addIntegerProperty("pose Count",() -> poseCount, null);
    builder.addIntegerProperty("ambiguity Count",() -> ambiguityCount, null);
    builder.addIntegerProperty("distance Count",() -> distanceCount, null);
    builder.addBooleanProperty("isMT2",() -> isMT2, (value) -> isMT2 = value);
  }

}
