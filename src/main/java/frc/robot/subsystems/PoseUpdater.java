// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.lang.reflect.Array;
import java.util.Optional;

import com.ctre.phoenix6.SignalLogger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.Odometry;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers.LimelightResults;
import frc.robot.Telemetry;

public class PoseUpdater extends SubsystemBase {
  /** Creates a new PoseUpdater. */
  private final LimeLight limeLightFront;
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

  public PoseUpdater(LimeLight limeLightFront, CommandSwerveDrivetrain swerveSubsystem) {
    this.limeLightFront = limeLightFront;
    this.commandSwerveDrivetrain = swerveSubsystem;
    txLocal = limeLightFront.txLocal;
    taLocal = limeLightFront.taLocal;
    cyclesSinceLocked = 0;

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

 /*  public void updateOdometry(double offsetValue) {
    // Transform2d transform2d = new Transform2d(new Translation2d(0,yError),new Rotation2d());
    System.out.println("inside update odometry");
    Pose2d currentPose = swerveSubsystem.getOdometry().getPoseMeters();
    numAdjustments++;
    
    // figure out correct sign for error
    boolean shouldInvert = false;  // assumes blue
    Optional<Alliance> alliance = DriverStation.getAlliance();
    if(alliance.isPresent()){
      if(alliance.get() == DriverStation.Alliance.Red){
        shouldInvert = true;
      }
    }
    if (shouldInvert){
      offsetValue *= -1;
    }

    // clamp the total adjustment to 0.25m
    double maxAdjustment = 0.25;
    if ((totalAdjustment + offsetValue) > maxAdjustment){
      offsetValue = maxAdjustment - totalAdjustment;
    } else if ((totalAdjustment + offsetValue) < -maxAdjustment){
      offsetValue = -maxAdjustment - totalAdjustment;
    }
    totalAdjustment += offsetValue;
    

    Translation2d translationAdjustment = new Translation2d(0, offsetValue);
    Translation2d translation2d = currentPose.getTranslation().plus(translationAdjustment);
    
    // These are from Tuesday, when the x coordinate was changing.  Note:  Rotation being set to 0
   // Pose2d newPose = new Pose2d(translation2d, new Rotation2d());
    Pose2d newPose = new Pose2d(translation2d, currentPose.getRotation());
    printInfo(currentPose, newPose);
    swerveSubsystem.getOdometry().resetPosition(swerveSubsystem.getRotation2d(), swerveSubsystem.getModulePositions(), newPose);
    
    // This is proposed method where rotation is preserved from pose
    // Pose2d newPose = new Pose2d(translation2d, currentPose.getRotation());
    // swerveSubsystem.resetOdometry(newPose);
  }
  public void undoTotalAdjustment() {

    System.out.println("inside undoTotalAdjustment");
    Pose2d currentPose = swerveSubsystem.getOdometry().getPoseMeters();
    Translation2d translationTotalAdjustment = new Translation2d(0, -totalAdjustment);
    Translation2d translation2d = currentPose.getTranslation().plus(translationTotalAdjustment);
    Pose2d newPose = new Pose2d(translation2d, currentPose.getRotation());
    printInfo(currentPose, newPose);
    swerveSubsystem.getOdometry().resetPosition(swerveSubsystem.getRotation2d(), swerveSubsystem.getModulePositions(), newPose);
  }
  public void resetTotalAdjustment() {
    System.out.println("Total Adjustment zeroed.....");
    totalAdjustment = 0;

  }*/
  public void enable() {
    isEnabled = true;
  }
  public void disable() {
    isEnabled = false;
  }
  @Override
  public void periodic() {
    LimelightResults limelightResults = limeLightFront.getLatestResults();
    // This method will be called once per scheduler run
    isTargetVisible = limeLightFront.getVisionTargetStatus();
    SignalLogger.writeDouble("LimeLight Front/Vision Area", limeLightFront.getVisionArea(), "mm^2");
    
    // Calculate error
    if (limelightResults.botpose_tagcount > 0) {
      System.out.println("Position is being updated");

      // update pose if active  
      if (isEnabled) {
        commandSwerveDrivetrain.addVisionMeasurement(limelightResults.getBotPose2d(), limelightResults.timestamp_LIMELIGHT_publish);
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
    builder.addDoubleProperty("Y Error", () -> yError, null);
    builder.addDoubleProperty("Distance Estimate", () -> distanceEstimate, null);
    builder.addBooleanProperty("isEnabled", () -> isEnabled, null);
    builder.addDoubleProperty("totalAdjustment",() -> totalAdjustment, null);
  }

}
