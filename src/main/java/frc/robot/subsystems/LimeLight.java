package frc.robot.subsystems;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.PoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableValue;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.LimelightResults;

public class LimeLight extends SubsystemBase {
  public NetworkTable tableLimelight;
  public NetworkTableEntry txLocal; //horizontal error
  public NetworkTableEntry tyLocal; //vertical error
  public NetworkTableEntry taLocal; //area error
  public NetworkTableEntry tvLocal; //valid target found
  public NetworkTableEntry tsLocal; //skew error
  public NetworkTableEntry tzLocal;
  public NetworkTableEntry tLongLocal;
  public final String limelightName;
  private double horizontalError = 0.0;
  private double verticalError = 0.0;
  private double area = 0.0;
  private double xOffset;
  private double aim = -0.1;
  private double distance = -0.1;
  private double min_aim = -0.1;
  private double visionSpeed;
  private boolean targetInView = false;
  public boolean isLoggingEnabled = false;
  public double spinThreshold = 75;

  double[] positionStandardDeviations = new double[12];
  public LimeLight(String networkTableName) {
    tableLimelight = NetworkTableInstance.getDefault().getTable(networkTableName);
    txLocal = tableLimelight.getEntry("tx"); // communicates horizontal degree offset from target
    tyLocal = tableLimelight.getEntry("ty"); // communicates verticle degree offset from target
    taLocal = tableLimelight.getEntry("ta"); // communicates percentage of image the target takes up
    tvLocal = tableLimelight.getEntry("tv"); // communicates whether a valid target is acquired, 0 or 1
    tsLocal = tableLimelight.getEntry("ts"); // communicates skew offset from target
    tzLocal = tableLimelight.getEntry("tz");
    tLongLocal = tableLimelight.getEntry("tlong");
    limelightName = networkTableName;
    LimelightHelpers.setPipelineIndex(limelightName, 0);

    // Change the camera pose relative to robot center (x forward, y left, z up, degrees)
    LimelightHelpers.setCameraPose_RobotSpace(limelightName,
    0.2,    // Forward offset (meters)
    -0.25,    // Side offset (meters)
    0.64,    // Height offset (meters)
    90,    // Roll (degrees)
    0.0,   // Pitch (degrees)
    0.0     // Yaw (degrees)
    );
  }


/*@Override
  public void periodic() {
    horizontalError = getVisionTargetHorizontalError();
    verticalError = getVisionTargetVerticalError();
    area = getVisionTargetAreaError();
    targetInView = getVisionTargetStatus();
    Pose2d p = getRobotPose();
    if (isLoggingEnabled) {
    SmartDashboard.putString("Camera Pose2d", p.toString());
    LimelightResults limelightResults = LimelightHelpers.getLatestResults(limelightName);
    positionStandardDeviations = limelightResults.standardDeviations;
    if (positionStandardDeviations.length > 2) {
      //System.out.println("Standard deviation of limelight measurement (MegaTag2) " + positionStandardDeviations[0]+" "+positionStandardDeviations[1]);
    } else {
      //System.out.println("No position measurement (MegaTag2)");
    }
    //System.out.println("Time of Limelight publishing measurement: (ms since Limelight boot) " + limelightResults.timestamp_LIMELIGHT_publish + " Rio receipt time: " + limelightResults.timestamp_RIOFPGA_capture);
    //System.out.println("Number of tags visible:"+limelightResults.botpose_tagcount);
    }
    SmartDashboard.putNumber("tx", getVisionTargetHorizontalError());
  }*/
  
  public LimelightResults getLatestResults() {
    return LimelightHelpers.getLatestResults(limelightName);
  }
  // This was refactored into SwerveJoystickCommand
  public boolean alignToTarget(boolean targetFound, double xError, double yError, String zone){
    double yOffset = 0;
    boolean targetAligned = false;
    boolean headingAligned = false;
    boolean distanceAligned = false;
    double vertical_error = 0;
    double steering_adjust = 0;
    double distance_adjust = 0;
    
    
    double xSpeedAdjust = 0;
    double ySpeedAdjust = 0;
    double turningSpeedAdjust = 0;
    
    //Meadow calibrated the limelight cross-hair at ideal distance for first zone to target we shoot from.
    //this meant ty = 0 at learned position at Blue zone.  Team did zones Target->Green->Blue->Yellow->Red
    //Meadow then recorded the different ty "offset" values at each of the other shooter positions.
    //Note - in Green zone we are too close to see target so not used

    vertical_error = - (yError - yOffset);  //error goes to zero as we approach our offset positions and negative accounts for drivetrain
    

    //Calculating the speed at which to rotate based on tx
    /*if (targetFound == true){ //Valid Target Found
      //Clamp speeds to never go below minimum per constants file
      steering_adjust = Math.signum(xError)* (Math.max(VisionConstants.kpAim*Math.abs(xError), VisionConstants.kMinRotateSpeed));
      distance_adjust = Math.signum(verticalError) * (Math.max(VisionConstants.kpVertical*Math.abs(vertical_error), VisionConstants.kMinLinearSpeed));
    } else {  //Target not within range
      steering_adjust = 0;
      distance_adjust = 0;
    } */

    /** 
    //Drive the Robot with the adjusted speeds simultaneously
    double leftSideSpeed = distance_adjust + steering_adjust;
    double rightSideSpeed = distance_adjust - steering_adjust;
    drive.tankDrivePWM(leftSideSpeed, rightSideSpeed);
    **/

    //Check to see if we are in range for heading and turn first
      /*if (Math.abs(xError) <= VisionConstants.kAngleThreshold){// Target is within angle threshold
        headingAligned = true;
      } else {
        headingAligned = false;
        drive.tankDrivePWM(steering_adjust, -steering_adjust);
      }

      //Check to seeif we are in range for distance once we are aligned to heading (based on vertical)
      if (headingAligned && (vertical_error == VisionConstants.kVerticalThreshold)){// Target is within vertical threshold
        distanceAligned = true;
      } else {
        distanceAligned = false;
        drive.tankDrivePWM(distance_adjust, distance_adjust);
      }*/
      
    //Set Exit Flag only once both are aligned
    if (headingAligned && distanceAligned){
      targetAligned = true;
    }else{
      targetAligned = false;
    }

    return targetAligned;

  }  


  // Checks for cone orientation
  public boolean isOriented(){
    if (tLongLocal.getDouble(0) >= spinThreshold) {
      return true;
      //return false;
    } else{
      return false;
    }
  }
  
  // responds true if target is identified in field of view
  public boolean getVisionTargetStatus(){
    boolean returnValue = false;
    
    if (tvLocal.getDouble(0) == 1){
      returnValue = true;
    }

    return returnValue;
  }


  public double getVisionTargetHorizontalError(){
    return txLocal.getDouble(0);
  }

  public double getVisionTargetAreaError(){
    return 0.9-taLocal.getDouble(0);
  }
  public double getVisionArea(){
    return taLocal.getDouble(0.0);
  }

public double getVisionTargetVerticalError(){
  return tyLocal.getDouble(0);
}

public double getVisionTargetSkew(){
  return tsLocal.getDouble(0);
}
public Pose2d getRobotPose() {
  return LimelightHelpers.getBotPose2d(limelightName);
}
public void enable() {
  isLoggingEnabled = true;
}
public void disable() {
  isLoggingEnabled = false;
}
public Color getStatusLed() {
  int tags = LimelightHelpers.getTargetCount(limelightName);
  if (tags == 0) {
    return Color.kRed;
  } else if (tags == 1) {
    return Color.kYellow;
  } else {
    return Color.kGreen;
  }
}
}