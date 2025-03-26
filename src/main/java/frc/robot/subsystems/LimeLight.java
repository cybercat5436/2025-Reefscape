package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
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
    System.out.println("********Change Limelight Pipeline to 0 in limelight Subsystem********");

    
  }

  public LimeLight(String networkTableName, double forwardMeters, double sideMeters, double upMeters, 
                    double rollDeg, double pitchDeg, double yawDeg){

    this(networkTableName);

    // Change the camera pose relative to robot center (x forward, y left, z up, degrees)
    LimelightHelpers.setCameraPose_RobotSpace(limelightName,
      forwardMeters,    // Forward offset (meters)
      sideMeters,    // Side offset (meters)
      upMeters,    // Height offset (meters)
      rollDeg,    // Roll (degrees)
      pitchDeg,   // Pitch (degrees)
      yawDeg     // Yaw (degrees)
    );

  }

//   LimelightHelpers.setCameraPose_RobotSpace(limelightName,
//   0.02,    // Forward offset (meters)
//   -0.3,    // Side offset (meters)
//   0.65,    // Height offset (meters)
//   -90,    // Roll (degrees)
//   0.0,   // Pitch (degrees)
//   0.0     // Yaw (degrees)
// );
//

  public LimelightResults getLatestResults() {
    return LimelightHelpers.getLatestResults(limelightName);
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