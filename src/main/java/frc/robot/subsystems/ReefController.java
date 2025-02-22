// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.HashMap;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.generated.TunerConstants;

public class ReefController extends SubsystemBase {
  public enum ReefPosition{
    A, 
    B, 
    C,
    D,
    E,
    F,
    G,
    H,
    I,
    J,
    K,
    L
  }
  private ReefPosition targetReefPosition = ReefPosition.A;
  private int reefLevel = 4;
  private HashMap<ReefPosition, Integer> blueAprilTagMap = new HashMap<>();
  private HashMap<ReefPosition, Integer> redAprilTagMap = new HashMap<>();
  private AprilTagFieldLayout aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

  /** Creates a new ReefController. */
  public ReefController() {
    populateBlueAprilTagMap();
    populateRedAprilTagMap();
  }

  public Pose2d getTargetRobotPose(){
    var alliance = DriverStation.getAlliance().isPresent()?DriverStation.getAlliance().get():DriverStation.Alliance.Blue;
    int tagId =0;
    if (alliance == Alliance.Blue){
      tagId = blueAprilTagMap.get(targetReefPosition);
      System.out.println(tagId + "@#$%^&*(&^%$#$%^&**&^%$%^&*%)");

    } else{
      tagId = redAprilTagMap.get(targetReefPosition);
    }
    Pose2d tagPose2d = aprilTagFieldLayout.getTagPose(tagId).isPresent()?aprilTagFieldLayout.getTagPose(tagId).get().toPose2d():new Pose2d();
    System.out.println(tagPose2d + " $%^");
    var botOffset = new Translation2d(TunerConstants.kBotOffset, tagPose2d.getRotation());
    var botPose = new Pose2d(tagPose2d.getTranslation().plus(botOffset), tagPose2d.getRotation().rotateBy(Rotation2d.k180deg));
    return botPose;
    
    
  }
  public int getTagId(){
    return tagId;
  }

  public void populateBlueAprilTagMap(){
    blueAprilTagMap.put(ReefPosition.A, 18);
    blueAprilTagMap.put(ReefPosition.B, 18);
    blueAprilTagMap.put(ReefPosition.C, 17);
    blueAprilTagMap.put(ReefPosition.D, 17);
    blueAprilTagMap.put(ReefPosition.E, 22);
    blueAprilTagMap.put(ReefPosition.F, 22);
    blueAprilTagMap.put(ReefPosition.G, 21);
    blueAprilTagMap.put(ReefPosition.H, 21);
    blueAprilTagMap.put(ReefPosition.I, 20);
    blueAprilTagMap.put(ReefPosition.J, 20);
    blueAprilTagMap.put(ReefPosition.K, 19);
    blueAprilTagMap.put(ReefPosition.L, 19);
    
  }
  public void populateRedAprilTagMap(){
    redAprilTagMap.put(ReefPosition.A, 7);
    redAprilTagMap.put(ReefPosition.B, 7);
    redAprilTagMap.put(ReefPosition.C, 8);
    redAprilTagMap.put(ReefPosition.D, 8);
    redAprilTagMap.put(ReefPosition.E, 9);
    redAprilTagMap.put(ReefPosition.F, 9);
    redAprilTagMap.put(ReefPosition.G, 10);
    redAprilTagMap.put(ReefPosition.H, 10);
    redAprilTagMap.put(ReefPosition.I, 11);
    redAprilTagMap.put(ReefPosition.J, 11);
    redAprilTagMap.put(ReefPosition.K, 6);
    redAprilTagMap.put(ReefPosition.L, 6);
    
  }
  public void setTargetReefPosition(ReefPosition reefPosition){
    targetReefPosition = reefPosition;
  }
  public void setTargetReefLevel(int level){
    reefLevel = level;
  }
  public ReefPosition getTargetReefPosition(){
    return targetReefPosition;
  };
    int tagId = 0;
  public int getTargetReefLevel(){
    return reefLevel;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
