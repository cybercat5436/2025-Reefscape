// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Set;

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
  public enum PolePlacement{
    Left,
    Right
  }

  private ReefPosition targetReefPosition = ReefPosition.A;
  private int reefLevel = 4;
  private int tagId = 0;
  private Alliance alliance = DriverStation.getAlliance().isPresent()?DriverStation.getAlliance().get():DriverStation.Alliance.Blue;
  private PolePlacement polePlacement = PolePlacement.Left;
  private HashMap<ReefPosition, Integer> blueAprilTagMap = new HashMap<>();
  private HashMap<ReefPosition, Integer> redAprilTagMap = new HashMap<>();
  private HashMap<PolePlacement, Set<ReefPosition>> polePlacementMap = new HashMap<>();
  private AprilTagFieldLayout aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
  private double poleOffsetDistanceMeters = 0.165; // Offset distance between center of April tag and reef pole 33 / 2 cm
  private Pose2d aprilTagPose2d = new Pose2d();
  private static ReefController reefControllerInstance;
  /** Creates a new ReefController. */
  private ReefController() {
    populateBlueAprilTagMap();
    populateRedAprilTagMap();
    populatePolePlacementMap();
  }
  public static ReefController getInstance(){
    if (reefControllerInstance == null) {
      reefControllerInstance = new ReefController();
    }
    return reefControllerInstance;
  }
  public Pose2d getTargetRobotPose(){
    alliance = DriverStation.getAlliance().isPresent()?DriverStation.getAlliance().get():DriverStation.Alliance.Blue;
    tagId =0;
    if (alliance == Alliance.Blue){
      tagId = blueAprilTagMap.get(targetReefPosition);
      // System.out.println(tagId + "@#$%^&*(&^%$#$%^&**&^%$%^&*%)");

    } else{
      tagId = redAprilTagMap.get(targetReefPosition);
    }
    aprilTagPose2d = aprilTagFieldLayout.getTagPose(tagId).isPresent()?aprilTagFieldLayout.getTagPose(tagId).get().toPose2d():new Pose2d();
    // System.out.println(aprilTagPose2d + " $%^");

    // From the center of the tag, the following translations are required:
    // a) translate in tag orientation to center of bot x-dir
    // b) translate left or right relative to april tag depending  on which reef polePlacement
    // c) translate bot to the right (relative to april tag) to align coral mechanism with pole (coral mech is biased to bot's left side)
    // d) rotate 180 from tag so front of bot is facing april tag
    
    // a) translate in tag orientation to center of bot x-dir
    var aprilTagTranslation2d = aprilTagPose2d.getTranslation();
    var botOffset = new Translation2d(TunerConstants.kBotOffset, aprilTagPose2d.getRotation());

    // b) translate left or right depending  on which reef polePlacement
    boolean isPoleLeft = polePlacementMap.get(PolePlacement.Left).contains(targetReefPosition);
    polePlacement = isPoleLeft ? PolePlacement.Left : PolePlacement.Right;
    // if pole is to the left, the vector pointing out of the tag must rotate clockwise
    var poleTranslationAngle = isPoleLeft ? Rotation2d.kCW_90deg : Rotation2d.kCCW_90deg;
    var poleTranslation2d = new Translation2d(poleOffsetDistanceMeters, aprilTagPose2d.getRotation().rotateBy(poleTranslationAngle));

    // c) translate bot to the right (relative to april tag) to align coral mechanism with pole (coral mech is biased to bot's left side)
    var coralTranslation2d = new Translation2d(TunerConstants.kCoralYOffset, aprilTagPose2d.getRotation().rotateBy(Rotation2d.kCCW_90deg));

    // d) rotate 180 from tag so front of bot is facing april tag
    var botRotation2d = aprilTagPose2d.getRotation().rotateBy(Rotation2d.k180deg);

    var botTranslation2d = aprilTagTranslation2d
                            .plus(botOffset)
                            .plus(poleTranslation2d)
                            .plus(coralTranslation2d);

    var botPose = new Pose2d(botTranslation2d, botRotation2d);
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

  private void populatePolePlacementMap(){
    polePlacementMap.put(PolePlacement.Left, new HashSet<>(List.of(
      ReefPosition.A,ReefPosition.C,ReefPosition.E,ReefPosition.G,ReefPosition.I,ReefPosition.K
      )));
    polePlacementMap.put(PolePlacement.Right, new HashSet<>(List.of(
      ReefPosition.B,ReefPosition.D,ReefPosition.F,ReefPosition.H,ReefPosition.J,ReefPosition.L
    )));
  }


  public void setTargetReefPosition(ReefPosition reefPosition){
    targetReefPosition = reefPosition;
  }

  public void setTargetReefLevel(int level){
    reefLevel = level;
  }

  public ReefPosition getTargetReefPosition(){
    return targetReefPosition;
  }

  public int getTargetReefLevel(){
    return reefLevel;
  }

  @Override
  public String toString(){
    // must run getTargetRobotPose first to set all the correct values
    var botPose2d = getTargetRobotPose();
    return String.format("~~~~ Reef Position %s Alliance: %s Tag ID: %d Pole Placement: %s\nApril Tag Pose: %s\nCalculated Bot Pose: %s\n", 
    targetReefPosition.name(), alliance.toString(), tagId, polePlacement.name(), aprilTagPose2d.toString(), botPose2d.toString());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
