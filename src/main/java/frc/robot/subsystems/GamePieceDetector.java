// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ArrayList;

import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.hardware.CANrange;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
 

public class GamePieceDetector extends SubsystemBase {
  /** Creates a new CoralSensor. */ 

private boolean isGamePieceClose;
private double signalStrengthThreshhold;
private double distanceThreshold = 0;
private CANrange sensorUsed;
private String gamePiece;
private ArrayList<Double> signalStrengths = new ArrayList<>();
private int numValues = 1;
private Sensors sensor;

public enum Sensors{
  coral,
  algae,
}
  public GamePieceDetector(double signalStrength, Sensors sensor) {
    CANrangeConfiguration config = new CANrangeConfiguration();
    signalStrengthThreshhold = signalStrength;
    this.sensor = sensor;

    if (sensor == Sensors.coral){
      numValues =10;
      sensorUsed = new CANrange(17);
      this.gamePiece = "coral";
      config.FovParams.FOVRangeX = 10;
      config.FovParams.FOVRangeY = 27;

    } else if (sensor == Sensors.algae){
      sensorUsed = new CANrange(18);
      this.gamePiece = "algae";
      config.FovParams.FOVRangeX=10;
      config.FovParams.FOVRangeY=10;
      //config.ProximityParams.MinSignalStrengthForValidMeasurement= signalStrength;
    }
    sensorUsed.clearStickyFaults();
    sensorUsed.getConfigurator().apply(config);
    
    
  }
  public GamePieceDetector(double signalStrength, Sensors sensor, double distanceThreshold) {
    this(signalStrength, sensor);
    this.distanceThreshold = distanceThreshold;

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    addValueToBuffer();
    isGamePieceClose = (calculateAverage() > signalStrengthThreshhold);
    if(this.sensor == Sensors.algae) {
      isGamePieceClose = isGamePieceClose && (sensorUsed.getDistance().getValueAsDouble() < this.distanceThreshold);
    }

    //System.out.println(isCoralClose + " signal: " + sensorUsed.getSignalStrength().getValueAsDouble());
    SmartDashboard.putBoolean(this.gamePiece + " is present" , isGamePieceClose);
    SmartDashboard.putNumber(this.gamePiece + " signal strength" , sensorUsed.getSignalStrength(false).getValueAsDouble());
    SmartDashboard.putNumber(this.gamePiece + " Distance" , sensorUsed.getDistance(false).getValueAsDouble());
  }

  private void addValueToBuffer(){
    if (signalStrengths.size() >= numValues){
      signalStrengths.remove(0);
    } 
    signalStrengths.add(sensorUsed.getSignalStrength().getValueAsDouble());

  }
  private double calculateAverage(){
    if (signalStrengths.size() < numValues){
      return 0;
    }

    double average = 0;
    for (double n : signalStrengths){
      average += n;
    }
    
    return average / numValues;
  }
}
