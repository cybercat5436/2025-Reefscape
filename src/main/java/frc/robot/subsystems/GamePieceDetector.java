// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.security.CryptoPrimitive;
import java.util.ArrayList;

import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.configs.FovParamsConfigs;
import com.ctre.phoenix6.configs.ProximityParamsConfigs;
import com.ctre.phoenix6.hardware.CANrange;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
 

public class GamePieceDetector extends SubsystemBase {
  /** Creates a new CoralSensor. */ 

public boolean isGamePieceClose;
private double signalStrengthThreshhold;
private double distanceThreshold = 0;
private CANrange canRange;
private String gamePiece;
private ArrayList<Double> signalStrengths = new ArrayList<>();
private int numValues = 1;
private Sensors sensor;
private double maxDistanceThreshold = .525;
private double minDistanceThreshold = .4;



public enum Sensors{
  coral,
  algae,
  reef
}
  public GamePieceDetector(double signalStrength, Sensors sensor) {
    CANrangeConfiguration config = new CANrangeConfiguration();
    signalStrengthThreshhold = signalStrength;
    this.sensor = sensor;

    if (sensor == Sensors.coral){
      numValues =10;
      canRange = new CANrange(17);
      this.gamePiece = "coral";
      config.FovParams.FOVRangeX = 10;
      config.FovParams.FOVRangeY = 27;

    // } else if (sensor == Sensors.algae){
    //   canRange = new CANrange(18);
    //   this.gamePiece = "algae";
    //   config.FovParams.FOVRangeX=10;
    //   config.FovParams.FOVRangeY=10;
    //   //config.ProximityParams.MinSignalStrengthForValidMeasurement= signalStrength;
    } else if(sensor == Sensors.reef) { 
      canRange = new CANrange(18);
      this.gamePiece = "reef";
       config = new CANrangeConfiguration()
            .withFovParams(new FovParamsConfigs()
                .withFOVRangeX(6.75)
                .withFOVRangeY(6.75)
            ).withProximityParams(new ProximityParamsConfigs()
                .withProximityHysteresis(0.05)
                .withProximityThreshold(0.45)
                .withMinSignalStrengthForValidMeasurement(1500)
            );
    }
    
    canRange.clearStickyFaults();
    canRange.getConfigurator().apply(config);
    
    
  }
  public GamePieceDetector(double signalStrength, Sensors sensor, double distanceThreshold) {
    this(signalStrength, sensor);
    this.distanceThreshold = distanceThreshold;

  }
  public void setConfiguration(int level) {
    if(sensor != Sensors.reef) return;
    var config = new CANrangeConfiguration()
      .withFovParams(new FovParamsConfigs()
        .withFOVRangeX(6.75)
        .withFOVRangeY(6.75)
      );
    
      maxDistanceThreshold = 0.525;
      minDistanceThreshold = 0.4;
      config = config
          .withProximityParams(new ProximityParamsConfigs()
              .withProximityHysteresis(0.05)
              .withProximityThreshold(0.75)
              .withMinSignalStrengthForValidMeasurement(1500)
          );
    
    canRange.getConfigurator().apply(config);
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    addValueToBuffer();
    isGamePieceClose = (calculateAverage() > signalStrengthThreshhold);
    if(this.sensor == Sensors.algae) {
      isGamePieceClose = isGamePieceClose && (canRange.getDistance().getValueAsDouble() < this.distanceThreshold);
    }else if( this.sensor == Sensors.reef) {
      // isGamePieceClose = canRange.getIsDetected().getValue();
      
      boolean isSignalStrong = canRange.getSignalStrength().getValueAsDouble() > signalStrengthThreshhold;
      double currentDistance = canRange.getDistance().getValueAsDouble();
      boolean isDistanceCorrect = (currentDistance > minDistanceThreshold) && (currentDistance < maxDistanceThreshold);
      isGamePieceClose = isSignalStrong && isDistanceCorrect;
    }
    

    //System.out.println(isCoralClose + " signal: " + sensorUsed.getSignalStrength().getValueAsDouble());
    SmartDashboard.putBoolean(this.gamePiece + " is present" , isGamePieceClose);
    SmartDashboard.putNumber(this.gamePiece + " signal strength" , canRange.getSignalStrength(false).getValueAsDouble());
    SmartDashboard.putNumber(this.gamePiece + " Distance" , canRange.getDistance(false).getValueAsDouble());
  }

  private void addValueToBuffer(){
    if (signalStrengths.size() >= numValues){
      signalStrengths.remove(0);
    } 
    signalStrengths.add(canRange.getSignalStrength().getValueAsDouble());

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
