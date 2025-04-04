// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.lang.reflect.Array;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.ExternalFeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorArrangementValue;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.DetectReefWithCANrange;
import static edu.wpi.first.units.Units.*;



public class Elevator extends SubsystemBase {
  /** Creates a new Elevator. */
  MotionMagicVoltage m_motmag = new MotionMagicVoltage(0);
  private final VelocityVoltage m_velocityVoltage = new VelocityVoltage(0).withSlot(1);
  final VoltageOut m_request = new VoltageOut(0);
  private double L1 = 0;
  private double L2 = 0.93;
  private double L3 = 2.89;
  private double L4 = 5.65; //5.85
 
  public int blueHeightAdjustmentLevel4 = 0;
  private int blueHeightAdjustmentLevel3 = 0;
  private int blueHeightAdjustmentLevel2 = 0;
 
  public int redHeightAdjustmentLevel4 = 0;
  private int redHeightAdjustmentLevel3 = 0;
  private int redHeightAdjustmentLevel2 = 0;


  public int elevatorLevel;
  private final TalonFX elevator = new TalonFX(12);
  // private final CANrange reefDetector;
  private boolean readyToShoot;
  private double higherdistanceThresholdL3 = 0;
  private double lowerdistanceThresholdL3 = 0;
  private double higherdistanceThresholdL2 = 0;
  private double lowerdistanceThresholdL2 = 0;

  private double standardIncrement = 0.1;
  public boolean l4HasRun;
  public boolean l3HasRun;
  public boolean l2HasRun;
  

  public Alliance alliance;
  public Elevator() {
    // reefDetector = new CANrange(18);
    // var talonFXConfigs = new TalonFXConfiguration();
    // talonFXConfigs.Commutation.MotorArrangement = MotorArrangementValue.Minion_JST;
    // talonFXConfigs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    // talonFXConfigs.ExternalFeedback.ExternalFeedbackSensorSource = ExternalFeedbackSensorSourceValue.Commutation;
    // elevator.setPosition(0);
    // // set slot 0 gains
    // var slot0Configs = talonFXConfigs.Slot0;
    // slot0Configs.kS = 0.24; // add 0.24 V to overcome friction
    // slot0Configs.kV = 0.12; // apply 12 V for a target velocity of 100 rps
    // // PID runs on position
    // slot0Configs.kP = 4.8;
    // slot0Configs.kI = 0;
    // slot0Configs.kD = 0.1;

    // set Motion Magic settings
    // var motionMagicConfigs = talonFXConfigs.MotionMagic;
    // motionMagicConfigs.MotionMagicCruiseVelocity = 300; // 80 rps cruise velocity
    // motionMagicConfigs.MotionMagicAcceleration = 1000; // 160 rps/s acceleration (0.5 seconds)
    // motionMagicConfigs.MotionMagicJerk = 200; // 1600 rps/s^2 jerk (0.1 seconds)

    // elevator.getConfigurator().apply(talonFXConfigs, 0.050);
     TalonFXConfiguration cfg = new TalonFXConfiguration();
    //  .withCurrentLimits(
    //         new CurrentLimitsConfigs()
    //             .withStatorCurrentLimit(Amps.of(120))
    //             .withStatorCurrentLimitEnable(true)
    //     );
        
     cfg.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
     FeedbackConfigs fdb = cfg.Feedback;
     fdb.SensorToMechanismRatio = 12.8;
    /* Configure gear ratio */
    /* Configure Motion Magic */
    MotionMagicConfigs mm = cfg.MotionMagic;
    mm.withMotionMagicCruiseVelocity(75) // 75 // 5 (mechanism) rotations per second cruise
      .withMotionMagicAcceleration(30) //30 // Take approximately 0.5 seconds to reach max vel
      // Take approximately 0.1 seconds to reach max accel 
      .withMotionMagicJerk(300);

    Slot0Configs slot0 = cfg.Slot0;
    slot0.kS = 0.25; // Add 0.25 V output to overcome static friction
    slot0.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
    slot0.kA = 0.01; // An acceleration of 1 rps/s requires 0.01 V output
    slot0.kP = 60; // A position error of 0.2 rotations results in 12 V output
    slot0.kI = 0; // No output for integrated error
    slot0.kD = 0.5; // A velocity error of 1 rps results in 0.5 V output
    Slot1Configs slot1 = cfg.Slot1;
    slot1.kS = 0.1; // To account for friction, add 0.1 V of static feedforward
    slot1.kV = 0.12; // Kraken X60 is a 500 kV motor, 500 rpm per V = 8.333 rps per V, 1/8.33 = 0.12 volts / rotation per second
    slot1.kP = 0.11; // An error of 1 rotation per second results in 0.11 V output
    slot1.kI = 0; // No output for integrated error
    slot1.kD = 0; // No output for error derivative
    // Peak output of 8 volts
    // cfg.Voltage.withPeakForwardVoltage(Volts.of(8))
    //   .withPeakReverseVoltage(Volts.of(-8));


    StatusCode status = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {
      status = elevator.getConfigurator().apply(cfg);
      if (status.isOK()) break;
    }
    if (!status.isOK()) {
      System.out.println("Could not configure device. Error: " + status.toString());
    }
    alliance = DriverStation.getAlliance().isPresent()?DriverStation.getAlliance().get():DriverStation.Alliance.Blue;
     //Register the sendables
    SendableRegistry.addLW(this, this.getClass().getSimpleName(), this.getClass().getSimpleName());
    SmartDashboard.putData(this);

    elevator.setPosition(0);
    SmartDashboard.putNumber("Level 4 target height", L4);
  }

  

  // public void raiseLevel1() {
  //   m_motmag.Slot = 0;
  //   elevator.setControl(m_motmag.withPosition(L1));
  // } 
  public void incrementHeightAdjustment() {
    if(alliance == Alliance.Red){
      if(elevatorLevel == 2){
        redHeightAdjustmentLevel2++;
        elevator.setControl(m_motmag.withPosition(L2 + redHeightAdjustmentLevel2 * standardIncrement));
        System.out.println("Increased Height Level 2 " + redHeightAdjustmentLevel2);
      }else if (elevatorLevel == 3) { 
        redHeightAdjustmentLevel3++;
        elevator.setControl(m_motmag.withPosition(L3 + redHeightAdjustmentLevel3 * standardIncrement));
        System.out.println("Decreased Height Level 3 " + redHeightAdjustmentLevel3);
      }else if(elevatorLevel == 4){
        redHeightAdjustmentLevel4++;
        elevator.setControl(m_motmag.withPosition(L4 + redHeightAdjustmentLevel4 * standardIncrement));
        System.out.println("Increased Height Level 4 " + redHeightAdjustmentLevel4);
      }
    } else{
    if(elevatorLevel == 2){
      blueHeightAdjustmentLevel2++;
      elevator.setControl(m_motmag.withPosition(L2 + blueHeightAdjustmentLevel2 * standardIncrement));
      System.out.println("Increased Height Level 2 " + blueHeightAdjustmentLevel2);
    }else if (elevatorLevel == 3) { 
      blueHeightAdjustmentLevel3++;
      elevator.setControl(m_motmag.withPosition(L3 + blueHeightAdjustmentLevel3 * standardIncrement));
      System.out.println("Increased Height Level 3 " + blueHeightAdjustmentLevel3);
    }else if(elevatorLevel == 4){
      blueHeightAdjustmentLevel4++;
      elevator.setControl(m_motmag.withPosition(L4 + blueHeightAdjustmentLevel4 * standardIncrement));
      System.out.println("Increased Height Level 4 " + blueHeightAdjustmentLevel4);
    }
  }
  }
  public void decrementHeightAdjustment() {
    if(alliance == Alliance.Red){
          if(elevatorLevel == 2) {
      redHeightAdjustmentLevel2--;
    elevator.setControl(m_motmag.withPosition(L2 + redHeightAdjustmentLevel2 * standardIncrement));
    System.out.println("Decreased Height Level 2 " + redHeightAdjustmentLevel2);
    }else if(elevatorLevel == 3) {
      redHeightAdjustmentLevel3--;
      elevator.setControl(m_motmag.withPosition(L3 + redHeightAdjustmentLevel3 * standardIncrement));
      System.out.println("Decreased Height Level 3 " + redHeightAdjustmentLevel3);
    }else if(elevatorLevel == 4) {
      redHeightAdjustmentLevel4--;
      elevator.setControl(m_motmag.withPosition(L4 + redHeightAdjustmentLevel4 * standardIncrement));
      System.out.println("Decreased Height Level 4 " + redHeightAdjustmentLevel4);
    }
    }else{
    if(elevatorLevel == 2) {
      blueHeightAdjustmentLevel2--;
    elevator.setControl(m_motmag.withPosition(L2 + blueHeightAdjustmentLevel2 * standardIncrement));
    System.out.println("Decreased Height Level 2 " + blueHeightAdjustmentLevel2);
    }else if(elevatorLevel == 3) {
      blueHeightAdjustmentLevel3--;
      elevator.setControl(m_motmag.withPosition(L3 + blueHeightAdjustmentLevel3 * standardIncrement));
      System.out.println("Decreased Height Level 3 " + blueHeightAdjustmentLevel3);
    }else if(elevatorLevel == 4) {
      blueHeightAdjustmentLevel4--;
      elevator.setControl(m_motmag.withPosition(L4 + blueHeightAdjustmentLevel4 * standardIncrement));
      System.out.println("Decreased Height Level 4 " + blueHeightAdjustmentLevel4);
    }
  }
  }
  


  public void raise() { 
    elevator.set(0.1);
  }
  public void lower() { 
    elevator.set(-0.1);
  }
  public void raiseStartLevel() {
    elevatorLevel = 0;
    m_motmag.Slot = 0;
    elevator.setControl(m_motmag.withPosition(L1));
  } 
  public void raiseLevel2() {
    if(alliance == Alliance.Red){
      elevatorLevel = 2;
      m_motmag.Slot = 0;
       elevator.setControl(m_motmag.withPosition(L2 + redHeightAdjustmentLevel2 * standardIncrement));
    } else{
    elevatorLevel = 2;
    m_motmag.Slot = 0;
    elevator.setControl(m_motmag.withPosition(L2 + blueHeightAdjustmentLevel2 * standardIncrement));
    }
  }
  public void raiseLevel3() {
    if(alliance == Alliance.Red){
      elevatorLevel = 3;
      m_motmag.Slot = 0;
      elevator.setControl(m_motmag.withPosition(L3 + redHeightAdjustmentLevel3 * standardIncrement));
    } else {
     elevatorLevel = 3;
      m_motmag.Slot = 0;
      elevator.setControl(m_motmag.withPosition(L3 + blueHeightAdjustmentLevel3 * standardIncrement));
    }
  } 
  public void raiseLevel4() {
    if (alliance == Alliance.Red){
      elevatorLevel = 4;
      m_motmag.Slot = 0;
      elevator.setControl(m_motmag.withPosition(L4 + redHeightAdjustmentLevel4 * standardIncrement));
      System.out.println("$$$$$$$$$$$$$$raised level 4$$$$%$$$$$$$");
    } else {
      elevatorLevel = 4;
      m_motmag.Slot = 0;
      elevator.setControl(m_motmag.withPosition(L4 + blueHeightAdjustmentLevel4 * standardIncrement));
    }
  }
  public void moveUpSlowly() {
    elevator.setControl(m_velocityVoltage.withVelocity(RotationsPerSecond.of(11)));
  }
  public void holdPosition() {
    elevator.setControl(m_motmag.withPosition(this.getEncoderValue()));
  }
  public void resetEncoder() {
    elevator.setPosition(0);
  }
   
  
  public void stopElevator() {
    elevator.setControl(m_request.withOutput(0));
  }
  public void transferTargetHeight() {
    switch (elevatorLevel) {
      case 4: L4 = getEncoderValue(); break;
      case 3: L3 = getEncoderValue(); break;
      case 2: L2 = getEncoderValue(); break;

      default: 
        break;
    }

    SmartDashboard.putNumber("Set Height ", getEncoderValue());
  }

  public double getEncoderValue() {
    return elevator.getPosition().getValueAsDouble();
  }

  public boolean detectReefL3(){
    System.out.println("*******Encoder is Correct L3*******");
    if ((this.getEncoderValue() < 3) && (this.getEncoderValue() > 2)) {
      // if ((reefDetector.getDistance().getValueAsDouble() < higherdistanceThresholdL3) && (reefDetector.getDistance().getValueAsDouble() > lowerdistanceThresholdL3)) {
      // System.out.println("******In Shooting Position L3******");
      // readyToShoot = true;
      // return true;
      // }
    }

    return false;
  }
  public boolean detectReefL2(){
    if ((this.getEncoderValue() < 1) && (this.getEncoderValue() > 0)) {
      System.out.println("*******Encoder is Correct L2*******");
      // if ((reefDetector.getDistance().getValueAsDouble() < higherdistanceThresholdL2) && (reefDetector.getDistance().getValueAsDouble() > lowerdistanceThresholdL2)) {
      // System.out.println("******In Shooting Position L2******");
      // readyToShoot = true;
      // return true;
      // }
    }

    return false;
  }

  public boolean atTargetHeight(){
    double targetHeight;
    double numAdjustments;
    switch (elevatorLevel) {
      case 4:
        targetHeight = L4;
        numAdjustments = alliance == Alliance.Blue ? blueHeightAdjustmentLevel4 : redHeightAdjustmentLevel4;
        break;
      case 3:
        targetHeight = L3;
        numAdjustments = alliance == Alliance.Blue ? blueHeightAdjustmentLevel3 : redHeightAdjustmentLevel3;

        break;
      case 2: 
        targetHeight = L2;
        numAdjustments = alliance == Alliance.Blue ? blueHeightAdjustmentLevel2 : redHeightAdjustmentLevel2;

        break;

      default:
        targetHeight = 0;
        numAdjustments = 0;
        break;
    }
    targetHeight += numAdjustments * standardIncrement;
    double allowableError = 0.2;
    double actualError = getEncoderValue() - targetHeight;

    return Math.abs(actualError) <= allowableError;
  }
  

  @Override
  
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Elevator Encoder", elevator.getPosition().getValueAsDouble());
    //SmartDashboard.putNumber("Elevator Quadrature Position", elevator.getRawQuadraturePosition().getValueAsDouble());
  
  }
  @Override
  public void initSendable(SendableBuilder builder) {
    // TODO Auto-generated method stub
    super.initSendable(builder);
    // builder.addDoubleArrayProperty("tx-ta", () -> new double[] {tx, ta}, null);

  }
}
