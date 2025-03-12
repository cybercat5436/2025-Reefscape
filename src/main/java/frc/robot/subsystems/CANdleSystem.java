// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.commands.AutoAlign;
import frc.robot.generated.TunerConstants;

import com.ctre.phoenix.led.*;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdle.VBatOutputMode;
import com.ctre.phoenix.led.ColorFlowAnimation.Direction;
import com.ctre.phoenix.led.LarsonAnimation.BounceMode;
import com.ctre.phoenix.led.TwinkleAnimation.TwinklePercent;
import com.ctre.phoenix.led.TwinkleOffAnimation.TwinkleOffPercent;

public class CANdleSystem extends SubsystemBase {
    private final CANdle m_candle = new CANdle(TunerConstants.kCANdleID, "rio");
    private final int LedCount = 512;
    private XboxController joystick;
    private GamePieceDetector coralSensor;
    private GamePieceDetector algaeSensor;
    
    private Animation m_toAnimate = null;
    LimeLight limeLightFront;
    public boolean animationOff = false;    

    public enum AnimationTypes {
        ColorFlow,
        Fire,
        Larson,
        Rainbow,
        RgbFade,
        SingleFade,
        Strobe,
        Twinkle,
        TwinkleOff,
        SetAll,
    }


    public enum AvailableColors{
        TeamColors,
        Red,
        Yellow,
        Green
    }
    private AnimationTypes m_currentAnimation;

    public CANdleSystem(XboxController joy, GamePieceDetector coralSensor, GamePieceDetector algaeSensor, LimeLight front) {
        //308 LEDS Total
        this.joystick = joy;
        changeAnimation(AnimationTypes.SetAll);
        CANdleConfiguration configAll = new CANdleConfiguration();
        configAll.statusLedOffWhenActive = true;
        configAll.disableWhenLOS = false;
        configAll.stripType = LEDStripType.GRB; 
        configAll.brightnessScalar = 0.1;
        configAll.vBatOutputMode = VBatOutputMode.Modulated;
        m_candle.configAllSettings(configAll, 100);
        this.coralSensor = coralSensor;
        this.algaeSensor = algaeSensor;
        this.limeLightFront = front;
        showGreen();
    }


    public void flashColor(AvailableColors color){
        switch (color){
            case TeamColors: showTeamColors(); break;
            case Red: showRed(); break;
            case Yellow: showYellow(); break;
            case Green: showGreen(); break;

        }
    }

    public void showTeamColors(){
        //blue
        m_candle.setLEDs(0,0,225,0,0,128);
        m_candle.setLEDs(0,0,225,0,256,128);
        
        //yellow
        //m_candle.setLEDs(225,215,0,0,2,2);
        //m_candle.setLEDs(225,215,0,0,4,2);
    }
    public void showRed() {
        m_candle.setLEDs(255, 0, 0);
    }
    public void showRed(int start, int end) {
        m_candle.setLEDs(255,0,0,0,start,end);
    }
    public void showYellow() {
        m_candle.setLEDs(255,255,0);
    }
    public void showYellow(int start, int end) {
        m_candle.setLEDs(255,255,0, 0, start, end);
    }
    public void showGreen() {
        m_candle.setLEDs(0,128,0);
    }
    public void showGreen(int start, int end) {
        m_candle.setLEDs(0,128,0,0,start,end);
    }
    public void showBlue(int start, int end){
        m_candle.setLEDs(0,0,255,0,start,end);
    }
    public void showBlue(){
        m_candle.setLEDs(0,0,255);
    }
    public void showMagenta(int start, int end){
        m_candle.setLEDs(255, 0, 255,0,start, end);
    }
    public void showMagenta(){
        m_candle.setLEDs(255, 0, 255);
    }

    public void showLightBlue(int start, int end){
        m_candle.setLEDs(173, 216, 230, 0, start, end);
    }
    public void showLightBlue(){
        m_candle.setLEDs(173, 216, 230);

    }
    public void showWhite(int start, int end){
        m_candle.setLEDs(255, 255, 255, 0, start, end);
    }
    public void showWhite(){
        m_candle.setLEDs(255, 255, 255);
    }
    public void turnOffColors() {
        m_candle.setLEDs(0,0,0);
        changeAnimation(AnimationTypes.SetAll);
        
    }
    public void turnOffColors(int start, int end){
        m_candle.setLEDs(0, 0, 0, 0, start, end);
    }
    public void turnOffAnimation(){  
        changeAnimation(AnimationTypes.SetAll);
    }

    public void autoAlignStatus(AutoAlign autoAlign){
        if (autoAlign.isFinished()){
            m_candle.setLEDs(0, 255, 0);
        } else{
            turnOffColors();
        }
    }
    public void coralColors(boolean isOn){
        int r = 0;
        int g = isOn ? 128 : 0;
        int b = 0;
        m_candle.setLEDs(r,g,b,0,8,103);
        m_candle.setLEDs(r,g,b,0,3,2);
        
        // showGreen(8,103);
        // showGreen(3,2);
    }
    public  void coralOff(){
        turnOffColors(8,103);
    }
    public void algaeColors(boolean isOn){
        //showBlue(0,20);
        int r = 0;
        int g = 0;
        int b = isOn ? 255 : 0;
        m_candle.setLEDs(r,g,b,0,207,308);
        m_candle.setLEDs(r,g,b,0,5,1);
        m_candle.setLEDs(r,g,b,0,2,1);
        // showBlue(207,308);
        // showBlue(5,1);
        // showBlue(2,1);

    }
    public void algaeOff(){
        turnOffColors(207,102);
    }

    public void limeLightStatusColors(int tagAmount){
        int start = 103;
        int count = 104;
        int candleStart1 = 0;
        int candleCount1 = 2;

        int candleStart2 = 6;
        int candleCount2 = 2;
        if (animationOff){

        if (tagAmount == 0){
            showRed(start, count);
            showRed(candleStart1, candleCount1);
            showRed(candleStart2, candleCount2);
        } else if (tagAmount == 1){
            showYellow(start,count); //Yellow
            showYellow(candleStart1, candleCount1);
            showYellow(candleStart2, candleCount2);
        } else if (tagAmount == 2){
            showMagenta(start,count);
            showMagenta(candleStart1, candleCount1);
            showMagenta(candleStart2, candleCount2);
        } else if (tagAmount == 3){
            showLightBlue(start,count);
            showLightBlue(candleStart1, candleCount1);
            showLightBlue(candleStart2, candleCount2);
        } 
        }
    }

    


    public void incrementAnimation() {
        switch(m_currentAnimation) {
            case ColorFlow: changeAnimation(AnimationTypes.Fire); break;
            case Fire: changeAnimation(AnimationTypes.Larson); break; //DAVID SUGGESTION
            case Larson: changeAnimation(AnimationTypes.Rainbow); break;
            case Rainbow: changeAnimation(AnimationTypes.RgbFade); break;
            case RgbFade: changeAnimation(AnimationTypes.SingleFade); break;
            case SingleFade: changeAnimation(AnimationTypes.Strobe); break;
            case Strobe: changeAnimation(AnimationTypes.Twinkle); break;
            case Twinkle: changeAnimation(AnimationTypes.TwinkleOff); break;
            case TwinkleOff: changeAnimation(AnimationTypes.ColorFlow); break; //DAVID SUGGESTION
            case SetAll: changeAnimation(AnimationTypes.ColorFlow); break;
        }
    }
    public void decrementAnimation() {
        switch(m_currentAnimation) {
            case ColorFlow: changeAnimation(AnimationTypes.TwinkleOff); break;
            case Fire: changeAnimation(AnimationTypes.ColorFlow); break;
            case Larson: changeAnimation(AnimationTypes.Fire); break;
            case Rainbow: changeAnimation(AnimationTypes.Larson); break;
            case RgbFade: changeAnimation(AnimationTypes.Rainbow); break;
            case SingleFade: changeAnimation(AnimationTypes.RgbFade); break;
            case Strobe: changeAnimation(AnimationTypes.SingleFade); break;
            case Twinkle: changeAnimation(AnimationTypes.Strobe); break;
            case TwinkleOff: changeAnimation(AnimationTypes.Twinkle); break;
            case SetAll: changeAnimation(AnimationTypes.ColorFlow); break;
        }
    }
    public void setColors() {
        changeAnimation(AnimationTypes.SetAll);
    }

    /* Wrappers so we can access the CANdle from the subsystem */
    public double getVbat() { return m_candle.getBusVoltage(); }
    public double get5V() { return m_candle.get5VRailVoltage(); }
    public double getCurrent() { return m_candle.getCurrent(); }
    public double getTemperature() { return m_candle.getTemperature(); }
    public void configBrightness(double percent) { m_candle.configBrightnessScalar(percent, 0); }
    public void configLos(boolean disableWhenLos) { m_candle.configLOSBehavior(disableWhenLos, 0); }
    public void configLedType(LEDStripType type) { m_candle.configLEDType(type, 0); }
    public void configStatusLedBehavior(boolean offWhenActive) { m_candle.configStatusLedState(offWhenActive, 0); }

    public void changeAnimation(AnimationTypes toChange) {
        m_currentAnimation = toChange;
        
        switch(toChange)
        {
            case ColorFlow:
                m_toAnimate = new ColorFlowAnimation(128, 20, 70, 0, 0.7, LedCount, Direction.Forward);
                break;
            case Fire:
                m_toAnimate = new FireAnimation(0.5, 0.7, LedCount, 0.7, 0.5);
                m_toAnimate = new FireAnimation();
                break;
            case Larson:
                m_toAnimate = new LarsonAnimation(0, 255, 46, 0, 2, LedCount, BounceMode.Front, 20);
                break;
            case Rainbow:
                m_toAnimate = new RainbowAnimation(1, 0.1, LedCount);
                break;
            case RgbFade:
                m_toAnimate = new RgbFadeAnimation(0.7, 0.4, LedCount);
                break;
            case SingleFade:
                m_toAnimate = new SingleFadeAnimation(50, 2, 200, 0, 0.5, LedCount);
                break;
            case Strobe:
                m_toAnimate = new StrobeAnimation(240, 10, 180, 0, 98.0 / 256.0, LedCount);
                break;
            case Twinkle:
                m_toAnimate = new TwinkleAnimation(30, 70, 60, 0, 0.4, LedCount, TwinklePercent.Percent6);
                break;
            case TwinkleOff:
                m_toAnimate = new TwinkleOffAnimation(70, 90, 175, 0, 0.8, LedCount, TwinkleOffPercent.Percent100);
                break;
            case SetAll:
                m_toAnimate = null;
                break;
        }
        m_candle.animate(m_toAnimate);
        System.out.println("Changed to " + m_currentAnimation.toString());
    }



    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        /*if(m_toAnimate == null) {
            m_candle.setLEDs((int)(joystick.getLeftTriggerAxis() * 255), 
                              (int)(joystick.getRightTriggerAxis() * 255), 
                              (int)(joystick.getLeftX() * 255));
        } else {
            m_candle.animate(m_toAnimate);
        }
        m_candle.modulateVBatOutput(joystick.getRightY());*/
        
        /*if (algaeSensor.getIsGamePieceDetected()){
            algaeColors();
        } else{
            algaeOff();

        }
            */
        /*if (coralSensor.getIsGamePieceDetected()){
            coralColors();
        } else{
            coralOff();
        }*/
        
        //limeLightStatusColors(LimelightHelpers.getTargetCount(limeLightFront.limelightName));
        
        
        
        

    }

    @Override
    public void simulationPeriodic() {
        // turnOffColors(8,103);
        // turnOffColors(2,4);
        // turnOffColors(206,100);
        // if(coralSensor.getIsGamePieceDetected()){
        //     coralColors();
        // }else{
        //     turnOffColors(3,1); 
        //     turnOffColors(4,1);
        // }
        // if(algaeSensor.getIsGamePieceDetected()){
        //     algaeColors();
        // }else{
        //     m_candle.setLEDs(0,0,255,0,2,1); 
        //     turnOffColors(5,1);
        // }
        // //limelight color
        // if(true){
        // limeLightStatusColors(limeLightFront.getTagCount(), 6, 2);
        // limeLightStatusColors(limeLightFront.getTagCount(), 0, 2);
        // }

    }
}
