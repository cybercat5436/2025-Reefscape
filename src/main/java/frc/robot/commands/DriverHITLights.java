package frc.robot.commands;

import frc.robot.subsystems.GamePieceDetector;
import frc.robot.subsystems.PhotonVision;
import frc.robot.subsystems.CANdleSystem;

public class DriverHITLights {
    
    private final GamePieceDetector gamePieceDetector;
    private final PhotonVision photonVision;
    private final CANdleSystem canDleSystem;

    public DriverHITLights(GamePieceDetector gamePieceDetector, PhotonVision photonVision, CANdleSystem canDleSystem) {
        this.gamePieceDetector = gamePieceDetector;
        this.photonVision = photonVision;
        this.canDleSystem = canDleSystem;
    }

    public void controlLights() {
        // Check if both coral and tag are detected
        if (gamePieceDetector.isGamePieceClose() && photonVision.observedArea > 0) {
            System.out.println("Lights Green On");
            canDleSystem.flashColor(CANdleSystem.AvailableColors.Green);
        } 
        // If coral is no longer detected, turn off lights
        else if (!gamePieceDetector.isGamePieceClose()) {
            System.out.println("Lights off");
            canDleSystem.turnOffColors();
        } 
        else {
            System.out.println("Cant see tag and/or coral");
        }
    }
}
