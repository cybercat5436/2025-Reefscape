// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.commands;
// import edu.wpi.first.wpilibj.Timer;
// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.subsystems.CANdleSystem;

// /* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
// public class CANdleTimer extends Command {
//   private CANdleSystem candleSystem = CANdleSystem.getInstance();

//   private Timer timer = new Timer();
  
//   /** Creates a new CANdleTimer. */
//   public CANdleTimer() {
//     // Use addRequirements() here to declare subsystem dependencies.
    
//   }

//   // Called when the command is initially scheduled.
//   @Override
//   public void initialize() {
//     timer.reset();
//     timer.start();
//   }

//   // Called every time the scheduler runs while the command is scheduled.
//   @Override
//   public void execute() {}

//   // Called once the command ends or is interrupted.
//   @Override
//   public void end(boolean interrupted) {
//     timer.stop();
//     candleSystem.turnOffColors();
//   }

//   // Returns true when the command should end.
//   @Override
//   public boolean isFinished() 
//   {
//     if (timer.get() > 4){
//       return true;
//     }
//     return false;
//   }
// }
