// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.io.IOException;
import org.json.simple.parser.ParseException;
import java.lang.annotation.Repeatable;
import java.time.Instant;
import java.time.temporal.TemporalAccessor;
import java.util.List;
import java.util.Optional;
import java.util.jar.Attributes.Name;

import javax.sound.sampled.SourceDataLine;

import org.photonvision.PhotonCamera;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.fasterxml.jackson.databind.util.Named;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import com.revrobotics.spark.SparkBase;

import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.AutoAlign;
import frc.robot.commands.CoralIntakeWithDetection;
import frc.robot.generated.TunerConstants;
// import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Climber2;
import frc.robot.subsystems.Algae;
import frc.robot.subsystems.CANdleSystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Coral;
import frc.robot.subsystems.GamePieceDetector;
import frc.robot.subsystems.CANdleSystem.AnimationTypes;
import frc.robot.subsystems.CANdleSystem.AvailableColors;
import frc.robot.subsystems.ReefController.ReefPosition;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.LimeLight;
import frc.robot.subsystems.PhotonVision;
import frc.robot.subsystems.PoseUpdater;
import frc.robot.subsystems.ReefController;

public class RobotContainer {
    private double maxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond) * 0.35; // kSpeedAt12Volts desired top speed
    private double HalfSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond) * 0.15;
    private double maxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond) * 0.5; // 3/4 of a rotation per second max angular velocity
    private double HalfAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond) *0.25;

    private double robotX;
    private double robotY;
    private double kP = 0.03;
    private double ySpeed = 0;
    private double xSpeed = 0;
    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(maxSpeed * 0.1).withRotationalDeadband(maxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.RobotCentric robotCentricDrive = new SwerveRequest.RobotCentric()
            .withDeadband(maxSpeed * 0.1).withRotationalDeadband(maxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
   
    private final Telemetry logger = new Telemetry(maxSpeed);
    private final PhotonVision photonVision = new PhotonVision();
    private final CommandXboxController joystick = new CommandXboxController(0);
    private final CommandXboxController joystick2 = new CommandXboxController(1);
    private final Joystick reefPositionJoystick = new Joystick(2);
    private final Joystick reefLevelJoystick = new Joystick(3);
   // private final ReefController reefController = new ReefController();
    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    private final LimeLight limeLightFront = new LimeLight("limelight-front", 0.02, -0.3, 0.65, -90.0, 0.0, 0.0);
    private final LimeLight limeLightFrontRight = new LimeLight("limelight-right", 0.15, 0.13, 0.45, 0.0, 0.0, 0.0);
    private final PoseUpdater poseUpdater = new PoseUpdater(limeLightFront, limeLightFrontRight, drivetrain);
    private final AutoAlign autoAlign = new AutoAlign(drivetrain,limeLightFront,photonVision);
    private SendableChooser<Command> autonChooser;
    // public final Climber climber = new Climber();
    public final Climber2 climber2 = new Climber2();
    public final GamePieceDetector coralSensor = new GamePieceDetector(35000, GamePieceDetector.Sensors.coral);
    public final GamePieceDetector algaeSensor = new GamePieceDetector(25000, GamePieceDetector.Sensors.algae, 0.1);
    
    public final Coral coral = new Coral();
    public final Algae algae = new Algae();
    public final Elevator elevator = new Elevator();
    public CANdleSystem candleSystem = new CANdleSystem(joystick.getHID());
    private SequentialCommandGroup autoCoralHigh = new SequentialCommandGroup(
        new InstantCommand(() -> elevator.raiseLevel4())
        ,Commands.waitSeconds(2.5)
        ,Commands.print("Auto Coral High")
        .andThen(new InstantCommand(() -> coral.backward(1)))
        , Commands.waitSeconds(.5)
         .andThen(new InstantCommand(() -> coral.stopMotor())))
         .andThen(new InstantCommand(() -> elevator.stopElevator()));
    // private Command autoCoralHigh = Commands.sequence(
    //     new InstantCommand(() -> elevator.raiseLevel4())
    //     ,Commands.print("done raising.")
    //     ,Commands.waitSeconds(2.5)
    //     ,Commands.print("Done waiting")
    //     ,(new InstantCommand(() -> coral.backward(1)))
    //     ,Commands.waitSeconds(.5)
    //     ,(new InstantCommand(() -> coral.stopMotor()))
    //     ,(new InstantCommand(() -> elevator.stopElevator()))


    // );


    public RobotContainer(){
        // autonChooser.addOption("Complex Auto", m_complexAuto);
        configureBindings();
        poseUpdater.enable();
        registerNamedCommands();
        //testReefController();
        autonChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auton Chooser", autonChooser);
    }

    

        //Command autoCoralHigh = Commands.sequence(new InstantCommand(() -> elevator.raiseLevel4()), new InstantCommand(() -> coral.forward(1)));





    

    private void registerNamedCommands(){
        NamedCommands.registerCommand("printSomething", new InstantCommand(() -> System.out.println(">>>>>>>>>>>>>Printing Something")));
        NamedCommands.registerCommand("autoCoralHigh", autoCoralHigh);
        NamedCommands.registerCommand("autoAlign", autoAlign);
        NamedCommands.registerCommand("raiseArmHigh", new InstantCommand(() -> elevator.raiseLevel4()));
        NamedCommands.registerCommand("shootCoral", new InstantCommand(() -> coral.backward(1)).andThen(Commands.waitSeconds(0.3)).andThen(new InstantCommand(() -> coral.stopMotor())));
        NamedCommands.registerCommand("lowerElevator", new InstantCommand(() -> elevator.raiseLevel1()).andThen(Commands.waitSeconds(2).andThen(new InstantCommand(() -> elevator.stopElevator()))));
        NamedCommands.registerCommand("coralIntake", new CoralIntakeWithDetection(coral, coralSensor));
        NamedCommands.registerCommand("stopCoralIntake", new InstantCommand(() -> coral.forward(0)));
        NamedCommands.registerCommand("setClimberArmsToAutonStartPosition", 
        new InstantCommand(() -> climber2.climberAutonStartPosition())
        .andThen(Commands.waitSeconds(0.5))
        .andThen(new InstantCommand(() -> climber2.stopClimb())
        ));
    }


    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        
        Trigger algaeIntakeTrigger = new Trigger ((joystick2.rightTrigger()));
        Trigger coralIntakeTrigger = new Trigger ((joystick2.leftTrigger()));


        coralIntakeTrigger
            .whileTrue(new InstantCommand(() -> coral.backward(1)))
            .onFalse(new InstantCommand(() -> coral.stopMotor()));
        
        algaeIntakeTrigger
            .whileTrue(new InstantCommand(() -> algae.intakeBall(1)))
            .onFalse(new InstantCommand(() -> algae.stopBallMotor()));

        joystick2.leftBumper()
           .whileTrue(new InstantCommand(() -> coral.forward(1)))
           .onFalse(new InstantCommand(() -> coral.stopMotor())); 
        joystick2.rightBumper()
           .whileTrue(new InstantCommand(() -> algae.intakeBall(-1)))
           .onFalse(new InstantCommand(() -> algae.stopBallMotor()));
        
        

        
        // joystick2.a()
        //     .whileTrue(new InstantCommand(() -> elevator.raise()))
        //     .onFalse(new InstantCommand(() -> elevator.stopElevator()));
        // joystick2.x()
        //     .whileTrue(new InstantCommand(() -> elevator.lower()))
        //     .onFalse(new InstantCommand(() -> elevator.stopElevator()));
        joystick2.x()
            .onTrue(new InstantCommand(() -> elevator.raiseLevel1())
            .andThen(Commands.waitSeconds(1.5))
            .andThen(new InstantCommand(() -> elevator.stopElevator())));
        joystick2.a()
            .onTrue(new InstantCommand(() -> elevator.raiseLevel2()));
        joystick2.b()
            .onTrue(new InstantCommand(() -> elevator.raiseLevel3()));
        joystick2.y()
            .onTrue(new InstantCommand(() -> elevator.raiseLevel4()));
           
       

        // joystick2.povUp().and(joystick2.rightBumper())
        //     .whileTrue(new InstantCommand(() -> algae.releaseBall(-0.3))) 
        //     .onFalse(new InstantCommand(() -> algae.stopBallMotor()));
        // joystick2.povRight().and(joystick2.rightBumper())
        //     .whileTrue(new InstantCommand(() -> algae.releaseBall(-0.3))) 
        //     .onFalse(new InstantCommand(() -> algae.stopBallMotor()));
        //joystick2.povLeft()
         // .whileTrue(new InstantCommand(() -> algae.releaseBall(-0.3))) 
         //  .onFalse(new InstantCommand(() -> algae.stopBallMotor()));

        // joystick2.povUp()
        //     .whileTrue(new InstantCommand(() -> algae.algaeHigh()).repeatedly())
        //     .onFalse(new InstantCommand(() -> algae.algaeStop()));
        // joystick2.povLeft()
        // .onTrue(autoCoralHigh);

        joystick2.povUp()
            .whileTrue(new InstantCommand(() -> algae.algaeHigh(-0.3)).repeatedly())
            .onFalse(new InstantCommand(() -> algae.algaeStop()));
        joystick2.povDown()
            .whileTrue(new InstantCommand(() -> algae.algaeHigh(0.3)))
            .onFalse(new InstantCommand(() -> algae.algaeStop()));
        // joystick2.povUp()
        //     .onTrue(new InstantCommand(() -> algae.algaeHigh()))
        //     .onFalse(new InstantCommand(() -> algae.algaeStop()));
        // joystick2.povRight()
        //     .onTrue(new InstantCommand(() -> algae.algaeLow()))
        //     .onFalse(new InstantCommand(() -> algae.algaeStop()));
        // joystick2.povLeft()
        //     .onTrue(new InstantCommand(() -> algae.algaeProcessor()))
        //     .onFalse(new InstantCommand(() -> algae.algaeStop()));
            
        
        

        SlewRateLimiter slewRateLimiterX = new SlewRateLimiter(maxSpeed * 2);  //Note: setting slewratelimiter to 2x speed means it takes 0.5s to accelerate to full speed
        SlewRateLimiter slewRateLimiterY = new SlewRateLimiter(maxSpeed * 2);
        SlewRateLimiter slewRateLimiterTurnX = new SlewRateLimiter(maxAngularRate * 2);  //corrected from using MaxSpeed
        
        SlewRateLimiter slowModeSlewRateLimiterX = new SlewRateLimiter(maxSpeed * 5);  //Note: setting slewratelimiter to 2x speed means it takes 0.5s to accelerate to full speed
        SlewRateLimiter slowModeSlewRateLimiterY = new SlewRateLimiter(maxSpeed * 5);
        SlewRateLimiter slowModeSlewRateLimiterTurnX = new SlewRateLimiter(maxAngularRate * 3);  //corrected from using MaxSpeed
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->{
                double xSpeed = slewRateLimiterX.calculate(joystick.getLeftY()* maxSpeed);
                
                double ySpeed = slewRateLimiterY.calculate(joystick.getLeftX()* maxSpeed);
                
                double yTurnSpeed = slewRateLimiterTurnX.calculate(joystick.getRightX()* maxAngularRate);

                SmartDashboard.putNumber("xSpeed",xSpeed);
                SmartDashboard.putNumber("ySpeed",ySpeed);
                SmartDashboard.putNumber("yTurnSpeed",yTurnSpeed);

                return drive.withVelocityX(xSpeed * Math.abs(xSpeed)) // Drive forward with negative Y (forward)
                    .withVelocityY(ySpeed * Math.abs(ySpeed)) // Drive left with negative X (left)
                    .withRotationalRate(-(yTurnSpeed * Math.abs(yTurnSpeed))); // Drive counterclockwise with negative X (left)
            }
            )
        );
    

    //     joystick.x().whileTrue(drivetrain.applyRequest(() -> {
    //     robotX = limeLightFront.getVisionArea();
    //     robotY = -limeLightFront.getVisionTargetHorizontalError();
    //     xSpeed = robotX * kP * maxSpeed;
    //     ySpeed = robotY * kP * maxSpeed;
    //         SmartDashboard.putNumber("robot y velocity", joystick.getLeftX() * maxSpeed);
    //     return robotCentricDrive.withVelocityX(0) // Drive forward with negative Y (forward)
    //         .withVelocityY(ySpeed) // Drive left with negative X (left)
    //         .withRotationalRate(-joystick.getRightX() * maxAngularRate); // Drive counterclockwise with negative X (left)
        
    // })
    // );
    joystick.b().whileTrue(drivetrain.applyRequest(() -> {
            double xSpeed = slewRateLimiterX.calculate(-joystick.getLeftY()* maxSpeed);             
            double ySpeed = slewRateLimiterY.calculate(-joystick.getLeftX()* maxSpeed);          
            double yTurnSpeed = slewRateLimiterTurnX.calculate(joystick.getRightX()* maxAngularRate);

        return robotCentricDrive.withVelocityX(xSpeed * Math.abs(xSpeed)) 
        .withVelocityY(ySpeed * Math.abs(ySpeed)) 
        .withRotationalRate(-(yTurnSpeed * Math.abs(yTurnSpeed)));
        }
    ));
        
        Trigger slowModeTrigger = new Trigger ((joystick.leftTrigger()));

        slowModeTrigger.whileTrue(drivetrain.applyRequest(() ->{
            
                double xSpeed = slowModeSlewRateLimiterX.calculate(joystick.getLeftY()* HalfSpeed);
                
                
                double ySpeed = slowModeSlewRateLimiterY.calculate(joystick.getLeftX()* HalfSpeed);
                
                
                double yTurnSpeed = slowModeSlewRateLimiterTurnX.calculate(joystick.getRightX()* HalfAngularRate);
                SmartDashboard.putNumber("RBySpeed",ySpeed);
                SmartDashboard.putNumber("RBxSpeed",xSpeed);
                SmartDashboard.putNumber("RByTurnSpeed",yTurnSpeed);

        return drive.withVelocityX(xSpeed * Math.abs(xSpeed)) // Drive forward with negative Y (forward)
            .withVelocityY(ySpeed * Math.abs(ySpeed)) // Drive left with negative X (left)
            .withRotationalRate(-(yTurnSpeed * Math.abs(yTurnSpeed) ));
        } // Drive counterclockwise with negative X (left)
    ));
    
        joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
        joystick.x().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))
        ));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // reset the field-centric heading on left bumper press
        joystick.back().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));
    
        joystick.y().whileTrue(new AutoAlign(drivetrain, limeLightFront, photonVision));

        // climber commands
        
        drivetrain.registerTelemetry(logger::telemeterize);
        SmartDashboard.putData("reset odymetry to 0,0",new InstantCommand(() -> drivetrain.resetPose(new Pose2d(0.0,0.0, new Rotation2d()))));
        
        Trigger rightClimbUpTrigger = new Trigger (() -> (joystick2.getRightY() < -0.2));
        Trigger rightClimbDownTrigger = new Trigger(() -> (joystick2.getRightY() > 0.2));
        Trigger leftClimbUpTrigger = new Trigger(() -> (joystick2.getLeftY() < -0.2));
        Trigger leftClimbDownTrigger = new Trigger(() -> (joystick2.getLeftY() > 0.2));
        Trigger rightClimberStopTrigger = new Trigger(() -> joystick2.getRightY() >= -0.2 && joystick2.getRightY() <= 0.2);
        Trigger leftClimberStopTrigger = new Trigger(() -> joystick2.getLeftY() >= -0.2 && joystick2.getLeftY() <= 0.2);

        leftClimbUpTrigger
            .whileTrue(new InstantCommand(() -> climber2.rightClimb(0.2)).repeatedly());
        rightClimbUpTrigger
            .whileTrue(new InstantCommand(() -> climber2.leftClimb(0.2)).repeatedly());
        leftClimbDownTrigger
            .whileTrue(new InstantCommand(() -> climber2.rightClimb(-0.2)).repeatedly());
        rightClimbDownTrigger
            .whileTrue(new InstantCommand(() -> climber2.leftClimb(-0.2)).repeatedly());
        rightClimberStopTrigger
            .whileTrue(new InstantCommand(() -> climber2.leftClimb(0)));
        leftClimberStopTrigger
            .whileTrue(new InstantCommand(() -> climber2.rightClimb(0)));
        joystick.povLeft().onTrue(new InstantCommand(() -> candleSystem.changeAnimation(AnimationTypes.Fire)));
        joystick.povDown().onTrue(new InstantCommand(() -> candleSystem.turnOffColors()));
        joystick.povUp().onTrue(new InstantCommand(() -> candleSystem.showTeamColors()));
        
        SmartDashboard.putData(" Climber arms to auton start position",
         new InstantCommand(() -> climber2.climberAutonStartPosition())
         .andThen(Commands.waitSeconds(0.5))
         .andThen(new InstantCommand(() -> climber2.stopClimb())
         ));
        
        /*joystick.povRight()
            .onTrue(new InstantCommand(() -> candleSystem.flashColor(AvailableColors.Red))
            .andThen(new WaitCommand(0.5))
            .andThen(new InstantCommand(()->candleSystem.turnOffColors()))
            .andThen(new WaitCommand(0.25))
            .andThen(new InstantCommand(() -> candleSystem.flashColor(AvailableColors.Yellow)))
            .andThen(new WaitCommand(0.5))
            .andThen(new InstantCommand(()->candleSystem.turnOffColors()))
            .andThen(new WaitCommand(0.25))
            .andThen(new InstantCommand(() -> candleSystem.flashColor(AvailableColors.Green)))
            .andThen(new WaitCommand(0.5))
            .andThen(new InstantCommand(()->candleSystem.turnOffColors()))
            );*/

        Command blinkLight = Commands.repeatingSequence(
            new InstantCommand(() -> candleSystem.flashColor(AvailableColors.Red)),
            new WaitCommand(0.5),
            new InstantCommand(()->candleSystem.turnOffColors()),
            new WaitCommand(0.25)
        );


        //joystick.povRight().onTrue(new ParallelRaceGroup(blinkLight, new WaitCommand(2.25)));
        //joystick.povRight().onTrue(new ColorBlinkCommand(AvailableColors.Red, candleSystem));
    //     Trigger button1 = new Trigger(() -> reefPositionJoystick.getRawButton(1));
    //     button1.onTrue(new InstantCommand(() ->reefController.setTargetReefPosition(ReefPosition.A)));
    //     Trigger button2 = new Trigger(() -> reefPositionJoystick.getRawButton(2));
    //     button2.onTrue(new InstantCommand(() ->reefController.setTargetReefPosition(ReefPosition.B)));
    //     Trigger button3 = new Trigger(() -> reefPositionJoystick.getRawButton(3));
    //     button3.onTrue(new InstantCommand(() ->reefController.setTargetReefPosition(ReefPosition.C)));
    //     Trigger button4 = new Trigger(() -> reefPositionJoystick.getRawButton(4));
    //     button4.onTrue(new InstantCommand(() ->reefController.setTargetReefPosition(ReefPosition.D)));
    //     Trigger button5 = new Trigger(() -> reefPositionJoystick.getRawButton(5));
    //     button5.onTrue(new InstantCommand(() ->reefController.setTargetReefPosition(ReefPosition.E)));
    //     Trigger button6 = new Trigger(() -> reefPositionJoystick.getRawButton(6));
    //     button6.onTrue(new InstantCommand(() ->reefController.setTargetReefPosition(ReefPosition.F)));
    //     Trigger button7 = new Trigger(() -> reefPositionJoystick.getRawButton(7));
    //     button7.onTrue(new InstantCommand(() ->reefController.setTargetReefPosition(ReefPosition.G)));
    //     Trigger button8 = new Trigger(() -> reefPositionJoystick.getRawButton(8));
    //     button8.onTrue(new InstantCommand(() ->reefController.setTargetReefPosition(ReefPosition.H)));
    //     Trigger button9 = new Trigger(() -> reefPositionJoystick.getRawButton(9));
    //     button9.onTrue(new InstantCommand(() ->reefController.setTargetReefPosition(ReefPosition.I)));
    //     Trigger button10 = new Trigger(() -> reefPositionJoystick.getRawButton(10));
    //     button10.onTrue(new InstantCommand(() ->reefController.setTargetReefPosition(ReefPosition.J)));
    //     Trigger button11 = new Trigger(() -> reefPositionJoystick.getRawButton(11));
    //     button11.onTrue(new InstantCommand(() ->reefController.setTargetReefPosition(ReefPosition.K)));
    //     Trigger button12 = new Trigger(() -> reefPositionJoystick.getRawButton(12));
    //     button12.onTrue(new InstantCommand(() ->reefController.setTargetReefPosition(ReefPosition.L)));

       
    //     Trigger buttonLevel1Trigger = new Trigger(() -> reefLevelJoystick.getRawButton(1));
    //     buttonLevel1Trigger.onTrue(new InstantCommand(() ->reefController.setTargetReefLevel(1)));
    //     Trigger buttonLevel2Trigger = new Trigger(() -> reefLevelJoystick.getRawButton(2));
    //     buttonLevel2Trigger.onTrue(new InstantCommand(() ->reefController.setTargetReefLevel(2)));
    //     Trigger buttonLevel3Trigger = new Trigger(() -> reefLevelJoystick.getRawButton(3));
    //     buttonLevel3Trigger.onTrue(new InstantCommand(() ->reefController.setTargetReefLevel(3)));
    //     Trigger buttonLevel4Trigger = new Trigger(() -> reefLevelJoystick.getRawButton(4));
    //     buttonLevel4Trigger.onTrue(new InstantCommand(() ->reefController.setTargetReefLevel(4)));



    }

    // private void testReefController(){
    //     reefController.setTargetReefPosition(ReefPosition.I);
    //     System.out.println(reefController.toString());

    //     reefController.setTargetReefPosition(ReefPosition.K);
    //     System.out.println(reefController.toString());

    //     reefController.setTargetReefPosition(ReefPosition.L);
    //     System.out.println(reefController.toString());

    //     reefController.setTargetReefPosition(ReefPosition.G);
    //     System.out.println(reefController.toString());
        
    //     reefController.setTargetReefPosition(ReefPosition.H);
    //     System.out.println(reefController.toString());        
    // }
    

    public void setOdometryPoseFromSelectedAuto(String autonName){
        System.out.println("~~~~Setting pose from Selected Auto: " + autonName);
        Pose2d startPose = new Pose2d();
        
        try {
    
          List<PathPlannerPath> paths = PathPlannerAuto.getPathGroupFromAutoFile(autonName);
          startPose = getFirstPose(paths);
    
        } catch (IOException e){
          // Exception thrown if starting pose is null in PathPlanner Auton file
          System.out.println("IOException No starting pose detected in Auton file!");
        } catch (ParseException e){
            // Exception thrown if starting pose is null in PathPlanner Auton file
            System.out.println("ParseException No starting pose detected in Auton file!");
        }
        System.out.println("~~~~ New starting pose: " + startPose.toString());
        drivetrain.resetPose(startPose);
    }

    public Alliance getAlliance(){
        if(!DriverStation.isDSAttached()){
            // System.out.println("~~~~~" + DriverStationSim.getAllianceStationId());
            // *** NOTE: if using simGUI set alliance manually here ****
            // System.out.println(String.format("isRedAlliance: %s", 
            // NetworkTableInstance.getDefault().getTable("FMSInfo").getEntry("IsRedAlliance").getBoolean(false)
            // ));
            boolean isRed = NetworkTableInstance.getDefault().getTable("FMSInfo").getEntry("IsRedAlliance").getBoolean(false);
            return isRed? Alliance.Red : Alliance.Blue;
        }
        return DriverStation.getAlliance().orElse(Alliance.Blue);
    }

    private Pose2d getFirstPose(List<PathPlannerPath> paths){
        Pose2d firstPose = new Pose2d();
        // Guard against empty paths list
        if(paths.isEmpty()){
            System.out.println("Auto has no paths, returing origin");
            return new Pose2d();
        }


        System.out.println("~~~~ Is Driver Station Attached: " + DriverStation.isDSAttached());
        // figure out if path flipping has to happen
        boolean shouldFlip = (getAlliance() == DriverStation.Alliance.Red);
        System.out.println("~~~~~ Should Flip Path: " + shouldFlip);

        PathPlannerPath firstPath = shouldFlip ? paths.get(0).flipPath() : paths.get(0);
        // get the ideal starting state
        var firstRotation = firstPath.getIdealStartingState().rotation();
        // use position from first pose and add ideal starting state
        firstPose = new Pose2d(firstPath.getAllPathPoints().get(0).position, firstRotation);

        return firstPose;
    }
        

    public Command getAutonomousCommand() {
        //return Commands.print("No autonomous command configured");
        
        return autonChooser.getSelected();
        
    }



    public void resetGyroAfterAuton() {
        drivetrain.setPigeonHeadingToOdometry();
    }

   
}

