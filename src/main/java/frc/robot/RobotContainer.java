// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;



//Java Imports
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;

//WPILIB Imports

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import edu.wpi.first.wpilibj2.command.button.Trigger;

//Robot Imports
import frc.robot.Constants.OIConstants.JoystickDriverConstants;
import frc.robot.Constants.OIConstants.MesinhaJoy1Constants;
import frc.robot.Constants.OIConstants.MesinhaJoy2Constants;

import frc.robot.Limelight.LimelightHelpers;
import frc.robot.Utils.OperatorHub;
import frc.robot.Utils.TouchScreenInterface;
import frc.robot.commands.Elevador.ElevadorFuncional;
import frc.robot.commands.Elevador.Subir;
import frc.robot.commands.Intake.Atirar;
import frc.robot.commands.Intake.AtirarGeral;
import frc.robot.commands.Intake.Coleta;
import frc.robot.commands.Intake.ColetaAlgae;
import frc.robot.commands.Intake.ColetaFinal;
import frc.robot.commands.Intake.Tracaocmd;
import frc.robot.commands.Intake.atirar2;
import frc.robot.commands.swerve.AimAndRangeCmd;
import frc.robot.commands.swerve.LeaveCmd;
import frc.robot.commands.swerve.TrackAlgae;
import frc.robot.subsystems.ElevadorS;
import frc.robot.subsystems.LedControl;
import frc.robot.subsystems.Intake.Shooter;
import frc.robot.subsystems.Intake.Tracao;
import frc.robot.subsystems.Swerve.SwerveSubsystem;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer 
{


  public static Shooter shooterA = new Shooter();
  public static Tracao tracaoA = new Tracao();
  public static ElevadorS elevadorS = new ElevadorS();

  public static Tracaocmd tracaocmd = new Tracaocmd(tracaoA, 0);

  public static frc.robot.commands.Intake.Coleta Coleta = new frc.robot.commands.Intake.Coleta(0);
  public static frc.robot.commands.Intake.Coleta2 Coleta2 = new frc.robot.commands.Intake.Coleta2(0);
  public static frc.robot.commands.Intake.PararShooter pararShooter = new frc.robot.commands.Intake.PararShooter(0);
  public static frc.robot.commands.Intake.Atirar atirar = new frc.robot.commands.Intake.Atirar(0.0);
  public static ColetaFinal coletaFinal = new ColetaFinal();
  public static ColetaAlgae coletaAlgae = new ColetaAlgae();
  public static Subir subirElevador = new Subir(0);
  public static AtirarGeral atirarGeral = new AtirarGeral();
  public static atirar2 atirar2 = new atirar2(0);

  public static TouchScreenInterface touchScreenInterface = new TouchScreenInterface();
  
  public static CommandXboxController driverJoystick = 
                                      new CommandXboxController(JoystickDriverConstants.kDriverControllerPort);
  
  public static Joystick mesinhaJoy1 = 
                          new Joystick(MesinhaJoy1Constants.kJoy1Port);
  public static Joystick mesinhaJoy2 = 
                          new Joystick(MesinhaJoy2Constants.kJoy2Port);

  public static final OperatorHub operatorHub = 
                                  new OperatorHub(mesinhaJoy1, mesinhaJoy2);
  /**
   * SUBSYSTEMS
   */
  //Put all subsystems here
  public static LimelightHelpers limelightHelpers = new LimelightHelpers();
  public static LedControl ledControl = new LedControl();
  
  //Swerve Drive
  public static SwerveSubsystem swerveDrive = new SwerveSubsystem();
  
  

  //Swerve commands
  public static TrackAlgae trackAlgaeCmd = new TrackAlgae(); 
  public static AimAndRangeCmd aimAndRangeCmd = new AimAndRangeCmd( );
  public static LeaveCmd leaveCmd = new LeaveCmd();
  
    
  /**
  * Declare class SendableChoser()  
  *  Pop up selection of option on the SmartDashBoard   
  */
  SendableChooser<Command> chooserAuto;



  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() 
  {
    NamedCommands.registerCommand("SubirL4", new ElevadorFuncional(3, -7));
    NamedCommands.registerCommand("Atirar", new AtirarGeral());
    NamedCommands.registerCommand("Alinhar", new InstantCommand(() -> swerveDrive.zeroHeading()));
    
       
    // Add commands to the subsystem
    
    //Swerve Commands
    trackAlgaeCmd.addRequirements(swerveDrive);
    aimAndRangeCmd.addRequirements(swerveDrive);
    leaveCmd.addRequirements(swerveDrive);
    
    //algaSubsystem.setDefaultCommand(algaeCoralManual);


    
    
    //Instantiate subsystems
    //swerveDrive 
    // Add the path to the auto chooser
   
    
    // Instantiate Xbox Controller
    
    
    

    //Default option
    /*chooserAuto.setDefaultOption("Autonomous 1", autoMundoSenai);
    chooserAuto.addOption("Autonomous 2", autoQuadrado);*/
   

    // Configure the button bindings
    configureBindings();

    //*******set default command to drive with joystick************/
     //The left stick controls translation of the robot.
     // Turning is controlled by the X axis of the right stick.
    
  
    /**
    * DEFAULT COMMANDS
    */
    //liftSubsystem.setDefaultCommand(liftCmd);

    //Runnable command
    swerveDrive.setDefaultCommand
                                  (new RunCommand
                                    (()-> swerveDrive.driveRobotOriented
                                      ( ()->driverJoystick.getLeftY(),
                                        ()->driverJoystick.getLeftX(),
                                        ()->-driverJoystick.getRightX(),
                                        ()->true
                                      ), 
                                      swerveDrive
                                    )//.onlyIf(()-> !driverJoystick.getHID().getXButton())
                          )
                                  ; 
    
    
    
                                                 
    
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() 
  {
     /**
      * XBOX CONTROLLER BUTTONS
      */
      driverJoystick.povDown().onTrue(new ElevadorFuncional(2, -4.3));
      driverJoystick.povUp().onTrue(new ElevadorFuncional(11.5, -9));
      driverJoystick.povLeft().onTrue(new ElevadorFuncional(23, -7));




      driverJoystick.y().onTrue(AutoBuilder.pathfindToPose(new Pose2d(4, 3, Rotation2d.fromDegrees(0)), new PathConstraints(3.0, 2.0, Math.toRadians(10), Math.toRadians(10))));

      //driverJoystick.x().onTrue(new ColetaAlgae());
      driverJoystick.b().onTrue(new AtirarGeral());



      driverJoystick.a().onTrue(new ColetaFinal());
     //Changes robot speed - speeding up
      driverJoystick.leftBumper().whileTrue(
                                            new InstantCommand(() -> {swerveDrive.robotSlower();
                                                                      ledControl.setWhiteLed();
                                                                      })); 
     driverJoystick.rightBumper().whileTrue(
                                            new InstantCommand(() -> {swerveDrive.robotFast();
                                                                      ledControl.setYellowLed();
                                                                      })); 
                                                                      //swerveDrive::zeroHeading


     driverJoystick.leftBumper().and(driverJoystick.rightBumper())  
                                .whileFalse(new InstantCommand(()->{swerveDrive.robotMaxSpeed();
                                                                    ledControl.setRedLed();
                                                                    })); 
     
    

    RobotContainer.driverJoystick.start().whileTrue(new InstantCommand(() -> swerveDrive.zeroHeading()));
    //Prepare to Pick Coral
     
   
     /**
      * OPERATOR CONTROLS
      */

    //JoystickButton leaveAlgaeReefHigh = new JoystickButton (mesinhaJoy2, operatorHub.kEleven);
    //leaveAlgaeReefHigh.whileTrue(freeAlgaeReefHighCmd);

    /*Trigger prepareClimb = new Trigger(()->(escaladorSubsystem.isDistanceValid() && !escaladorSubsystem.getSensorClimber()));
    prepareClimb.onTrue(new InstantCommand(ledControl::setGreenLed));
    prepareClimb.onTrue(new InstantCommand(() -> driverJoystick.setRumble(RumbleType.kBothRumble, 0.75)));
    prepareClimb.onFalse(new InstantCommand(ledControl::setRedLed));
    prepareClimb.onFalse(new InstantCommand(() -> driverJoystick.setRumble(RumbleType.kBothRumble, 0)));*/

   
  }
  

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // This method loads the auto when it is called, however, it is recommended
    // to first load your paths/autos when code starts, then return the
    // pre-loaded auto/path
    return new PathPlannerAuto("FabioAuto");
  }
  }


