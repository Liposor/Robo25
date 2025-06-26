// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.commands.Elevador.Subir;
import frc.robot.subsystems.Intake.Shooter;
import frc.robot.subsystems.Intake.Tracao;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ColetaFinal extends SequentialCommandGroup 
{
  public Coleta coleta1 = RobotContainer.Coleta;
  public Coleta2 coleta2 = RobotContainer.Coleta2;
  public Shooter shooter = RobotContainer.shooterA;
  public Tracaocmd tracaocmd = RobotContainer.tracaocmd;
  public Tracao tracao = RobotContainer.tracaoA;
  public PararShooter pararShooter = RobotContainer.pararShooter;
  /** Creates a new ColetaFinal. */
  public ColetaFinal() 
  {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new Subir(0.1).withTimeout(0.2),
      new Tracaocmd(tracao, -0.3).withTimeout(0.3),
      new Coleta(-0.3),
      new Coleta2(-0.1),
      new WaitCommand(0.15),
      new PararShooter(0),
      new Tracaocmd(tracao, -4.3)
      
      
      

    );
  }
}
