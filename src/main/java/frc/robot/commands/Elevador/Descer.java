// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Elevador;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
// Certifique-se que está a importar a versão corrigida do subsistema
import frc.robot.subsystems.ElevadorS; 

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class Descer extends Command {
  // Usar a versão corrigida do subsistema
  public ElevadorS elevadorS = RobotContainer.elevadorS; 
  private double kPosition;
  // Definir uma tolerância aceitável para a posição final
  // Este valor depende da precisão necessária e das unidades do seu encoder.
  // Ajuste conforme necessário. Exemplo: 0.5 unidades de posição.
  private final double kPositionTolerance = 0.5; 

  public Descer(double position) 
  {
    kPosition = position;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(elevadorS);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() 
  {
    // Considerar se realmente quer resetar os encoders aqui.
    // Se o elevador já estiver numa posição conhecida, talvez não seja necessário.
    // elevadorS.resetEncoders(); 
    System.out.println("Comando Subir_Modificado iniciado para a posição: " + kPosition);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
    elevadorS.driveToPosition(kPosition);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) 
  {
    // Quando o comando termina (seja por concluir ou interrupção),
    // geralmente queremos parar os motores para evitar que continuem a tentar 
    // manter a posição ou causem desgaste.

  
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // Obter a posição atual de ambos os motores
    double positionDireito = elevadorS.getPositionDireito();
    double positionEsquerdo = elevadorS.getPositionEsquerdo();

    // Calcular o erro absoluto para cada motor
    double errorDireito = Math.abs(positionDireito - kPosition);
    double errorEsquerdo = Math.abs(positionEsquerdo - kPosition);

    // O comando termina APENAS QUANDO AMBOS os motores estiverem
    // dentro da tolerância da posição desejada.
    boolean direitoNaPosicao = errorDireito <= kPositionTolerance;
    boolean esquerdoNaPosicao = errorEsquerdo <= kPositionTolerance;

    return direitoNaPosicao && esquerdoNaPosicao;
  }
}

