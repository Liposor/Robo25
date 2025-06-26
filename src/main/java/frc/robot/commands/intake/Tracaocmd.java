// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Intake; // Ajuste o pacote conforme a estrutura do seu projeto

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake.Tracao;

public class Tracaocmd extends Command {

  private final Tracao positionSubsystem;
  private final double targetPosition;

  // Tolerância para considerar que a posição foi atingida (em rotações)
  // !!! AJUSTE ESTE VALOR CONFORME A PRECISÃO NECESSÁRIA !!!
  private final double kPositionTolerance = 0.20; 

  /**
   * Cria um novo comando para mover o PositionControlledSubsystem para uma posição específica.
   *
   * @param subsystem A instância do PositionControlledSubsystem a ser controlada.
   * @param position A posição alvo em rotações.
   */
  public Tracaocmd(Tracao subsystem, double position) {
    this.positionSubsystem = subsystem;
    this.targetPosition = position;
    
    // Declara o subsistema como requisito
    addRequirements(this.positionSubsystem);
  }

  // Chamado quando o comando é agendado pela primeira vez.
  @Override
  public void initialize() {
    System.out.println("MoveToPositionCommand Initialized: Target = " + targetPosition);
    // Comanda o subsistema para ir para a posição alvo
    positionSubsystem.goToPosition(targetPosition);
  }

  // Chamado repetidamente enquanto o comando está agendado.
  @Override
  public void execute() {
    // O comando PositionVoltage continua a ser executado pelo TalonFX.
    // Poderíamos reenviar o comando aqui, mas geralmente não é necessário
    // positionSubsystem.goToPosition(targetPosition);
    
    // Poderia adicionar lógica aqui se necessário, como verificar limites.
  }

  // Chamado uma vez quando o comando termina ou é interrompido.
  @Override
  public void end(boolean interrupted) {
    if (interrupted) {
      System.out.println("MoveToPositionCommand Interrupted.");
      // Se interrompido, pode ser útil parar explicitamente ou deixar o comando padrão assumir.
      // positionSubsystem.stopMotor(); // Opcional: parar explicitamente
    } else {
      System.out.println("MoveToPositionCommand Finished. Reached target: " + targetPosition);
      // Ao terminar normalmente (atingiu a posição), geralmente não fazemos nada aqui,
      // pois queremos que o comando padrão (se houver) mantenha a posição.
      // Se não houver comando padrão, pode ser necessário chamar stopMotor() para manter.
      // positionSubsystem.stopMotor(); // Chama stopMotor que mantém a posição atual
    }
  }

  // Retorna true quando o comando deve terminar.
  @Override
  public boolean isFinished() {
    // Verifica se a posição atual está dentro da tolerância da posição alvo.
    double currentPosition = positionSubsystem.getCurrentPosition();
    boolean reached = Math.abs(currentPosition - targetPosition) <= kPositionTolerance;
    // if (reached) {
    //   System.out.println("MoveToPositionCommand Reached Tolerance. Current: " + currentPosition);
    // }
    return reached;
  }
}

