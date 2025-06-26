// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Intake; // Ajuste o pacote conforme a estrutura do seu projeto

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Tracao extends SubsystemBase {

  private final int kMotorID = 22; // !!! AJUSTE O ID DO MOTOR CONFORME O SEU ROBÔ !!!
  private final String kCanbus = "rio"; // Ou o nome do seu CANivore

  private final TalonFX positionMotor;
  private final PositionVoltage positionVoltageControl = new PositionVoltage(0); // Inicializa com posição 0

  // --- Constantes de Configuração (AJUSTE ESTES VALORES!) ---
  // Ganhos PID (Slot 0 será usado para PositionVoltage)
  private final double kP = 0.7; // Ganho Proporcional - Ajuste este valor!
  private final double kI = 0.0; // Ganho Integral - Geralmente 0 para posição
  private final double kD = 0.08; // Ganho Derivativo - Ajuste se necessário (cuidado!)
  private final double kS = 0.0; // Feedforward Estático (Opcional, mas útil)
  private final double kV = 0.00013; // Feedforward de Velocidade (Opcional)

  // Limites de Corrente
  private final double kStatorCurrentLimit = 40.0; // Amperes
  private final boolean kEnableStatorCurrentLimit = true;

  // Relação do Encoder (se usar encoder externo ou diferente relação)
  private final double kSensorToMechanismRatio = 1.0; // Relação entre rotações do sensor e do mecanismo
  private final double kRotorToSensorRatio = 1.0; // Relação entre rotações do rotor e do sensor

  /** Creates a new PositionControlledSubsystem. */
  public Tracao() {
    positionMotor = new TalonFX(kMotorID, kCanbus);
    configureMotor();
  }

  private void configureMotor() {
    TalonFXConfiguration configs = new TalonFXConfiguration();

    // Configuração do Slot 0 para PositionVoltage
    Slot0Configs slot0 = configs.Slot0;
    slot0.kP = kP;
    slot0.kI = kI;
    slot0.kD = kD;
    slot0.kS = kS;
    slot0.kV = kV;

    // Configuração do Feedback (usando o encoder interno do TalonFX por padrão)
    configs.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor; // Ou RemoteCANcoder se usar um
    configs.Feedback.SensorToMechanismRatio = kSensorToMechanismRatio;
    configs.Feedback.RotorToSensorRatio = kRotorToSensorRatio;
    // Se usar CANcoder remoto: 
    // configs.Feedback.FeedbackRemoteSensorID = ID_DO_CANCODER;
    // configs.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
    // configs.Feedback.RotorToSensorRatio = RELACAO_ROTOR_PARA_CANCODER;

    // Modo Neutro (Brake ou Coast)
    configs.MotorOutput.NeutralMode = NeutralModeValue.Brake; // Ou Coast

    // Limites de Corrente
    CurrentLimitsConfigs currentLimits = configs.CurrentLimits;
    currentLimits.StatorCurrentLimit = kStatorCurrentLimit;
    currentLimits.StatorCurrentLimitEnable = kEnableStatorCurrentLimit;

    // Aplicar configurações
    positionMotor.getConfigurator().apply(configs);

    // Zerar a posição do encoder na inicialização (opcional, mas comum)
    positionMotor.setPosition(0.0);
    System.out.println("PositionControlledSubsystem Motor Configured. Initial Position set to 0.");
  }

  /**
   * Comanda o motor para ir para uma posição específica usando PositionVoltage.
   *
   * @param targetPosition A posição desejada em rotações.
   */
  public void goToPosition(double targetPosition) {
    positionMotor.setControl(positionVoltageControl.withPosition(targetPosition));
  }

  /**
   * Para o motor.
   * Pode ser configurado para ir para 0V (Coast) ou manter a posição atual.
   * Aqui, vamos simplesmente desativar o controlo (vai para modo neutro).
   */
  public void stopMotor() {
    positionMotor.setControl(positionVoltageControl.withPosition(getCurrentPosition())); // Mantém a posição atual
    // Alternativa: Parar completamente (pode ir para modo neutro)
    // positionMotor.stopMotor(); 
  }

  /**
   * Obtém a posição atual do motor.
   *
   * @return A posição atual em rotações.
   */
  public double getCurrentPosition() {
    // O 'true' força a leitura do sinal sincronizado mais recente
    return positionMotor.getRotorPosition().getValueAsDouble();
  }

  /**
   * Obtém a posição alvo atual do controlo PositionVoltage.
   *
   * @return A posição alvo em rotações.
   */
  public double getTargetPosition() {
      return positionVoltageControl.Position;
  }

  @Override
  public void periodic() {
    // Este método é chamado repetidamente pelo Scheduler
    // Coloque aqui o código que deve ser executado periodicamente

    // Exemplo: Enviar a posição atual para o SmartDashboard
    SmartDashboard.putNumber("Subsystem Position (Rotations)", getCurrentPosition());
    // Pode adicionar mais informações úteis:
    // SmartDashboard.putNumber("Subsystem Target Position", getTargetPosition());
    // SmartDashboard.putNumber("Subsystem Voltage Output", positionMotor.getMotorVoltage().getValue());
    // SmartDashboard.putNumber("Subsystem Stator Current", positionMotor.getStatorCurrent().getValue());
  }
}

