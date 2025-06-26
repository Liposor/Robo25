// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ElevadorS extends SubsystemBase 
{
  private SparkMax motorDireito;
  private SparkMax motorEsquerdo;
  private SparkMaxConfig configMotor;
  private RelativeEncoder encoderDireito;
  private RelativeEncoder encoderEsquerdo;

  // Criar controladores PID separados para cada motor
  private SparkClosedLoopController pidControllerDireito;
  private SparkClosedLoopController pidControllerEsquerdo;
  
  public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM;



  /** Creates a new ElevadorS. */
  public ElevadorS() 
  {
    // --- Configuração dos Ganhos PID (mantidos do original) ---
    kP = 0.2;
    kI = 0;
    kD = 0.005;
    kIz = 0.0;
    kFF = 0.003;
    kMaxOutput= 0.99;
    kMinOutput = -0.13;
    maxRPM = 5700.0;

    // --- Inicialização dos Motores ---
    motorDireito = new SparkMax(Constants.ElevadorC.motor1ID, MotorType.kBrushless);
    motorEsquerdo = new SparkMax(Constants.ElevadorC.motor2ID, MotorType.kBrushless);

    // --- Obtenção dos Encoders (CORRIGIDO) ---
    encoderDireito = motorDireito.getEncoder();
    encoderEsquerdo = motorEsquerdo.getEncoder(); // Corrigido: usar motorEsquerdo

    // --- Configuração Comum dos Motores ---
    configMotor = new SparkMaxConfig();
    configMotor.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);
    configMotor
      .idleMode(Constants.ElevadorC.kMotorIdleMode)
      .smartCurrentLimit(Constants.ElevadorC.motorCurrentLimit)
      .inverted(false); // Assumindo que ambos não precisam ser invertidos. Ajustar se necessário.

    configMotor.encoder
      .velocityConversionFactor(1) // Verificar se estes fatores são iguais para ambos
      .positionConversionFactor(1);

    configMotor.closedLoop
      .pidf(kP, kI, kD, kFF)
      .outputRange(kMinOutput, kMaxOutput); // Definir output range aqui é boa prática


    // --- Aplicação da Configuração e Obtenção dos Controladores PID (CORRIGIDO) ---
    motorDireito.configure(configMotor, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    motorEsquerdo.configure(configMotor, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // Obter controladores PID separados
    pidControllerDireito = motorDireito.getClosedLoopController();
    pidControllerEsquerdo = motorEsquerdo.getClosedLoopController();

    // Resetar encoders na inicialização
    resetEncoders();
  }

  @Override
  public void periodic() 
  {
    // Exibir posição de ambos os encoders (ou média) no SmartDashboard
    SmartDashboard.putNumber("Encoder Direito Posição", getPositionDireito());
    SmartDashboard.putNumber("Encoder Esquerdo Posição", getPositionEsquerdo());
    SmartDashboard.putNumber("Encoder Média Posição", getPositionMedia());
    // Poderia adicionar também a velocidade, corrente, etc.
  }

  /**
   * Comanda ambos os motores para irem para a posição desejada usando PID.
   * @param position A posição alvo (em unidades do positionConversionFactor).
   */
  public void driveToPosition(double position) 
  {
    // Enviar o mesmo setpoint para ambos os controladores PID
    pidControllerDireito.setReference(position, SparkMax.ControlType.kPosition);
    pidControllerEsquerdo.setReference(position, SparkMax.ControlType.kPosition);
    
    // Opcional: Adicionar um feedforward arbitrário se necessário (ex: para gravidade)
    // double arbFeedforward = calcularFeedforwardGravidade(position);
    // pidControllerDireito.setReference(position, SparkMax.ControlType.kPosition, 0, arbFeedforward);
    // pidControllerEsquerdo.setReference(position, SparkMax.ControlType.kPosition, 0, arbFeedforward);
  }

  /**
   * Obtém a posição do encoder direito.
   * @return A posição em unidades do positionConversionFactor.
   */
  public double getPositionDireito() 
  {
    return encoderDireito.getPosition();
  }

  /**
   * Obtém a posição do encoder esquerdo.
   * @return A posição em unidades do positionConversionFactor.
   */
  public double getPositionEsquerdo() 
  {
    return encoderEsquerdo.getPosition();
  }

  /**
   * Obtém a posição média dos dois encoders.
   * Útil para ter uma leitura única do estado do elevador.
   * @return A posição média em unidades do positionConversionFactor.
   */
  public double getPositionMedia() 
  {
    return (encoderDireito.getPosition() + encoderEsquerdo.getPosition()) / 2.0;
  }

  /**
   * Reseta a posição de ambos os encoders para zero.
   */
  public void resetEncoders()
  {
    encoderDireito.setPosition(0);
    encoderEsquerdo.setPosition(0);
  }

  /**
   * Para ambos os motores.
   */
  public void stopMotors() 
  {
      motorDireito.stopMotor();
      motorEsquerdo.stopMotor();
  }
}

