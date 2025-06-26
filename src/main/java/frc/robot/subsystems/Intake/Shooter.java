// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Intake;

import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {
  private TalonFX Shooter1;
  public CANrange range1;
  public CANrange range2;
  /** Creates a new Shooter. */
  public Shooter() 
  {
    Shooter1 = new TalonFX(Constants.Intake.k_ShooterID, "rio");
    range1 = new CANrange(Constants.Intake.k_Canrager1, "rio");
    range2 = new CANrange(Constants.Intake.k_Canrager2, "rio");

    CANrangeConfiguration config = new CANrangeConfiguration();

    config.ProximityParams.MinSignalStrengthForValidMeasurement = 2500;
    config.ProximityParams.ProximityThreshold = 0.1;
    config.ProximityParams.ProximityHysteresis = 0.04;

    config.ToFParams.UpdateFrequency = 50;

    CANrangeConfiguration config2 = new CANrangeConfiguration();

    config2.ProximityParams.MinSignalStrengthForValidMeasurement = 2500;
    config2.ProximityParams.ProximityThreshold = 0.1;
    config2.ProximityParams.ProximityHysteresis = 0.04;

    config2.ToFParams.UpdateFrequency = 50;

    range1.getConfigurator().apply(config2);
    range2.getConfigurator().apply(config);

    range1.getIsDetected();




    krakenConfigs();

    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  SmartDashboard.putBoolean("canrage1", range1.getIsDetected().getValue());
  SmartDashboard.putBoolean("canrage2", range2.getIsDetected().getValue());

  SmartDashboard.putBoolean("tem algo?", range2.getIsDetected().getValue());
  }

   public void krakenConfigs()
  {
    // Config Motor
    TalonFXConfiguration globalconfig = new TalonFXConfiguration();
    Shooter1.clearStickyFaults();
    // Padrão de Fabrica
    Shooter1.getConfigurator().apply(new TalonFXConfiguration());

    CurrentLimitsConfigs currentLimitsConfigs = globalconfig.CurrentLimits;
    currentLimitsConfigs.StatorCurrentLimit =
                        Constants.Intake.SO_kCurrentThreshold;
    currentLimitsConfigs.StatorCurrentLimitEnable = 
                        Constants.Intake.SO_kEnableCurrentLimit;

    

    globalconfig.Feedback.SensorToMechanismRatio= 1;
    globalconfig.MotorOutput.NeutralMode = Constants.Intake.kNeutralMode;

    Shooter1.getConfigurator().apply(globalconfig);


  }

  public void SetShooter(double speed)
  {
    Shooter1.set(speed);
  }

  public void StopShooter(double speed)
  {
    Shooter1.set(speed);
  }



}
