// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Arm extends SubsystemBase {
  /** Creates a new Arm. */
  private TalonFX motor;

  private final MotionMagicVoltage motionMagic = new MotionMagicVoltage(0);

  private Arm() {
    this.motor = new TalonFX(Constants.IntakeConstants.KMotorID);

    TalonFXConfiguration configs = new TalonFXConfiguration();
    MotionMagicConfigs mm = new MotionMagicConfigs();

    mm.MotionMagicCruiseVelocity = Constants.ArmConstants.mmVelocity;
    mm.MotionMagicAcceleration = Constants.ArmConstants.mmAcceleration;
    mm.MotionMagicJerk = Constants.ArmConstants.mmJerk;
    configs.MotionMagic = mm;

    configs.Slot0.kP = Constants.ArmConstants.kP;
    configs.Slot0.kD = Constants.ArmConstants.kD;
    configs.Slot0.kV = Constants.ArmConstants.kV;
    configs.Slot0.kS = Constants.ArmConstants.kS;

    configs.Voltage.PeakForwardVoltage = Constants.ArmConstants.PeakForwardVoltage;
    configs.Voltage.PeakReverseVoltage = Constants.ArmConstants.PeakReverseVoltage;

    configs.Feedback.SensorToMechanismRatio = Constants.ArmConstants.SensorToMechanismRatio;

    StatusCode status = StatusCode.StatusCodeNotInitialized;
    
    for (int i = 0; i < 5; i++){
      status = motor.getConfigurator().apply(configs);
      if (status.isOK()){
        break;
      }
    }
  }

  public static Arm instance;

  public static Arm getInstance(){
    if (instance==null){
      instance = new Arm();
    }
    return instance;
  }


  public void moveArm(double motorspeed){
    this.motor.set(motorspeed);
  }


  public double getPosArm(){
    return this.motor.getPosition().getValue();
  }

  public void putArmInPoseMM(double pose){
    motor.setControl(motionMagic.withPosition(pose));
  }



  @Override
  public void periodic() {

  }
}
