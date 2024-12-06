// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class Shintake extends SubsystemBase {
  /** Creates a new Shintake. */
  CANSparkMax motor_shintake;
  PIDController pid = new PIDController(Constants.kP, Constants.kI, Constants.kD);
  double targetRPM;
  Compressor pcmCompressor;
  DoubleSolenoid shooterSolenoid;
  CANSparkFlex topFront, topBack, bottomFront;
  CANSparkMax bottomBack;


  public Shintake() {
    topFront = new CANSparkFlex(Constants.SHINTAKE_TOP_FRONT_MOTOR, MotorType.kBrushless);
    topBack = new CANSparkFlex(Constants.SHINTAKE_TOP_BACK_MOTOR, MotorType.kBrushless);
    bottomFront = new CANSparkFlex(Constants.SHINTAKE_BOTTOM_FRONT_MOTOR, MotorType.kBrushless);
    bottomBack = new CANSparkMax(Constants.SHINTAKE_BOTTOM_BACK_MOTOR, MotorType.kBrushless);
  }
  public void shintake_intake(){
    topFront.set(Constants.SHINTAKE_INTAKE_SPEED);
    topBack.set(Constants.SHINTAKE_INTAKE_SPEED);
    bottomFront.set(-Constants.SHINTAKE_INTAKE_SPEED);
    bottomBack.set(-Constants.SHINTAKE_INTAKE_SPEED);
  }

  public void shintake_shoot(){
    topFront.set(-Constants.SHINTAKE_SHOOT_SPEED);
    topBack.set(-Constants.SHINTAKE_SHOOT_SPEED);
    bottomFront.set(Constants.SHINTAKE_SHOOT_SPEED);
    bottomBack.set(Constants.SHINTAKE_SHOOT_SPEED);
  }

  public void off(){
    topFront.set(0);
    topBack.set(0);
    bottomFront.set(0);
    bottomBack.set(0);
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
