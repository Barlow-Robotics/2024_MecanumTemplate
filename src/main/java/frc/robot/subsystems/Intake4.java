// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake4 extends SubsystemBase {

  WPI_TalonFx intake4Motor;

  /** Creates a new Intake4. */
  public Intake4() {
    intake4Motor = new WPI_TalonFX(Constants.IntakeConsants.intake4MotorID);
    setMotorConfig(intake4Motor); 

  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
 

  private void setMotorConfig(WPI_TalonFX motor) { // changed to TalonFX for intake
    motor.configFactoryDefault();
    motor.configClosedloopRamp(Constants.IntakeConstants.closedVoltageRampingConstant);
    motor.configOpenloopRamp(Constants.IntakeConstants.manualVoltageRampingConstant);
    motor.config_kF(Constants.IntakeConstants.PID_id, Constants.IntakeConstants.kF);
    motor.config_kP(Constants.IntakeConstants.PID_id, Constants.IntakeConstants.kP);
    motor.config_kI(Constants.IntakeConstants.PID_id, 0);
    motor.config_kD(Constants.IntakeConstants.PID_id, 0);
    motor.setNeutralMode(NeutralMode.Brake);
}
 
  public void startIntake4() {
    intake4Motor.set(TalonFXControlMode.Velocity, Constants.IntakeConstants.IntakeMotorSpeed);
  }

  public void stopIntake4() {
    intake4Motor.set(TalonFXControlMode.Velocity, 0.0);
  }

  public boolean intake4IsRunning() {
    return intake4Motor.getSpeed() != 0.0;

  }
}
