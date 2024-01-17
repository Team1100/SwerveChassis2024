// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import frc.robot.Constants;
import frc.robot.testingdashboard.TestingDashboard;

public class Intake extends SubsystemBase {

  private static Intake m_intake;
  private CANSparkMax m_intakeMotor;
  /** Creates a new Intake. */
  private Intake() {
    m_intakeMotor = new CANSparkMax(Constants.ManipulatorConstants.kIntakeMotorCanId, MotorType.kBrushless);
    m_intakeMotor.setIdleMode(IdleMode.kBrake);
  }

  public static Intake getInstance() {
    if (m_intake == null) {
      m_intake = new Intake();
      TestingDashboard.getInstance().registerSubsystem(m_intake, "Intake");
    }
    return m_intake;
  }

  public void spin(double speed) {
    m_intakeMotor.set(speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // Two Motors

  }
}
