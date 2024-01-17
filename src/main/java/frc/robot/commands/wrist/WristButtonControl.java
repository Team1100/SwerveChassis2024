// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.wrist;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.PIDWrist;
import frc.robot.Constants;

public class WristButtonControl extends Command {
  /** Creates a new WristButtonControl. */
  PIDWrist m_wrist;
  int m_direction;

  public WristButtonControl(int direction) {
    m_wrist = PIDWrist.getInstance();
    m_direction = direction;
    addRequirements(m_wrist);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double angle = m_wrist.getPosition();
    angle += m_direction * Constants.ManipulatorConstants.kWristIncrement;
    m_wrist.setSetpoint(angle);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
