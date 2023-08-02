// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class DriveCommand extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final DriveSubsystem m_subsystem;
  private final XboxController m_controller;
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public DriveCommand(DriveSubsystem subsystem, XboxController controller) {
    m_subsystem = subsystem;
    m_controller = controller;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    // :> Sets the PID values from constants
    m_subsystem.setPIDValues(Constants.PIDConstants.pGain,
    Constants.PIDConstants.iGain,
    Constants.PIDConstants.dGain);
  
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    // :> Corresponds the controllers joy sticks to the data being passed in to the driving functions
    double xSpeed = m_controller.getLeftX() * Constants.DriveConstants.DriveMultiplier;
    double ySpeed = m_controller.getLeftY() * Constants.DriveConstants.DriveMultiplier;
    double zRotation = m_controller.getRightX() * Constants.DriveConstants.RotationMultiplier;
  
    // :> Corresponds the data from the joy sticks to the actual wheel speeds
    MecanumDriveWheelSpeeds wheelSpeeds = m_subsystem.getWheelSpeeds(xSpeed, ySpeed, zRotation);
    
    // :> Takes the corresponded wheel speeds and sets the wheels to that speed with the PIDs
    m_subsystem.setVelocityReference(wheelSpeeds);

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
