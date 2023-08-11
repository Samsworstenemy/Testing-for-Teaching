// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.revrobotics.CANSparkMax;

public class MotorSubsystem extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  public MotorSubsystem() {}
  //:> This creates the motor we will be using to spin
  CANSparkMax funnyMotor = new CANSparkMax( Constants.MotorValues.funnymotorid, CANSparkMax.MotorType.kBrushless);
  
  public static Pose2d funnyPose;

  // :> Test method that I called just to see if things were working
  public void ping() {
    System.out.println("pong");
  }

  // :> Creates the method with the logic inside to actually spin the motor
  public void spinMotor(double number) {
    funnyMotor.set(number);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
