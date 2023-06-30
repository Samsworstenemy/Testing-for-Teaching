// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.revrobotics.CANSparkMax;

public class DriveSubsystem extends SubsystemBase {
  
  // :> Let's define some motors
  CANSparkMax frontLeftMotor = new CANSparkMax( Constants.DriveConstants.frontLeftMotorid, CANSparkMax.MotorType.kBrushless);
  CANSparkMax frontRightMotor = new CANSparkMax( Constants.DriveConstants.frontRightMotorid, CANSparkMax.MotorType.kBrushless);
  CANSparkMax rearLeftMotor = new CANSparkMax( Constants.DriveConstants.backLeftMotorid, CANSparkMax.MotorType.kBrushless);
  CANSparkMax rearRightMotor = new CANSparkMax( Constants.DriveConstants.backRightMotorid, CANSparkMax.MotorType.kBrushless);
  
  // :> Creates a MecanumDrive from the motors we gave it
  MecanumDrive driveSpeeds = new MecanumDrive(frontLeftMotor, rearLeftMotor, frontRightMotor, rearRightMotor);

  /** Creates a new ExampleSubsystem. */
  public DriveSubsystem() {
    // :> I mayhaps have to invert some of the motors depending on how Zuri behaves
    // frontRightMotor.setInverted(true);
    // frontLeftMotor.setInverted(true);
    // rearRightMotor.setInverted(true);
    // rearLeftMotor.setInverted(true);
  }

  
  
  // :> Drive cartesian. This method is the meat and potatoes of driving the robot
  public void setDriveMotors(double xSpeed, double ySpeed, double zRotation) {
    // :> Directs the motors on how to move based off of the value we give it.
    driveSpeeds.driveCartesian(ySpeed, xSpeed, zRotation);

    // :> This can all be ignored this is my backup plan just in case things go wrong

    // // :> WheelSpeeds wheelSpeeds = MecanumDrive.driveCartesianIK(ySpeed, -xSpeed, zRotation);
    // // :> Set's the wheel speeds based on the information from the speeds of the WheelSpeeds object
    // // frontLeftMotor.set(wheelSpeeds.frontLeft);
    // // frontRightMotor.set(wheelSpeeds.frontRight);
    // // rearLeftMotor.set(wheelSpeeds.rearLeft);
    // // rearRightMotor.set(wheelSpeeds.rearRight);
  
  }
  // :> Basic test method
  public void ping() {
    System.out.println("pong");
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
