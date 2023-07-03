// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;

public class DriveSubsystem extends SubsystemBase {
  
  // :> Let's define some motors
  CANSparkMax frontLeftMotor = new CANSparkMax( Constants.DriveConstants.frontLeftMotorid, CANSparkMax.MotorType.kBrushless);
  CANSparkMax frontRightMotor = new CANSparkMax( Constants.DriveConstants.frontRightMotorid, CANSparkMax.MotorType.kBrushless);
  CANSparkMax rearLeftMotor = new CANSparkMax( Constants.DriveConstants.backLeftMotorid, CANSparkMax.MotorType.kBrushless);
  CANSparkMax rearRightMotor = new CANSparkMax( Constants.DriveConstants.backRightMotorid, CANSparkMax.MotorType.kBrushless);
  
  // :> Gotta define the relativeEncoders
  private final RelativeEncoder frontLeftEncoder;
  private final RelativeEncoder frontRightEncoder;
  private final RelativeEncoder backLeftEncoder;
  private final RelativeEncoder backRightEncoder;
  
  // :> Gotta create the PIDController objects
  private final SparkMaxPIDController frontLeftPIDController;
  private final SparkMaxPIDController frontRightPIDController;
  private final SparkMaxPIDController backLeftPIDController;
  private final SparkMaxPIDController backRightPIDController;
  
  // :> Mec Drive Kinematics Objects that will be used to calculate wheel speeds and positions on the robot.
  MecanumDriveKinematics kinematics = new MecanumDriveKinematics(
  Constants.DriveConstants.frontLeftMeters, 
  Constants.DriveConstants.frontRightMeters, 
  Constants.DriveConstants.backLeftMeters, 
  Constants.DriveConstants.backRightMeters
  );

  // :> Creates a MecanumDrive from the motors we gave it
  MecanumDrive driveSpeeds = new MecanumDrive(frontLeftMotor, rearLeftMotor, frontRightMotor, rearRightMotor);

  /** Creates a new ExampleSubsystem. */
  public DriveSubsystem() {

    // :> I mayhaps have to invert some of the motors depending on how Zuri behaves
    // frontRightMotor.setInverted(true);
    // frontLeftMotor.setInverted(true);
    // rearRightMotor.setInverted(true);
    // rearLeftMotor.setInverted(true);
    
    // :> Setting the encoders to the actual encoders on the motors
    frontLeftEncoder = frontLeftMotor.getEncoder();
    frontRightEncoder = frontRightMotor.getEncoder();
    backLeftEncoder = rearLeftMotor.getEncoder();
    backRightEncoder = rearLeftMotor.getEncoder();

    // :> Setting the PID Controller to the PID Controllers on the actual motors
    frontLeftPIDController = frontLeftMotor.getPIDController();
    frontRightPIDController = frontRightMotor.getPIDController();
    backLeftPIDController = rearLeftMotor.getPIDController();
    backRightPIDController = rearRightMotor.getPIDController();

    // :> Sets the velocity/Positional conversion factor that way we can use it for methods later
    frontLeftEncoder.setVelocityConversionFactor(Constants.DriveConstants.velocityConversionRatio);
    frontRightEncoder.setVelocityConversionFactor(Constants.DriveConstants.velocityConversionRatio);
    backLeftEncoder.setVelocityConversionFactor(Constants.DriveConstants.velocityConversionRatio);
    backRightEncoder.setVelocityConversionFactor(Constants.DriveConstants.velocityConversionRatio);
    // :> Gives us errors and information if something goes wrong with the PIDs
    frontRightPIDController.setFeedbackDevice(frontRightEncoder);
    frontLeftPIDController.setFeedbackDevice(frontLeftEncoder);
    backRightPIDController.setFeedbackDevice(backRightEncoder);
    backLeftPIDController.setFeedbackDevice(backLeftEncoder);
  }

  // :> Sets the references for the PID controllers to look at and get a reading from
  public void setVelocityReference(double flRef, double blRef, double frRef, double brRef) { 
    
    frontLeftPIDController.setReference(flRef, ControlType.kVelocity);
    frontRightPIDController.setReference(frRef, ControlType.kVelocity);
    backLeftPIDController.setReference(blRef, ControlType.kVelocity);
    backRightPIDController.setReference(brRef, ControlType.kVelocity);

  }
  
  // :> Sets the Velocity Reference from the actual wheel speeds
  public void setVelocityReference(MecanumDriveWheelSpeeds wheelSpeeds) {
    setVelocityReference(
      wheelSpeeds.frontLeftMetersPerSecond,
      wheelSpeeds.rearLeftMetersPerSecond,
      wheelSpeeds.frontRightMetersPerSecond,
      wheelSpeeds.rearRightMetersPerSecond 
    );
  }

  // :> PID time
  // :> Sets the P I and D gain for all the motors
  public void setPIDValues(double kP, double kI, double kD) {

    frontLeftPIDController.setP(kP);
    frontLeftPIDController.setI(kI);
    frontLeftPIDController.setD(kD);
    
    frontRightPIDController.setP(kP);
    frontRightPIDController.setI(kI);
    frontRightPIDController.setD(kD);
    
    backLeftPIDController.setP(kP);
    backLeftPIDController.setI(kI);
    backLeftPIDController.setD(kD);

    backRightPIDController.setP(kP);
    backRightPIDController.setI(kI);
    backRightPIDController.setD(kD);

    frontLeftPIDController.setIAccum(0);
    frontRightPIDController.setIAccum(0);
    backLeftPIDController.setIAccum(0);
    backRightPIDController.setIAccum(0);

  }

  
  // :> The following methods get each of the wheel speeds indiviudally. This is very important
  // :> Gets the kinematics object for the inverse kinematics for commands later
  public MecanumDriveWheelSpeeds toWheelSpeeds(ChassisSpeeds robotSpeed) {
    return kinematics.toWheelSpeeds(robotSpeed);
  }

  public ChassisSpeeds getTeleopChassisSpeed(double x, double y, double r) {
    // :> Outputs wheels speed object based off of X, Y and rotation values
    ChassisSpeeds vehicleSpeed = new ChassisSpeeds(y, x, r);
      return vehicleSpeed;           
  }

  public MecanumDriveWheelSpeeds getWheelSpeeds(double x, double y, double r) {
    ChassisSpeeds vehicleSpeed = getTeleopChassisSpeed(x, y, r);
    MecanumDriveWheelSpeeds wheelSpeeds = toWheelSpeeds(vehicleSpeed);
    wheelSpeeds.desaturate(Constants.DriveConstants.maxWheelSpeed);
    return wheelSpeeds;
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
