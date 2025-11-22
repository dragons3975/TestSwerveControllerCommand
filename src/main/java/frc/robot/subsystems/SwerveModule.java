// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radian;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.lang.module.Configuration;
import java.rmi.dgc.DGC;

import org.opencv.core.Mat;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.DistanceUnit;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.Encoder;
import frc.robot.Constants;
import frc.robot.Constants.ModuleConstants;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SwerveModule {

  private final TalonFX m_driveMotor;
  private final TalonFX m_turningMotor;
  private final CANcoder m_turningEncoder;
  private double distance = 0;
  private double previousDist = 0;
  //private double deltaDist = 0;

  private final PIDController m_drivePIDController = new PIDController(ModuleConstants.kPModuleDriveController, 0, 0);

  // Using a TrapezoidProfile PIDController to allow for smooth turning
  private final ProfiledPIDController m_turningPIDController =
      new ProfiledPIDController(
          ModuleConstants.kPModuleTurningController,
          0,
          0,
          new TrapezoidProfile.Constraints(
              ModuleConstants.kMaxModuleAngularSpeedRadiansPerSecond,
              ModuleConstants.kMaxModuleAngularAccelerationRadiansPerSecondSquared));


  public SwerveModule(int driveMotorChannel, int turningMotorChannel, int turningEncoderChannel, 
      boolean driveMotorReversed,
      boolean turnMotorReversed,
      boolean turnEncoderReversed)
  {
    m_driveMotor = new TalonFX(driveMotorChannel);
    m_turningMotor = new TalonFX(turningMotorChannel);

    m_turningEncoder = new CANcoder(turningEncoderChannel);

    TalonFXConfiguration driveMotorConfig = new TalonFXConfiguration();
    driveMotorConfig.Feedback.SensorToMechanismRatio = 1/ModuleConstants.kDriveDistanceParTick;
    driveMotorConfig.MotorOutput.Inverted = driveMotorReversed ? InvertedValue.CounterClockwise_Positive : InvertedValue.Clockwise_Positive;
    m_driveMotor.getConfigurator().apply(driveMotorConfig);

    TalonFXConfiguration turnMotorConfig = new TalonFXConfiguration();
    turnMotorConfig.MotorOutput.Inverted = turnMotorReversed ? InvertedValue.CounterClockwise_Positive : InvertedValue.Clockwise_Positive;
    m_turningMotor.getConfigurator().apply(turnMotorConfig);

    CANcoderConfiguration turnEncoderConfig = new CANcoderConfiguration();
    m_turningEncoder.getConfigurator().refresh(turnEncoderConfig); // Lire la config actuelle pour ne pas écraser l'offset configuré avec Phoenix Tuner
    turnEncoderConfig.MagnetSensor.SensorDirection = turnEncoderReversed ? SensorDirectionValue.CounterClockwise_Positive : SensorDirectionValue.Clockwise_Positive;
    m_turningEncoder.getConfigurator().apply(turnEncoderConfig);

    // Limit the PID Controller's input range between -pi and pi and set the input
    // to be continuous.
    m_turningPIDController.enableContinuousInput(-Math.PI, Math.PI);
    //m_turningPIDController.setTolerance(Radian.convertFrom(90, Degrees));
  }

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  /*public SwerveModuleState getState() {
    return new SwerveModuleState(getDriveVelocityMetersPerSecond(), getTurningRotation2d());
  }*/

  /**
   * Returns the current position of the module.
   *
   * @return The current position of the module.
   */
  /*public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(getDriveDistanceMeter(), getTurningRotation2d());
  }*/

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
public void setDesiredState(SwerveModuleState desiredState) {
    var encoderRotation = getTurningRotation2d();

    // Optimize the reference state to avoid spinning further than 90 degrees
    desiredState.optimize(encoderRotation);

    // Scale speed by cosine of angle error. This scales down movement perpendicular to the desired
    // direction of travel that can occur when modules change directions. This results in smoother
    // driving.
    desiredState.cosineScale(encoderRotation);

    // Calculate the drive output from the drive PID controller.
    //SmartDashboard.putNumber("Encoder Brut" + m_driveMotor.getDeviceID(), m_driveMotor.getPosition().getValueAsDouble());
    //SmartDashboard.putNumber("Encoder Turn Brut" + m_turningMotor.getDeviceID(), getRotation());
    //SmartDashboard.putNumber("Encoder Turn Brut Motor" + m_turningMotor.getDeviceID(), m_turningMotor.getPosition().getValueAsDouble());
    //SmartDashboard.putNumber("Meters", getDriveDistanceMeter());
    
    //SmartDashboard.putNumber("deltaDist", deltaDist);
    //SmartDashboard.putNumber("previousDist", previousDist);
    //SmartDashboard.putNumber("dist", distance);

    /*SmartDashboard.putNumber("CurrentSpeedMetersPerSecond" + m_driveMotor.getDeviceID(), getDriveVelocityMetersPerSecond());    
    SmartDashboard.putNumber("TargetSpeedMetersPerSecond" + m_driveMotor.getDeviceID(), desiredState.speedMetersPerSecond);
    SmartDashboard.putNumber("EncodeurBrut" + m_driveMotor.getDeviceID(), m_driveMotor.getPosition().getValueAsDouble());*/

    final double driveOutput = m_drivePIDController.calculate(getDriveVelocityMetersPerSecond(), desiredState.speedMetersPerSecond);    
    SmartDashboard.putNumber("driveOutput" + m_driveMotor.getDeviceID(), driveOutput);

    // Calculate the turning motor output from the turning PID controller.
    SmartDashboard.putNumber("CurrentAngleRadians" + m_turningMotor.getDeviceID(), getTurningRotation2d().getRadians());    
    SmartDashboard.putNumber("TargetAngleRadians" + m_turningMotor.getDeviceID(), desiredState.angle.getRadians());
    final double turnOutput = m_turningPIDController.calculate(getTurningRotation2d().getRadians(), desiredState.angle.getRadians());
    SmartDashboard.putNumber("turnOutput" + m_turningMotor.getDeviceID(), turnOutput);

    //Calculate the turning motor output from the turning PID controller.
    m_driveMotor.set(driveOutput);
    m_turningMotor.set(turnOutput);
  }

  /** Zeroes all the SwerveModule encoders. */
  /*public void resetEncoders() {
    m_driveEncoder.reset();
    m_turningEncoder.reset();
  }*/

  private double getDriveVelocityMetersPerSecond() {
    return m_driveMotor.getVelocity().getValueAsDouble();
  }

  private Rotation2d getTurningRotation2d() {
    return Rotation2d.fromDegrees(m_turningEncoder.getAbsolutePosition().getValue().in(Degrees));
  }

}
