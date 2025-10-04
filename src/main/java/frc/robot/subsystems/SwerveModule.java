// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.lang.module.Configuration;

import org.opencv.core.Mat;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

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
  private double distance;
  private double previousDist;
  private double deltaDist;

  private final PIDController m_drivePIDController =
      new PIDController(ModuleConstants.kPModuleDriveController, 0, 0);

  // Using a TrapezoidProfile PIDController to allow for smooth turning
  private final ProfiledPIDController m_turningPIDController =
      new ProfiledPIDController(
          ModuleConstants.kPModuleTurningController,
          0,
          0,
          new TrapezoidProfile.Constraints(
              ModuleConstants.kMaxModuleAngularSpeedRadiansPerSecond,
              ModuleConstants.kMaxModuleAngularAccelerationRadiansPerSecondSquared));

  /**
   * Constructs a SwerveModule.
   *
   * @param driveMotorChannel The channel of the drive motor.
   * @param turningMotorChannel The channel of the turning motor.
   * @param driveEncoderChannels The channels of the drive encoder.
   * @param turningEncoderChannels The channels of the turning encoder.
   * @param driveEncoderReversed Whether the drive encoder is reversed.
   * @param turningEncoderReversed Whether the turning encoder is reversed.
   */
  public SwerveModule(
      int driveMotorChannel,
      int turningMotorChannel,
      int turningEncoderChannel/*, 
      boolean driveEncoderReversed,
      boolean turningEncoderReversed*/) {
    m_driveMotor = new TalonFX(driveMotorChannel);
    m_turningMotor = new TalonFX(turningMotorChannel);

    m_turningEncoder = new CANcoder(turningEncoderChannel);

    // Set the distance per pulse for the drive encoder. We can simply use the
    // distance traveled for one rotation of the wheel divided by the encoder
    // resolution.
    TalonFXConfiguration driveConfig = new TalonFXConfiguration();
    driveConfig.Feedback.SensorToMechanismRatio = ModuleConstants.kDriveEncoderDistancePerPulse;
    // Set whether drive encoder should be reversed or not
    //m_driveEncoder.setReverseDirection(driveEncoderReversed);
    m_driveMotor.getConfigurator().apply(driveConfig);

    // Set the distance (in this case, angle) in radians per pulse for the turning encoder.
    // This is the the angle through an entire rotation (2 * pi) divided by the
    // encoder resolution.
    CANcoderConfiguration turningConfig = new CANcoderConfiguration();
    driveConfig.Feedback.SensorToMechanismRatio = ModuleConstants.kTurningEncoderDistancePerPulse;
    // Set whether turning encoder should be reversed or not
    //m_turningEncoder.setReverseDirection(turningEncoderReversed);
    m_turningEncoder.getConfigurator().apply(turningConfig);

    // Limit the PID Controller's input range between -pi and pi and set the input
    // to be continuous.
    m_turningPIDController.enableContinuousInput(-Math.PI, Math.PI);

    previousDist = m_driveMotor.getPosition().getValueAsDouble();
  }

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    return new SwerveModuleState(getDriveVelocityMetersPerSecond(), getTurningRotation2d());
  }

  /**
   * Returns the current position of the module.
   *
   * @return The current position of the module.
   */
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(getDriveDistanceMeter(), getTurningRotation2d());
  }

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
    SmartDashboard.putNumber("Encoder Brut" + m_driveMotor.getDeviceID(), m_driveMotor.getPosition().getValueAsDouble());
    SmartDashboard.putNumber("Encoder Turn Brut" + m_turningMotor.getDeviceID(), getRotation());
    SmartDashboard.putNumber("Encoder Turn Brut Motor" + m_turningMotor.getDeviceID(), m_turningMotor.getPosition().getValueAsDouble());
    SmartDashboard.putNumber("Meters", getDriveDistanceMeter());
    

    getDriveDistanceMeter();
    SmartDashboard.putNumber("deltaDist", deltaDist);
    SmartDashboard.putNumber("previousDist", previousDist);
    SmartDashboard.putNumber("dist", distance);


    SmartDashboard.putNumber("CurrentSpeedMetersPerSecond" + m_driveMotor.getDeviceID(), getDriveVelocityMetersPerSecond().in(MetersPerSecond));    
    SmartDashboard.putNumber("TargetSpeedMetersPerSecond" + m_driveMotor.getDeviceID(), desiredState.speedMetersPerSecond);
    final double driveOutput = m_drivePIDController.calculate(getDriveVelocityMetersPerSecond().in(MetersPerSecond), desiredState.speedMetersPerSecond);
    SmartDashboard.putNumber("driveOutput" + m_driveMotor.getDeviceID(), driveOutput);

    // Calculate the turning motor output from the turning PID controller.
    SmartDashboard.putNumber("CurrentAngleRadians" + m_turningMotor.getDeviceID(), getTurningRotation2d().getRadians());    
    SmartDashboard.putNumber("TargetAngleRadians" + m_turningMotor.getDeviceID(), desiredState.angle.getRadians());
    final double turnOutput = m_turningPIDController.calculate(getTurningRotation2d().getRadians(), desiredState.angle.getRadians());
    SmartDashboard.putNumber("turnOutput" + m_turningMotor.getDeviceID(), turnOutput);

    //Calculate the turning motor output from the turning PID controller.
    //m_driveMotor.set(driveOutput);
    //m_turningMotor.set(turnOutput);
  }

  /** Zeroes all the SwerveModule encoders. */
  /*public void resetEncoders() {
    m_driveEncoder.reset();
    m_turningEncoder.reset();
  }*/

  private LinearVelocity getDriveVelocityMetersPerSecond() {
    return MetersPerSecond.of(m_driveMotor.getVelocity().getValueAsDouble());
  }

  private Double getDriveDistanceMeter() {
    deltaDist = m_driveMotor.getPosition().getValueAsDouble() - previousDist;
    previousDist = m_driveMotor.getPosition().getValueAsDouble();

    if (deltaDist < -16000) {
        deltaDist += 32000;
    }
    if (deltaDist > 16000) {
      deltaDist -= 32000;
    }

    distance += deltaDist;

    double nbrTours = distance / Constants.DriveConstants.kTickParTour;

    SmartDashboard.putNumber("Nbr Tours", nbrTours);
    return nbrTours * Constants.DriveConstants.kCirconferenceRoueMeters;
    
  }

  private Double getRotation() {
    return m_turningEncoder.getPosition().getValueAsDouble() * 2 * Math.PI;
  }

  private Rotation2d getTurningRotation2d() {
    return Rotation2d.fromDegrees(m_turningEncoder.getAbsolutePosition().getValue().in(Degrees));
  }

}
