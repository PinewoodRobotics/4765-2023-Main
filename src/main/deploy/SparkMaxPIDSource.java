/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj.Joystick;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import com.ctre.phoenix.sensors.CANCoder;

public class Robot extends TimedRobot {

  private Joystick m_joystick;

  private static final int turnCANID = 11;
  private CANSparkMax m_turnMotor;
  private SparkMaxPIDController m_turnPIDController;
  private RelativeEncoder m_turnEncoder;

  private static final int driveCANID = 21;
  private CANSparkMax m_driveMotor;
  private SparkMaxPIDController m_drivePIDController;
  private RelativeEncoder m_driveEncoder;

  private static final int cancoderCANID = 1;
  private CANCoder m_cancoder;

  public double kTurnP,
      kTurnI,
      kTurnD,
      kTurnIZ,
      kTurnFF,
      kTurnMaxOutput,
      kTurnMinOutput;

  public double kDriveP,
      kDriveI,
      kDriveD,
      kDriveIZ,
      kDriveFF,
      kDriveMaxOutput,
      kDriveMinOutput,
      kDriveMaxRPM;

      public double kCANCoderOffset;

      private double cancoderOffset;

      private double turnRotations = 0;

      private double turnDelta = 0;

  @Override
  public void robotInit() {

    m_joystick = new Joystick(0);

    // initialize motors
    m_turnMotor = new CANSparkMax(turnCANID, MotorType.kBrushless);
    m_driveMotor = new CANSparkMax(driveCANID, MotorType.kBrushless);

    m_cancoder = new CANCoder(cancoderCANID);

    cancoderOffset = m_cancoder.getAbsolutePosition() * 0.059;

    //m_cancoder.configMagnetOffset(61.5);




    /**
     * In order to use PID functionality for a controller, a SparkMaxPIDController
     * object
     * is constructed by calling the getPIDController() method on an existing
     * CANSparkMax object
     */
    m_turnPIDController = m_turnMotor.getPIDController();
    m_drivePIDController = m_driveMotor.getPIDController();

    // Encoder object created to display position values
    m_turnEncoder = m_turnMotor.getEncoder();
    

    m_driveEncoder = m_driveMotor.getEncoder();

   

    //m_turnPIDController.setReference( - m_turnEncoder.getPosition(), CANSparkMax.ControlType.kPosition);

    /**
     * The restoreFactoryDefaults method can be used to reset the configuration
     * parameters
     * in the SPARK MAX to their factory default state. If no argument is passed,
     * these
     * parameters will not persist between power cycles
     */
    m_turnMotor.restoreFactoryDefaults();
    m_driveMotor.restoreFactoryDefaults();

    //m_turnEncoder.setPositionConversionFactor(16.8);
    turnDelta = m_cancoder.getAbsolutePosition() - (m_turnEncoder.getPosition() * 16.8);
    SmartDashboard.putNumber("turnDelta", turnDelta);


    // PID coefficients
    kTurnP = 0.1;
    kTurnI = 1e-4;
    kTurnD = 1;
    kTurnIZ = 0;
    kTurnFF = 0;
    kTurnMaxOutput = 1;
    kTurnMinOutput = -1;

    kDriveP = 6e-5;
    kDriveI = 0;
    kDriveD = 0;
    kDriveIZ = 0;
    kDriveFF = 0.000015;
    kDriveMaxOutput = 1;
    kDriveMinOutput = -1;

    kDriveMaxRPM = 5700;

    kCANCoderOffset = 0;


    // set PID coefficients
    m_turnPIDController.setP(kTurnP);
    m_turnPIDController.setI(kTurnI);
    m_turnPIDController.setD(kTurnD);
    m_turnPIDController.setIZone(kTurnIZ);
    m_turnPIDController.setFF(kTurnFF);
    m_turnPIDController.setOutputRange(kTurnMinOutput, kTurnMaxOutput);

    m_drivePIDController.setP(kDriveP);
    m_drivePIDController.setI(kDriveI);
    m_drivePIDController.setD(kDriveD);
    m_drivePIDController.setIZone(kDriveIZ);
    m_drivePIDController.setFF(kDriveFF);
    m_drivePIDController.setOutputRange(kDriveMinOutput, kDriveMaxOutput);


    // display PID coefficients on SmartDashboard
    SmartDashboard.putNumber("Turn P Gain", kTurnP);
    SmartDashboard.putNumber("Turn I Gain", kTurnI);
    SmartDashboard.putNumber("Turn D Gain", kTurnD);
    SmartDashboard.putNumber("Turn I Zone", kTurnIZ);
    SmartDashboard.putNumber("Turn Feed Forward", kTurnFF);
    SmartDashboard.putNumber("Turn Max Output", kTurnMaxOutput);
    SmartDashboard.putNumber("Turn Min Output", kTurnMinOutput);

    SmartDashboard.putNumber("Set Turn Rotations", 0);

    SmartDashboard.putNumber("Drive P Gain", kDriveP);
    SmartDashboard.putNumber("Drive I Gain", kDriveI);
    SmartDashboard.putNumber("Drive D Gain", kDriveD);
    SmartDashboard.putNumber("Drive I Zone", kDriveIZ);
    SmartDashboard.putNumber("Drive Feed Forward", kDriveFF);
    SmartDashboard.putNumber("Drive Max Output", kDriveMaxOutput);
    SmartDashboard.putNumber("Drive Min Output", kDriveMinOutput);

    SmartDashboard.putNumber("Set Drive RPM", 0);
  }

  @Override
  public void teleopInit() {

    
  }

  @Override
  public void teleopPeriodic() {

    SmartDashboard.putNumber("J Angle", m_joystick.getDirectionDegrees());
    SmartDashboard.putNumber("J Mag", m_joystick.getMagnitude());
    SmartDashboard.putNumber("J Twist", m_joystick.getTwist());
    SmartDashboard.putNumber("J Trottle", m_joystick.getThrottle());

  SmartDashboard.putNumber("CANCoder Pos", m_cancoder.getAbsolutePosition());
  SmartDashboard.putNumber("turnEncoder", m_turnEncoder.getPosition());
  SmartDashboard.putNumber("turnDelta", turnDelta);

    // read PID coefficients from SmartDashboard
    double turnP = SmartDashboard.getNumber("Turn P Gain", 0);
    double turnI = SmartDashboard.getNumber("Turn I Gain", 0);
    double turnD = SmartDashboard.getNumber("Turn D Gain", 0);
    double turnIZ = SmartDashboard.getNumber("Turn I Zone", 0);
    double turnFF = SmartDashboard.getNumber("Turn Feed Forward", 0);
    double turnMax = SmartDashboard.getNumber("Turn Max Output", 0);
    double turnMin = SmartDashboard.getNumber("Turn Min Output", 0);

    //double turnRotations = SmartDashboard.getNumber("Set Turn Rotations", 0);

    double driveP = SmartDashboard.getNumber("Drive P Gain", 0);
    double driveI = SmartDashboard.getNumber("Drive I Gain", 0);
    double driveD = SmartDashboard.getNumber("Drive D Gain", 0);
    double driveIZ = SmartDashboard.getNumber("Drive I Zone", 0);
    double driveFF = SmartDashboard.getNumber("Drive Feed Forward", 0);
    double driveMax = SmartDashboard.getNumber("Drive Max Output", 0);
    double driveMin = SmartDashboard.getNumber("Drive Min Output", 0);

    //double driveRPM = SmartDashboard.getNumber("Set Drive RPM", 0);

    // if PID coefficients on SmartDashboard have changed, write new values to
    // controller
    if ((turnP != kTurnP)) {
      m_turnPIDController.setP(turnP);
      kTurnP = turnP;
    }
    if ((turnI != kTurnI)) {
      m_turnPIDController.setI(turnI);
      kTurnI = turnI;
    }
    if ((turnD != kTurnD)) {
      m_turnPIDController.setD(turnD);
      kTurnD = turnD;
    }
    if ((turnIZ != kTurnIZ)) {
      m_turnPIDController.setIZone(turnIZ);
      kTurnIZ = turnIZ;
    }
    if ((turnFF != kTurnFF)) {
      m_turnPIDController.setFF(turnFF);
      kTurnFF = turnFF;
    }
    if ((turnMax != kTurnMaxOutput) || (turnMin != kTurnMinOutput)) {
      m_turnPIDController.setOutputRange(turnMin, turnMax);
      kTurnMinOutput = turnMin;
      kTurnMaxOutput = turnMax;
    }

    if ((driveP != kDriveP)) {
      m_drivePIDController.setP(driveP);
      kDriveP = driveP;
    }
    if ((driveI != kDriveI)) {
      m_drivePIDController.setI(driveI);
      kDriveI = driveI;
    }
    if ((driveD != kDriveD)) {
      m_drivePIDController.setD(driveD);
      kDriveD = driveD;
    }
    if ((driveIZ != kDriveIZ)) {
      m_drivePIDController.setIZone(driveIZ);
      kDriveIZ = driveIZ;
    }
    if ((driveFF != kDriveFF)) {
      m_drivePIDController.setFF(driveFF);
      kDriveFF = driveFF;
    }
    if ((driveMax != kDriveMaxOutput) || (driveMin != kDriveMinOutput)) {
      m_drivePIDController.setOutputRange(driveMin, driveMax);
      kDriveMinOutput = driveMin;
      kDriveMaxOutput = driveMax;
    }

    /**
     * PIDController objects are commanded to a set point using the
     * SetReference() method.
     * 
     * The first parameter is the value of the set point, whose units vary
     * depending on the control type set in the second parameter.
     * 
     * The second parameter is the control type can be set to one of four
     * parameters:
     * com.revrobotics.CANSparkMax.ControlType.kDutyCycle
     * com.revrobotics.CANSparkMax.ControlType.kPosition
     * com.revrobotics.CANSparkMax.ControlType.kVelocity
     * com.revrobotics.CANSparkMax.ControlType.kVoltage
     */

     SmartDashboard.putNumber("Turn Conversion", m_turnEncoder.getPositionConversionFactor());

    if (m_joystick.getMagnitude() > 0.2) {

      double newHeading360 = m_joystick.getDirectionDegrees() + 180;

      SmartDashboard.putNumber("NewHeading360", newHeading360);

      double oldHeading360 = m_turnEncoder.getPosition() * 16.8;

      SmartDashboard.putNumber("OldHeading360", oldHeading360);


      turnDelta = (m_cancoder.getAbsolutePosition() - (m_turnEncoder.getPosition() * 16.8) % 360);
      
      SmartDashboard.putNumber("Turn Delta", turnDelta);


      double delta = m_cancoder.getAbsolutePosition() - m_turnEncoder.getPosition();

      SmartDashboard.putNumber("Delta", delta);

      turnRotations = (((m_joystick.getDirectionDegrees() + 180)- 306.56) * 0.059);

      turnRotations = (newHeading360 - turnDelta) * 0.059;

      //SmartDashboard.putNumber("Set Turn Rotations", 0);
      m_turnPIDController.setReference(turnRotations, CANSparkMax.ControlType.kPosition);

    } 

    SmartDashboard.putNumber("Turn SetPoint", turnRotations);
    SmartDashboard.putNumber("Turn ProcessVariable", m_turnEncoder.getPosition());

    double driveRPM;

    if (m_joystick.getMagnitude() > 0.1) {
      driveRPM = m_joystick.getMagnitude() * kDriveMaxRPM;
    } else {
      driveRPM = 0;
    }

    m_drivePIDController.setReference(driveRPM, CANSparkMax.ControlType.kVelocity);

    SmartDashboard.putNumber("Drive SetPoint", driveRPM);
    SmartDashboard.putNumber("Drive ProcessVariable", m_driveEncoder.getVelocity());
  }
}