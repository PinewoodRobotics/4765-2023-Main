// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
//import edu.wpi.first.wpilibj2.Servo
import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;

public class PitchSubsystem extends SubsystemBase {
    private final TalonSRX m_pitchMotor = new TalonSRX(0);
    private final DutyCycleEncoder m_pitchEncoder = new DutyCycleEncoder (0); // pitch
    private final double m_pitchOffset = m_pitchEncoder.get();
    private final double upperLimit = m_pitchOffset - 0.01;
    private final double lowerLimit = upperLimit-1.4;
    private final double upMotorCoefficient = 0.15;
    private final double downMotorCoefficient = 0.35;

    
//0.93
//0.5

         /** Creates a new PitchSubsystem. */
         public PitchSubsystem() {
            SmartDashboard.putNumber("Meas Pitch", m_pitchEncoder.get());
            



         }
         
    public void move(double axis){
        double encoderPitchValue = m_pitchEncoder.get();
        if (encoderPitchValue <= lowerLimit) {
            if (axis<0) {
                m_pitchMotor.set(ControlMode.PercentOutput, axis * downMotorCoefficient);
            } else {
                m_pitchMotor.set(ControlMode.PercentOutput, 0);
            }
        } else if (encoderPitchValue >= upperLimit) {
            if (axis>0) {
                m_pitchMotor.set(ControlMode.PercentOutput, axis * upMotorCoefficient);
            } else {
                m_pitchMotor.set(ControlMode.PercentOutput, 0);
            }
        } else {
            if (axis<0) {
                m_pitchMotor.set(ControlMode.PercentOutput, axis*downMotorCoefficient);
            } else if (axis>0) {
                m_pitchMotor.set(ControlMode.PercentOutput, axis*upMotorCoefficient);
            }
        }
        SmartDashboard.putNumber("pitch", encoderPitchValue);
        SmartDashboard.putNumber("Meas Pitch", m_pitchEncoder.get());
    }

    public void set(double number) {
        double currentPitch = m_pitchEncoder.get();
        if (currentPitch > number) {
            m_pitchMotor.set(ControlMode.PercentOutput, -0.3);
        } else if (currentPitch < number) {
            m_pitchMotor.set(ControlMode.PercentOutput, 0.3);
        }
    }

    public double getPos(){
        return m_pitchEncoder.get();
    }

}