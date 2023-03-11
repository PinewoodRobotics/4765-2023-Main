// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.ControlMode;
import edu.wpi.first.wpilibj.DutyCycleEncoder;

public class PitchSubsystem extends SubsystemBase {
    private final TalonSRX m_pitchMotor = new TalonSRX(5);
    private final DutyCycleEncoder m_pitchEncoder = new DutyCycleEncoder (5); // pitch
    private final double upperLimit = 0.95;
    private final double lowerLimit = 0.75;
    private final double motorCoefficient = 0.4;
    
    public void move(double axis){
        double encoderPitchValue = m_pitchEncoder.get();
        if (encoderPitchValue <= lowerLimit) {
            if (axis<0) {
                m_pitchMotor.set(ControlMode.PercentOutput, axis * motorCoefficient);
            } else {
                m_pitchMotor.set(ControlMode.PercentOutput, 0);
            }
        } else if (encoderPitchValue >= upperLimit) {
            if (axis>0) {
                m_pitchMotor.set(ControlMode.PercentOutput, axis * motorCoefficient);
            } else {
                m_pitchMotor.set(ControlMode.PercentOutput, 0);
            }
        } else {
            m_pitchMotor.set(ControlMode.PercentOutput, axis*motorCoefficient);
        }
    }

    public double getPos(){
        return m_pitchEncoder.get();
    }

}