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
    private final TalonSRX m_motordrive_pitch = new TalonSRX(5);
    private final DutyCycleEncoder m_encoder_pitch = new DutyCycleEncoder (5); // pitch
    private final double upper_limit = 0.95;
    private final double lower_limit = 0.75;
    private final double motorCoefficient = 0.4;
    
    public void move(double axis){
        double encoder_pitch_value = m_encoder_pitch.get();
        if (encoder_pitch_value<=lower_limit) {
            if (axis<0) {
                m_motordrive_pitch.set(ControlMode.PercentOutput, axis*motorCoefficient);
            } else {
                m_motordrive_pitch.set(ControlMode.PercentOutput, 0);
            }
        } else if (encoder_pitch_value>=upper_limit) {
            if (axis>0) {
                m_motordrive_pitch.set(ControlMode.PercentOutput, axis*motorCoefficient);
            } else {
                m_motordrive_pitch.set(ControlMode.PercentOutput, 0);
            }
        } else {
            m_motordrive_pitch.set(ControlMode.PercentOutput, axis*motorCoefficient);
        }
    }

    public double getPos(){
        return m_encoder_pitch.get();
    }

}