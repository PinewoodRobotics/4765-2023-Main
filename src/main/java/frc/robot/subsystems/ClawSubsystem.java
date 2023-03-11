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

public class ClawSubsystem extends SubsystemBase {
    private final TalonSRX m_motordrive_claw = new TalonSRX(10);
    private final DutyCycleEncoder m_encoder_claw = new DutyCycleEncoder (10); // pitch
    private final double upper_limit = 0.95;
    private final double lower_limit = 0.55;
    
    public void grab(double axis){
        double encoder_pitch_value = m_encoder_claw.get();
        if (encoder_pitch_value>lower_limit && encoder_pitch_value<upper_limit)
        m_motordrive_claw.set(ControlMode.PercentOutput, axis*0.5);
    }
}