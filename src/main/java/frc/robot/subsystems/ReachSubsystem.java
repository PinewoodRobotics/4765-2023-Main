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

public class ReachSubsystem extends SubsystemBase {
    private final TalonSRX m_motordrive_reach = new TalonSRX(9);
    private final DutyCycleEncoder m_encoder_reach = new DutyCycleEncoder (0); // pitch
    private final double upper_limit = 5.37;
    private final double lower_limit = -4.22;
    
    public void move(double axis){
        double encoder_reach_value = m_encoder_reach.get();
        if (encoder_reach_value>lower_limit && encoder_reach_value<upper_limit)
        m_motordrive_reach.set(ControlMode.PercentOutput, axis*0.5);
    }


    public double getPos(){
        return m_encoder_reach.get();
    }
}