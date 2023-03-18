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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ArmSubsystem extends SubsystemBase {
    private final ReachSubsystem m_reachSubsystem = new ReachSubsystem();
    private final PitchSubsystem m_pitchSubsystem = new PitchSubsystem();
    private final ClawSubsystem m_clawSubsystem = new ClawSubsystem();
    private final double ArmPivotX = 12.0; // Distance of arm pivot from front of frame
    private final double ArmPivotY = 23.0; // Height of arm pivot above floor (inches)


    public void movePitch(double axis){
        m_pitchSubsystem.move(axis);
    }

    public void moveReach(double axis){
        m_reachSubsystem.move(axis);
    }

    //public void moveClaw(double axis){
    //    m_clawSubsystem.grab();
    //}

    public void setArm(double pitch, double reach) {
        m_pitchSubsystem.setSetpoint(pitch);
        m_reachSubsystem.set(reach);
    }

    public void move(double pitch, double reach) {
        // TODO: Do smart things here...

        m_pitchSubsystem.move(pitch * 0.9);
        m_reachSubsystem.move(reach * 0.9);

        //if (grab > -1) {
        //    m_clawSubsystem.grab();
        //} else {
        //    m_clawSubsystem.release();
        //}
    }

    public double getPosPitch(){
        return m_pitchSubsystem.getPos();
    }

    public double getPosReach(){
        return m_reachSubsystem.getPos();
    }

    // Return the arm pitch in radians, where 0.0° is horizontal.
    // Angles above horizontal are positive, and below horizontal are negative.
    public double getPitchRadians(){
        return m_pitchSubsystem.getPosRadians();
    }

    // Return the arm reach in inches, from the arm pivot point
    public double getReachInches(){
        return m_reachSubsystem.getPosInches();
    }

    // This returns the cartesian X coordinate in inches, relative to the arm pivot
    public double getPosX(){
        // convert the polar coordiantes to cartesian, relative to the arm pivot
        double Xinches = Math.cos(getPitchRadians()) * getReachInches() + ArmPivotX;
            
        // Now offset the X value so it is relative to the front of the frame
        Xinches -= ArmPivotX;
        return Xinches;
    }
    
    // This returns the cartesian Y coordinate in inches, relative to the floor;
    public double getPosY(){
        // convert the polar coordiantes to cartesian, relative to the arm pivot
        double Yinches = Math.sin(getPitchRadians()) * getReachInches();
        
        // Now offset the Y value so it is relative to the floor
        Yinches += ArmPivotY;
        return Yinches;
    }

    // Move the arm so the claw is at the described coordinates:
    // X is inches in front of frame, and Y is inches above the floor.
    public void moveCartesian(double X, double Y){
        // Enforce rules while coordinates are frame/floor relative
        X = Math.min(46.0, X); // Clip if we are too far out
        Y = Math.min(52.0, Y); // Clip if we are too high
        Y = Math.max( 2.0, Y); // Avoid digging into the floor

        // Convert the coordinates so they are relative to the arm pivot
        X += ArmPivotX;
        Y -= ArmPivotY;

        // Convert X & Y to Polar coordinates
        double pitch = Math.atan( Y / X );
        double reach = Math.sqrt( X*X + Y*Y );

        // Send the commands
        m_pitchSubsystem.moveTo(pitch);
        m_reachSubsystem.moveTo(reach);
    }
}
