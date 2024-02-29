package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.motorcontrol.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix.motorcontrol.*;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkLowLevel.MotorType;


public class ShooterSubsystem extends SubsystemBase {
    private CANSparkMax rightMotor;
    private CANSparkMax leftMotor;
    private SparkPIDController PIDController;
    private  RelativeEncoder m_encoder;

    
    public ShooterSubsystem(){
        
        rightMotor = new CANSparkMax(9,MotorType.kBrushless);
        leftMotor = new CANSparkMax(8, MotorType.kBrushless);
        PIDController = rightMotor.getPIDController();
        m_encoder = rightMotor.getEncoder();
        
    }

    public void setShooter(Boolean open){
        if (open == true){
            rightMotor.setInverted(false);
            leftMotor.setInverted(false);
            PIDController.setReference(2800, CANSparkMax.ControlType.kVelocity);
        }
        else{
            rightMotor.set(0);
            leftMotor.set(0);
        }
    }
    public void amphShoot(){
         rightMotor.setInverted(true);
         leftMotor.setInverted(false);
         rightMotor.set(-0.17);
         leftMotor.set(-0.17);
     
    }

    @Override
    public void periodic(){
        SmartDashboard.putNumber("Shooter Velocity", m_encoder.getVelocity());
    }

}