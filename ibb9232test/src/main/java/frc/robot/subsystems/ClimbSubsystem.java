package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.motorcontrol.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix.motorcontrol.*;

public class ClimbSubsystem extends SubsystemBase {
    public TalonSRX climbMotor;
    public DigitalInput limitSwitch = new DigitalInput(2);

    public ClimbSubsystem(){
        climbMotor = new TalonSRX(7);
        climbMotor.setSelectedSensorPosition(0);
        }

    public void setClimb(int open){
        if (open == 1){
            climbMotor.set(ControlMode.PercentOutput, -0.7);
        }
        else if(open == -1){
            if(!limitSwitch.get()){
                climbMotor.set(ControlMode.PercentOutput, 0.0);
            }
            else{
                climbMotor.set(ControlMode.PercentOutput, 0.7);
            }
            
        }
        else{
            climbMotor.set(ControlMode.PercentOutput, 0);
        }
    }

    @Override
    public void periodic(){
        SmartDashboard.putBoolean("Limit Switch", limitSwitch.get());
    }





}