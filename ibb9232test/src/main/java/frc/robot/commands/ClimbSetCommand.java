package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimbSubsystem;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;



public class ClimbSetCommand extends Command {
    
    private final ClimbSubsystem climbSubsystem;
    private int open;
    private boolean finished;

    
    public ClimbSetCommand(ClimbSubsystem climbSubsystem, int open){
        this.climbSubsystem = climbSubsystem;
        this.open = open;
        addRequirements(climbSubsystem);
    }

    @Override
    public void initialize() {
 
    }

    public void execute(){
        finished = false;
        switch(open){
            case 1:
                climbSubsystem.setClimb(1);
                break;
            case 0:
                climbSubsystem.setClimb(0);
                break;
            case -1:
            System.out.println("aa");
                if(!climbSubsystem.limitSwitch.get()){
                    System.out.println("bb");
                    climbSubsystem.setClimb(0);
                    break;
                }
                else{
                    System.out.println("cc");
                    climbSubsystem.setClimb(-1);
                    break;
                }
                
        }
    }


    @Override
    public boolean isFinished() {
           if(this.open == -1 && !climbSubsystem.limitSwitch.get()){
            climbSubsystem.climbMotor.set(TalonSRXControlMode.PercentOutput, 0);    
            return false;
           }
        return true;
    }
    













 }