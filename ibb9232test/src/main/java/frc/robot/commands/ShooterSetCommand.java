package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.ColorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;



public class ShooterSetCommand extends Command {
    
    private final ShooterSubsystem shooterSubsystem;
    private final ColorSubsystem colorSubsystem;
    private String durum;
    private boolean finished;
    public ShooterSetCommand(ShooterSubsystem shooterSubsystem,ColorSubsystem colorSubsystem,String durum){
        this.shooterSubsystem = shooterSubsystem;
        this.colorSubsystem = colorSubsystem;
        this.durum = durum;
        addRequirements(shooterSubsystem, colorSubsystem);
    } 
    public void execute(){
        finished = false;
        switch(durum){
            case "speaker":
                shooterSubsystem.setShooter(true);
                finished = true;
                break;
            case "amph":
                shooterSubsystem.amphShoot();
                finished = true;
                break;
            case "stop":
                shooterSubsystem.setShooter(false);
                finished = true;
                break;
        }
        isFinished();
    }
    
    
    
    public void StartShooter(){

        shooterSubsystem.setShooter(true);
    }
    
    @Override
    public void end(boolean interrupted) {
        //shooterSubsystem.setShooter(false);
        
    }
    
    @Override
    public boolean isFinished() {
        return finished;
    }
}
