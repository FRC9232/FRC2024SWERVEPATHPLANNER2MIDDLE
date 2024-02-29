package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ColorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
public class IntakeSetCommand extends Command{
    
    private final IntakeSubsystem intakeSubsystem;
    private final ColorSubsystem colorSubsystem;
    private final ShooterSubsystem shooterSubsystem;
    private String durum;
    private boolean finished;
    public IntakeSetCommand(IntakeSubsystem intakeSubsystem,ColorSubsystem colorSubsystem,ShooterSubsystem shooterSubsystem, String durum){
        this.intakeSubsystem = intakeSubsystem;
        this.colorSubsystem = colorSubsystem;
        this.shooterSubsystem = shooterSubsystem;
        this.durum = durum;

        addRequirements(intakeSubsystem, colorSubsystem,shooterSubsystem);
    }
    public void execute(){
        finished = false;
        switch(durum){
            
            case "intake":
                intakeSubsystem.setIntake(colorSubsystem.a);
                finished = !colorSubsystem.a;
                break;
            case "intakeToShooter":
                intakeToShooter();
                finished=true;
                break;
            case "outtake":
                intakeSubsystem.setIntakeReversed();
                finished=true;
                break;
            case "stop":
                intakeSubsystem.setIntake(false);
                finished = true;
                break;
            case "autoIntake":
                intakeSubsystem.autoIntake();
                finished = true;
                break;
        }
        
        isFinished();
        
        SmartDashboard.putBoolean("Intake Durum", colorSubsystem.a);
    }
    public void intakeClose(){
        intakeSubsystem.setIntake(false);
    }
    public void intakeToShooter(){
        intakeSubsystem.intakeToShooter();

    }
    @Override
    public void end(boolean interrupted) {
        if(durum == "outtake" || durum == "intakeToShooter"){}
        else{
            intakeSubsystem.setIntake(false);
        }
        
  }
    @Override
    public boolean isFinished() {
        return finished;
  }
}
