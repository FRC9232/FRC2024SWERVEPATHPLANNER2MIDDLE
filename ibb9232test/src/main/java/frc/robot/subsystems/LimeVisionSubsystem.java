package frc.robot.subsystems;

//import frc.robot.commands.LimeVisionCommand;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
//import edu.wpi.first.wpilibj.drive.Vector2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * An example subsystem. You can replace me with your own Subsystem.
 */
public class LimeVisionSubsystem extends SubsystemBase{
	// Put methods for controlling this subsystem
	// here. Call these from Commands.
	
	
    
    public double aimOmega;
    public double distanceVx;
    private static final double KLIMEAIM = 0.075; //aim alırken 0 a yaklaştıkça yavaşlaması için kullanılan kat sayı değeri
    private static final double KLIMEDIS = -0.6; //robot apriltag e olan uzaklığını ayarlarken 0 a yaklaştıkça yavaşlaması için kullanılan kat sayı değeri
    private static final double LIME_AIM_LIMIT = 5; // aim almayı hangi açıya gelince bırakacağı limit
    private static final double LIME_DISTANCE_LIMIT_METERS = 1; // Robotun apriltag e ne kadar uzakta durması istenen değişken
    private static final double LIME_MOUNT_ANGLE_DEGREES = 20; //limelightın monte edilme açısı
    private static final double LIME_LENS_MOUNT_HEIGHT_METERS = 0.20; //limelight ın lensinin yerden yüksekliği
    private static final double GOAL_HEIGHT_METERS = 1.445; // hedefin yerden yüksekliği(speakerın apriltagleri için bu değer 1.445)
    
	
	
	//Create variables
	public double targetD;
	public boolean hasTarget;
	public double xOffset;
	public double yOffset;
	public double area;
	public double skew;
	public double LEDMode;
	public double camMode;
	public double pipeline;


	// public static final Vector2d resolution = new Vector2d(120,120);
	
	public NetworkTable getLimetable() {
		return NetworkTableInstance.getDefault().getTable("limelight");
	}

	
	//public void initDefaultCommand() {
		// Set the default command for a subsystem here.
		//setDefaultCommand(new LimeVisionCommand());
	//}
	
	//Does the camera proccessor have a target?
	public boolean getHasTarget() {
		targetD = getLimetable().getEntry("tv").getDouble(0); 
		if(targetD == 0) {
			hasTarget = false;
		}else if(targetD == 1) {
			hasTarget = true;
		}
		return hasTarget;
	}
	
	// Horizontal Offset From Crosshair To Target (-27 degrees to 27 degrees)
	public double getXOffset() {
		xOffset = getLimetable().getEntry("tx").getDouble(0);
		return xOffset;
	}
	
	//Vertical Offset From Crosshair To Target (-20.5 degrees to 20.5 degrees)
	public double getYOffset() {
		yOffset = getLimetable().getEntry("ty").getDouble(0);
		return yOffset;
	}
	
	//Target Area (0% of image to 100% of image)
	public double getArea() {
		area = getLimetable().getEntry("ta").getDouble(0);
		return area;
	}
	
	//Skew or rotation (-90 degrees to 0 degrees)
	public double getSkew() {
		skew = getLimetable().getEntry("ts").getDouble(0);
		return skew;
	}

	//Limelight LED state
	public double getLEDMode() {
		LEDMode = getLimetable().getEntry("ledMode").getDouble(1);
		return LEDMode;
	}
	
	//Limelight Camera state
	public double getCamMode() {
		camMode = getLimetable().getEntry("camMode").getDouble(0);
		return camMode;
	}
	
	//get current pipeline that is being used
	public double getPipeline() {
		pipeline = getLimetable().getEntry("pipeline").getDouble(0);
		return pipeline;
	}
	
	//Set the LED mode of the limelight
	public void switchLED() {
		if(getLEDMode() == 0) {
			getLimetable().getEntry("ledMode").setDouble(1);
			SmartDashboard.putString("LED Mode", "Off");
		}else if(getLEDMode() == 1) {
			getLimetable().getEntry("ledMode").setDouble(0);
			SmartDashboard.putString("LED Mode", "On");
		}else if(getLEDMode() == 2) {
			getLimetable().getEntry("ledMode").setDouble(1);
			SmartDashboard.putString("LED Mode", "Off");
		}
	}
	
	//Set the camera mode
	public void switchCamera() {
		if(getCamMode() == 0) {
			getLimetable().getEntry("camMode").setDouble(1);
			SmartDashboard.putString("Camera Mode", "Camera");
		}else if(getCamMode() == 1) {
			getLimetable().getEntry("camMode").setDouble(0);
			SmartDashboard.putString("Camera Mode", "Vision");
		}
	}
	
	
	//Set the pipeline
	public void setPipeline(double pipeline) {
		getLimetable().getEntry("pipeline").setDouble(pipeline);
		SmartDashboard.putNumber("Pipeline: ", pipeline);
	}
	
    //limelight'ın aim alırken msDrive'ın omega değerinin ne olması gerektiğini belirleyen
    public double getAimOmega() {
        setPipeline(0);
        if (getXOffset() > LIME_AIM_LIMIT){
            
            aimOmega = getXOffset()*KLIMEAIM;
        }
        else if (getXOffset() < -1 * LIME_AIM_LIMIT){
            
            aimOmega = getXOffset()*KLIMEAIM;
        }
        else{
            
            aimOmega = 0;
        
        }
        
        return aimOmega;
    }
    
    // limelight'ın hedefe aim alıp almadığını kontrol eden method
    public boolean isLimeAimed(){
        if (getXOffset() < LIME_AIM_LIMIT && getXOffset() > -1 * LIME_AIM_LIMIT ){
            
            return true;

        }
        
        else{
            return false;
        }
    }
    
    //Robotun AprilTag e olan uzaklığını hesaplayan method
    
    public double getLimeDistance(){

        double angleToGoalRadians = (LIME_MOUNT_ANGLE_DEGREES + getYOffset()) * (3.14159 / 180.0);
        double distanceFromLimelightToGoalMeters = (GOAL_HEIGHT_METERS - LIME_LENS_MOUNT_HEIGHT_METERS) / Math.tan(angleToGoalRadians);
        
        return distanceFromLimelightToGoalMeters;

    }
    //Robotun AprilTag e olan uzaklığını msDrive için vX e çeviren method
    
    public double getLimeDistanceVx(){
        
        if (getLimeDistance() > (LIME_DISTANCE_LIMIT_METERS + 0.1)){
            
            distanceVx = getLimeDistance()*KLIMEDIS;
        }
        else if (getXOffset() < (LIME_DISTANCE_LIMIT_METERS - 0.1)){
            
            distanceVx = getLimeDistance()*KLIMEDIS * -1;
        }
        else{
            
            distanceVx = 0;
        
        }
        
        return distanceVx;


    }


}