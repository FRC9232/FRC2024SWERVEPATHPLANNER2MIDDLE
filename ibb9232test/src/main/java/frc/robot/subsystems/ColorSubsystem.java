package frc.robot.subsystems;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.ColorSensorV3;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorMatch;

public class ColorSubsystem extends SubsystemBase{
    
    public final I2C.Port i2cPort = I2C.Port.kOnboard;

    public final ColorSensorV3 m_colorSensor = new ColorSensorV3(i2cPort);

    public final ColorMatch m_colorMatcher = new ColorMatch();
    public boolean a;

    public final Color kBlueTarget = new Color(0.143, 0.427, 0.429);
    public final Color kGreenTarget = new Color(0.197, 0.561, 0.240);
    public final Color kRedTarget = new Color(0.561, 0.232, 0.114);
    public final Color kYellowTarget = new Color(0.361, 0.524, 0.113);
    public final Color kOrangeTarget = new Color(0.461, 0.401, 0.123);

    public void robotInit() {
    
      m_colorMatcher.addColorMatch(kBlueTarget);
      m_colorMatcher.addColorMatch(kGreenTarget);
      m_colorMatcher.addColorMatch(kRedTarget);
      m_colorMatcher.addColorMatch(kYellowTarget); 
      m_colorMatcher.addColorMatch(kOrangeTarget); 
  
  
    }
    public boolean hasIntakeTaken(){

      Color detectedColor = m_colorSensor.getColor();
      
      /**
       * Run the color match algorithm on our detected color
       */
      String colorString;
      ColorMatchResult match = m_colorMatcher.matchClosestColor(detectedColor);
      int proximity = m_colorSensor.getProximity();
  
      if (match.color == kBlueTarget) {
        colorString = "Blue";
      } else if (match.color == kRedTarget) {
        colorString = "Red";
      } else if (match.color == kGreenTarget) {
        colorString = "Green";
      } else if (match.color == kYellowTarget) {
        colorString = "Yellow";
      } else if (match.color == kOrangeTarget) {
        colorString = "Orange";
      } else {
        colorString = "Unknown";
      }
      SmartDashboard.putNumber("Red", detectedColor.red);
      SmartDashboard.putNumber("Green", detectedColor.green);
      SmartDashboard.putNumber("Blue", detectedColor.blue);
      SmartDashboard.putNumber("Confidence", match.confidence);
      SmartDashboard.putNumber("Proximity", proximity);
      SmartDashboard.putString("Detected Color", colorString);
      
      /**
       * Open Smart Dashboard or Shuffleboard to see the color detected by the 
       * sensor.
       */
    

      boolean intake;
      if(colorString == "Orange"){
          intake = false;
      }
      else{
          intake = true;
      }
      return intake;
  
      
    }
    @Override
    public void periodic(){
  

      
      m_colorMatcher.addColorMatch(kBlueTarget);
      m_colorMatcher.addColorMatch(kGreenTarget);
      m_colorMatcher.addColorMatch(kRedTarget);
      m_colorMatcher.addColorMatch(kYellowTarget); 
      m_colorMatcher.addColorMatch(kOrangeTarget);
      Color detectedColor = m_colorSensor.getColor();

      /**
       * Run the color match algorithm on our detected color
       */
      String colorString;
      ColorMatchResult match = m_colorMatcher.matchClosestColor(detectedColor);
      int proximity = m_colorSensor.getProximity();
  
      if (match.color == kBlueTarget) {
        a=true;
        colorString = "Blue";
      } else if (match.color == kRedTarget) {
        a=true;
        colorString = "Red";
      } else if (match.color == kGreenTarget) {
        a=true;
        colorString = "Green";
      } else if (match.color == kYellowTarget) {
        a=true;
        colorString = "Yellow";
      } else if (match.color == kOrangeTarget) {
        a=false;
        colorString = "Orange";
      } else {
        a=true;
        colorString = "Unknown";
      }
      SmartDashboard.putNumber("Red", detectedColor.red);
      SmartDashboard.putNumber("Green", detectedColor.green);
      SmartDashboard.putNumber("Blue", detectedColor.blue);
      SmartDashboard.putNumber("Confidence", match.confidence);
      SmartDashboard.putNumber("Proximity", proximity);
      SmartDashboard.putString("Detected ", colorString); 
    
    }

}
