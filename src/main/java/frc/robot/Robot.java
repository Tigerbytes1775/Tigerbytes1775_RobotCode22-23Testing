// note: refactor the code

//main imports
package frc.robot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;

// motor controllers
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.motorcontrol.PWMVictorSPX;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxRelativeEncoder;

//pneumatics
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;

//joysticks
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.XboxController;

public class Robot extends TimedRobot {
  // variables for the left controllers
  PWMVictorSPX driveLeftA = new PWMVictorSPX(1);
  PWMVictorSPX driveLeftB = new PWMVictorSPX(2);
  // Motor controller group for the left motors
  MotorControllerGroup leftMotors = new MotorControllerGroup(driveLeftA, driveLeftB);

  // variables for the right motor controllers
  PWMVictorSPX driveRightA = new PWMVictorSPX(3);
  PWMVictorSPX driveRightB = new PWMVictorSPX(4);
  // Motor controller group for the right motors
  MotorControllerGroup rightMotors = new MotorControllerGroup(driveRightA, driveRightB);

  // establishing differential drive
  DifferentialDrive drive = new DifferentialDrive(leftMotors, rightMotors);


  // variables for the arm controls
  CANSparkMax armYAxis = new CANSparkMax(11, MotorType.kBrushless);
  PWMVictorSPX armXAxis = new PWMVictorSPX(5);

  // Encoders for autonomous
  private RelativeEncoder yAxisEncoder;


  //variables for the pneumatics system
  Compressor compressor = new Compressor(1, PneumaticsModuleType.REVPH);
  DoubleSolenoid solenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, 0, 1);


  // joysticks
  Joystick driverController = new Joystick(1);
  XboxController armController = new XboxController(0);


  //Constants for controlling the arm. needs adjustments for this robot
  final double armTimeUp = 0.5;
  //current limit for the arm
  static final int ArmCurrentLimitA = 20;
  //Arm power output
  static final double ArmOutputPower = 0.1;
  //time to move the arm
  static final double ArmExtendTime = 2.0;
  double armPower = 0.0;

  //Varibles needed for the code
  boolean armUp = true; //Arm initialized to up because that's how it would start a match
  boolean burstMode = false;
  double lastBurstTime = 0; 

  double autoStart = 0;
  boolean goForAuto = true;


  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  
   @Override

  //function for setting the initial conditions of all the hardware
  public void robotInit() {

    //initial conditions for the drive motors
    leftMotors.setInverted(true);
    rightMotors.setInverted(false);
    drive.setDeadband(0.05);
    
    //initla conditions for the arm
    armYAxis.setInverted(true);
    armYAxis.setIdleMode(IdleMode.kBrake);
    armYAxis.setSmartCurrentLimit(ArmCurrentLimitA);
    ((CANSparkMax) armYAxis).burnFlash();
    armXAxis.setInverted(false);

    //initial conditions for the intake
    compressor.disable();

    //add a thing on the dashboard to turn off auto if needed
    SmartDashboard.putBoolean("Go For Auto", false);
    goForAuto = SmartDashboard.getBoolean("Go For Auto", false);

    //Initialize encoders
    yAxisEncoder = armYAxis.getEncoder(SparkMaxRelativeEncoder.Type.kQuadrature, 4096);
    yAxisEncoder.setPosition(0);

    armYAxis.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, true);
    armYAxis.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, true);

    armYAxis.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, 5);
    armYAxis.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, 0);

  }

  /**
   * *set the arm output power. Positive is out, negative is in
   * 
   * @param percent
   */

    //function to set the arm output power in the vertical direction
  public void setArmYAxisMotor(double percent) {
    armYAxis.set(percent);
    SmartDashboard.putNumber("armYAxis power(%)", percent);
    SmartDashboard.putNumber("arm motor current (amps)", armYAxis.getOutputCurrent());
    SmartDashboard.putNumber("arm motor temperature(C)", armYAxis.getMotorTemperature());
  }

  //function to set the arm output power in the horizontal direction
  public void setArmXAxisMotor(double percent) {
    armXAxis.set(percent);
    SmartDashboard.putNumber("armXaxis power(%)", percent);
    /*SmartDashboard.putNumber("armXAxis motor current (amps)", armXAxis.getVoltage());
    SmartDashboard.putNumber("armXAxis motor temperature(C)", armXAxis.getMotorTemperature());*/
  }
  
 /**
  * set the arm output power.
  *
  * @param percent desired speed
  * @param amps current limit
  */
  
  //function for starting autonomous
  @Override
  public void autonomousInit() {
    //get a time for auton start to do events based on time later
    autoStart = Timer.getFPGATimestamp();
    //check dashboard icon to ensure good to do auto
    goForAuto = SmartDashboard.getBoolean("Go For Auto", false);
  }

  //function that is called periodically during autonomous
  @Override
  public void autonomousPeriodic() {
    //arm control code for autonomous
    if (yAxisEncoder.getPosition() < 2){
      armPower = 0.1;

    } else{
      armPower = 0;
      armYAxis.setIdleMode(IdleMode.kBrake);
    }
    armYAxis.set(armPower);
    
    //get time since start of auto then run drive code for autonomous
    double autoTimeElapsed = Timer.getFPGATimestamp() - autoStart;
    if(goForAuto){

      //series of timed events making up the flow of auto
      if(autoTimeElapsed < 3){
        //drop the ball
        setArmXAxisMotor(0.1);
        
      }if(autoTimeElapsed < 3){
        //stop spitting out the ball and drive backwards *slowly* for three seconds
        driveLeftA.set(-0.3);
        driveLeftB.set(-0.3);
        driveRightA.set(-0.3);
        driveRightB.set(-0.3);
      } else {
        //do nothing for the rest of auto
        driveLeftA.set(0);
        driveLeftB.set(0);
        driveRightA.set(0);
        driveRightB.set(0);
      }
    }

  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {}

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    //Set up arcade steer
    double forward = -driverController.getRawAxis(1);
    double turn = -driverController.getRawAxis(4);

    //set up arcade drive
    drive.arcadeDrive(forward, turn);
    

    //Code for the arm
    
    // motion for the arm in the vertical direction
    if (armController.getLeftY() > 0.5) {
      //raise the arm
      armPower = ArmOutputPower;
    }
    else if (armController.getLeftY() < -0.5) {
      //lower the arm
      armPower = -ArmOutputPower;
    }
    else {
      //do nothing and let it sit where it is
      armPower = 0.0;
      armYAxis.setIdleMode(IdleMode. kBrake);
    }
    setArmYAxisMotor(armPower);
    
    // motion for the arm in the horizontal direction
    if (armController.getLeftTriggerAxis() > 0.5) {
      //extend the arm
      armPower = ArmOutputPower;
    }
    else if (armController.getRightTriggerAxis() > 0.5) {
      //retract the arm
      armPower = -ArmOutputPower;
    }
    else {
      // do nothing and let it sit where it is
      armPower = 0.0;
      armXAxis.stopMotor();
    }
    setArmXAxisMotor(armPower);


    //Intake controls

    //solenoid controls
    if(armController.getLeftBumperPressed()){

      //fire the air one way
      solenoid.set(DoubleSolenoid.Value.kForward);
      
    } else if(armController.getRightBumperPressed()){

      //fire the air the other way
      solenoid.set(DoubleSolenoid.Value.kReverse);
    }

    //compressor controls
    if (armController.getAButton()) {

      //enable the compressdor
      compressor.enableAnalog(0, 50);

    } else if (armController.getBButton()) {

      //disable the compressor
      compressor.disable();
    }
  }

  //function for disabling everything at the end of the game
  @Override
  public void disabledInit() {
    //On disable turn off everything
    //done to solve issue with motors "remembering" previous setpoints after reenable

    leftMotors.set(0);
    rightMotors.set(0);
    
    armYAxis.set(0);
    armXAxis.set(0);

    compressor.disable();
    solenoid.set(DoubleSolenoid.Value.kReverse);
  }

}