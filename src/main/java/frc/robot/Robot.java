// note: refactor the code

//main imports
package frc.robot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
//import java.lang.Math;

// motor controllers
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.motorcontrol.PWMVictorSPX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;


import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

//encoders
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxRelativeEncoder;

//pneumatics
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

//joysticks
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.XboxController;

public class Robot extends TimedRobot {
  //variables for the left controllers
  WPI_TalonSRX driveLeftLead = new WPI_TalonSRX(1);
  PWMVictorSPX driveLeftFollower = new PWMVictorSPX(2);
  // Motor Controller group for the left motors
  MotorControllerGroup leftMotors = new MotorControllerGroup(driveLeftLead, driveLeftFollower);

  //variables for the right motor controllers
  WPI_TalonSRX driveRightLead = new WPI_TalonSRX(3);
  PWMVictorSPX driveRightFollower = new PWMVictorSPX(4);
  // Motor controller group for the right motors
  MotorControllerGroup rightMotors = new MotorControllerGroup(driveRightLead, driveRightFollower);

  //establishing differential drive
  DifferentialDrive drive = new DifferentialDrive(leftMotors, rightMotors);


  //variables for the arm controls
  CANSparkMax armYAxis = new CANSparkMax(11, MotorType.kBrushless);
  WPI_TalonSRX armXAxis = new WPI_TalonSRX(5);

  //Encoders for autonomous
  private RelativeEncoder yAxisEncoder;


  //variables for the pneumatics system
  Compressor compressor = new Compressor(1, PneumaticsModuleType.REVPH);
  DoubleSolenoid solenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, 0, 1);


  //joysticks
  Joystick driverController = new Joystick(1);
  XboxController armController = new XboxController(0);


  //Constants for controlling the arm. needs adjustments for this robot
  //current limit for the arm
  static final int ArmCurrentLimitA = 20;
  //Arm power output
  static final double ArmOutputPower = 0.1;

  double armPower = 0.0;

  //Varibles needed for the code
  boolean armUp = true; //Arm initialized to up because that's how it would start a match
  boolean burstMode = false;
  double lastBurstTime = 0; 

  double autoStart = 0;
  boolean goForAuto = true;

  //Conversion factor: #ticks x 1/4096ticks x gear ratio x 6pi inches/rotation x 1/12 inch = ? feet
  //remember to adjust the conversion factor so that it corresponds to our robot's setup (it should be correct)
  private final double kArmTick2Deg = 360.0/4096 * 1/7;
  private final double kDriveTick2Feet = 1.0/4096 * 6*Math.PI / 12;


  /*PID Variables: 
    This section contains variables needed to set the speed of the robot in relation to how far away it is from the target
  */
  //distance the robot needs to drive to get on the platform
  double distanceToDrive = 0;
  
  //PID Proportional constant: Adjust as necessary
  final double kp = 0;

  //PID Integral constant: Adjust as necessary
  final double kI = 0;

  //PID Derivative constant: Adjust as necessary
  final double kD = 0;

  //errorSum accumulation zone limit
  double iLimit = 1;

  //variable to account for small errors
  double errorSum = 0;
  double lastTimeStamp = 0;

  //varibales to account for the last known error rate
  double lastError = 0;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  
   
  //function for setting the initial conditions of all the hardware
   @Override
  public void robotInit() {

    //initial conditions for the drive motors
    leftMotors.setInverted(true);
    rightMotors.setInverted(true);
    drive.setDeadband(0.05);
    
    //initial conditions for the arm
    armYAxis.setInverted(false);
    armYAxis.setIdleMode(IdleMode.kBrake);
    armYAxis.setSmartCurrentLimit(ArmCurrentLimitA);
    ((CANSparkMax) armYAxis).burnFlash();
    armXAxis.setInverted(false);
    NeutralMode mode = NeutralMode.Brake;
    armXAxis.setNeutralMode(mode);

    //initial conditions for the intake
    compressor.disable();

    //add a thing on the dashboard to turn off auto if needed
    SmartDashboard.putBoolean("Go For Auto", false);
    goForAuto = SmartDashboard.getBoolean("Go For Auto", false);


    // intialize the left lead drive motor encoder
    driveLeftLead.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute, 0, 10);
    driveLeftLead.setSensorPhase(false);
    driveLeftLead.setSelectedSensorPosition(0, 0, 10);


    //initialize the right lead drive motor encoder
    driveRightLead.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute, 0, 10);
    driveRightLead.setSensorPhase(true);
    driveRightLead.setSelectedSensorPosition(0, 0, 10);


    //Initialize Y Axis encoder
    yAxisEncoder = armYAxis.getEncoder(SparkMaxRelativeEncoder.Type.kQuadrature, 4096);
    yAxisEncoder.setPosition(0);

    //enable a soft limit for motion in the horizontal direction
    armYAxis.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, true);
    armYAxis.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, true);

    //define the limit in the horizontal direction
    armYAxis.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, 5);
    armYAxis.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, 0);


    //Initialize X Axis Encoder
    armXAxis.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0,10);
    armXAxis.setSensorPhase(false);
    armXAxis.setSelectedSensorPosition(0, 0, 10);

    // setting boundaries for the x axis motor
    armXAxis.configReverseSoftLimitThreshold((int) (0 / kArmTick2Deg) , 10);
    armXAxis.configForwardSoftLimitThreshold((int) (0 / kArmTick2Deg), 10);

    //enhabling the limits
    armXAxis.configReverseSoftLimitEnable(true, 10);
    armXAxis.configForwardSoftLimitEnable(true, 10);
  }

  @Override
  public void robotPeriodic() {
    SmartDashboard.putNumber("Arm Ecoder Value", armXAxis.getSelectedSensorPosition() * kArmTick2Deg);
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

    //reset these variables to 0
    lastTimeStamp = Timer.getFPGATimestamp();
    errorSum = 0;
    lastError = 0;

    //check dashboard icon to ensure good to do auto
    goForAuto = SmartDashboard.getBoolean("Go For Auto", false);
  }

  //function that is called periodically during autonomous
  @Override
  public void autonomousPeriodic() {
    //set all encoder values to 0
    armXAxis.setSelectedSensorPosition(0,0,10);
    NeutralMode mode = NeutralMode.Brake;

    //get time since start of auto then run drive code for autonomous
    double autoTimeElapsed = Timer.getFPGATimestamp() - autoStart;

    //series of timed events making up the flow of auto
    if(goForAuto){
      //Y Axis: Raise the arm
      double liftDistance = 2;
      if (liftDistance < 2){
        armPower = 0.1;
      } else{
        armPower = 0;
        armYAxis.setIdleMode(IdleMode.kBrake);
      }
      armYAxis.set(armPower);
    
      //X Axis: Extend the arm
      double extensionDistance = armXAxis.getSelectedSensorPosition() * kArmTick2Deg;
      if (extensionDistance < 2) {
        armPower = 0.1;
      } else {
        armPower = 0;
        armXAxis.setNeutralMode(mode);
      }
      armXAxis.set(armPower);

      //Drop the cargo
      if (extensionDistance == 2){
        solenoid.set(Value.kForward);
      }

      //retract the arm if the solenoid has opened
      if (solenoid.get() == Value.kForward) {
        if (extensionDistance == 2) {
          armXAxis.set(-0.1);
        }  
      }
        //close the solenoid
        solenoid.set(Value.kReverse);

      //lower it if the arm has retracted
      if (extensionDistance == 0) {
        armYAxis.set(-0.1);
      }

      //drive backwards onto the platform - Updated to use PID
      double driveDistance = driveRightLead.getSelectedSensorPosition() * kDriveTick2Feet;
      if(liftDistance == 0) {
        if (autoTimeElapsed < 5) {
          distanceToDrive = 10;

          // calculate the error
          double error = distanceToDrive - driveDistance;

          //calculate the change in time
          double dT = Timer.getFPGATimestamp() - lastTimeStamp;

          //calculte the error rate
          double errorRate = (error - lastError) / dT;

          // Only accumulate the errors if the robot is close to the platform
          if (Math.abs(error) < iLimit) {
            errorSum += error * dT;
          }

          // set output speed according to the error and the small errors
          double driveSpeed = kp*error + kI*errorSum + kD*errorRate;

          leftMotors.set(driveSpeed);
          rightMotors.set(driveSpeed);

          //update last time stamp
          lastTimeStamp = Timer.getFPGATimestamp();
        }
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
      solenoid.set(Value.kForward);
      
    } else if(armController.getRightBumperPressed()){

      //fire the air the other way
      solenoid.set(Value.kReverse);
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