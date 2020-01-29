/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.core;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.core.components.ControlSystem;
import frc.robot.core.components.Drivetrain;
import frc.robot.core.components.Climber.Climber;
import frc.robot.core.components.ControlSystem.AuxillaryButtons;
import frc.robot.core.components.Launcher.Launcher;
import frc.robot.core.components.Transport.Transport;
import frc.robot.core.utils.Hardware;
import frc.robot.core.utils.SensorManager;
import frc.robot.core.components.WheelOfFortune.ColorGetterFromField;
import frc.robot.core.components.WheelOfFortune.WheelArm;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  public static ControlSystem controlSystem;
  
  private Drivetrain drivetrain;
  private Launcher launcher;
  private Transport transport;
  private Climber climber;
  private boolean isClimbed = false;
  private WheelArm colorArm;
  private ColorGetterFromField FieldColor;
  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  @Override
  public void robotInit() {


     m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);

    controlSystem = new ControlSystem();
    controlSystem.robotInit();

    drivetrain = new Drivetrain();
    drivetrain.robotInit();

    //launcher = new Launcher(Hardware.LauncherMotor, Hardware.LauncherEncoder, transport);

    climber = new Climber();
    
    colorArm = new WheelArm();

  }

  /**
   * This function is called every robot packet, no matter the mode. Use
   * this for items like diagnostics that you want ran during disabled,
   * autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before
   * LiveWindow and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {

  }

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different autonomous modes using the dashboard. The sendable
   * chooser code works with the Java SmartDashboard. If you prefer the
   * LabVIEW Dashboard, remove all of the chooser code and uncomment the
   * getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to
   * the switch structure below with additional strings. If using the
   * SendableChooser make sure to add them to the chooser code above as well.
   */
  @Override
  public void autonomousInit() {
    m_autoSelected = m_chooser.getSelected();
    // autoSelected = SmartDashboard.getString("Auto Selector",
    // defaultAuto);
    System.out.println("Auto selected: " + m_autoSelected);
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    switch (m_autoSelected) {
      case kCustomAuto:
        // Put custom auto code here
        break;
      case kDefaultAuto:
      default:
        // Put default auto code here
        break;
    }
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
    /*drivetrain.teleopPeriodic();
    if (controlSystem.getButton(AuxillaryButtons.Fire)) {
      if (true){// TODO: need to change to if aim is right... if button pressed the robot fires all of it's current balls
        //launcher.shoot(0.1);//need to velocity based upon the aiming
        SmartDashboard.putString("CurrentObjective", "Shooting");
      }
  }
    if (controlSystem.getButton(AuxillaryButtons.Intake)) {// if button pressed the robot intakes
      //transport.intake();
      SmartDashboard.putString("CurrentObjective", "Intaking");

    }
    if (controlSystem.getButton(AuxillaryButtons.ColorWheel)) {
      //colorArm.Lower();
      if (FieldColor.getColorToFind() != 'n') {
        //colorArm.RotateForColor(FieldColor.getColorToFind());
        SmartDashboard.putString("CurrentObjective", "RotateForColor");
      } else {
      //colorArm.RotateControl();
        SmartDashboard.putString("CurrentObjective", "RotateControl");
      }
      //colorArm.Raise();
    }
    if (controlSystem.getButton(AuxillaryButtons.Climb)) {//if button pressed then climb!
      //climber.climb(6.5, 0.5);
      SmartDashboard.putString("CurrentObjective", "Climbing");
      isClimbed = true;
    }
    if (isClimbed == true) {//if climbed then move along bar!!
      //climber.move();
      SmartDashboard.putString("CurrentObjective", "Moving");
    }*/
  }

  /**
   * This function is called periodically during test mode.
   */

  @Override
  public void testPeriodic() {
<<<<<<< HEAD
<<<<<<< Updated upstream
=======
    SensorManager.calibrateColor();
    System.out.println("r"+Hardware.m_colorSensor.getRed());
    System.out.println("g"+Hardware.m_colorSensor.getGreen());
    System.out.println("b"+Hardware.m_colorSensor.getBlue());

>>>>>>> Stashed changes
=======
    SensorManager.calibrateColor();

>>>>>>> Dylan-Cleanup
  }
}
