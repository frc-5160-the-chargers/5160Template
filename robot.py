import wpilib
import wpilib.drive
import magicbot
import navx

from networktables import NetworkTables
from wpilib import SmartDashboard as dash
from ctre import WPI_TalonSRX
from rev import CANSparkMax, MotorType
from rev.color import ColorSensorV3
from networktables import NetworkTables

#Import robot components as classes to interface with (henceforth referred to as component classes)
from oi import Driver, Sysop
from utils import config_spark, config_talon
from robotmap import RobotMap
from dash import Tunable
from components.limelight import Limelight

from components.drivetrain import Drivetrain, Powertrain, DrivetrainState, EncoderSide
from components.sensors import Encoders, NavX
from components.shooter import Shooter
from components.serializer import Serializer
from components.intake import IntakeRoller, Intake

#Maybe add this one later
#from components.position_approximation import PosApprox

#"Main class", all functionality is ultimately contained/concentrated here
class robot(magicbot.MagicBot):
    #Initialize component classes
    powertrain: Powertrain
    encoders: Encoders
    navx: NavX
    drivetrain: Drivetrain
    limelight : Limelight
    shooter : Shooter
    serializer : Serializer
    intake_roller: IntakeRoller
    intake: Intake
    #location: PosApprox

    def createObjects(self):
        #Grab the network table (data feed) for the limelight
        self.limelight_table = NetworkTables.getTable("limelight")

        #Group motors on the left side of the robot into a "speed controller group" which links inputs/outputs between them
        motor_objects_left = [
            CANSparkMax(port, MotorType.kBrushless) for port in RobotMap.Drivetrain.motors_left
        ]
        self.left_motors = wpilib.SpeedControllerGroup(motor_objects_left[0], *motor_objects_left[1:])

        #Does the same thing for the right
        motor_objects_right = [
            CANSparkMax(port, MotorType.kBrushless) for port in RobotMap.Drivetrain.motors_right
        ]
        self.right_motors = wpilib.SpeedControllerGroup(motor_objects_right[0], *motor_objects_right[1:])

        #Configure motor controllers based on parameters set in the robot map
        for motor in motor_objects_left + motor_objects_right:
            config_spark(motor, RobotMap.Drivetrain.motor_config)

        #Create a differential drive setup with the motor groups to control the robots movement
        self.differential_drive = wpilib.drive.DifferentialDrive(self.left_motors, self.right_motors)
        self.differential_drive.setMaxOutput(RobotMap.Drivetrain.max_motor_power)

        #Create and configure encoder objects based on robotmap parameters to measure kinematics
        self.left_encoder = wpilib.Encoder(RobotMap.Encoders.left_encoder_b, RobotMap.Encoders.left_encoder_a)
        self.right_encoder = wpilib.Encoder(RobotMap.Encoders.right_encoder_b, RobotMap.Encoders.right_encoder_a)
        self.right_encoder.setReverseDirection(False)
        self.left_encoder.setDistancePerPulse(RobotMap.Encoders.distance_per_pulse)
        self.right_encoder.setDistancePerPulse(RobotMap.Encoders.distance_per_pulse)

        #Create a gyroscope object using SPI communication with a navx module
        self.navx_ahrs = navx.AHRS.create_spi()

        #Create driver interfaces using an xbox controller
        self.driver = Driver(wpilib.XboxController(0))
        self.sysop = Sysop(wpilib.XboxController(1))

        #Create and configure the shooter motor
        self.shooter_motor = WPI_TalonSRX(RobotMap.Shooter.motor_port)
        config_talon(self.shooter_motor, RobotMap.Shooter.motor_config) 

        #Create and configure the serializer motor
        self.serializer_motor = WPI_TalonSRX(RobotMap.Serializer.motor_port)
        config_talon(self.serializer_motor, RobotMap.Serializer.motor_config)

        #Create and configure intake motor
        self.intake_roller_motor = WPI_TalonSRX(RobotMap.IntakeRoller.motor_port)
        self.intake_roller_motor.configPeakOutputForward(RobotMap.IntakeRoller.max_power)
        self.intake_roller_motor.configPeakOutputReverse(-RobotMap.IntakeRoller.max_power)
        config_talon(self.intake_roller_motor, RobotMap.IntakeRoller.motor_config)

        #Launch network tables
        NetworkTables.initialize(server="roborio");
        #Launch camera server for visual feed
        wpilib.CameraServer.launch()

    #Reset all subsystems to their default state
    def reset_subsystems(self):
        self.drivetrain.reset()
        self.limelight.reset()
    
    #Runs once every time teleop mode is enabled
    def teleopInit(self):
        self.reset_subsystems()
    
    #Loops while teleop mode is enabled, starts after init
    def teleopPeriodic(self):
        try:
            #Get input from drivers
            driver_x, driver_y = self.driver.get_curvature_output()

            #Give driver commands to drivetrain if manual drive is enabled
            if self.drivetrain.state == DrivetrainState.MANUAL_DRIVE:
                self.drivetrain.curvature_drive(driver_y, driver_x)
            
            #Enable turbo mode (set maximum power values to those specified by turbo mode)
            self.drivetrain.set_power_scaling(self.driver.process_turbo_mode())

            #Set drivetrain to manual control if manual override button is pressed
            if self.driver.get_manual_control_override():
                self.drivetrain.state = DrivetrainState.MANUAL_DRIVE
        except:
            #Notify drivers if something goes wrogn
            print("DRIVETRAIN ERROR")
        
        try:
            #Enable intake suction
            if self.sysop.get_intake_intake():
                self.intake_roller.intake()
            #Enable intake spitout
            elif self.sysop.get_intake_outtake():
                self.intake_roller.outtake()
            #Disable intake
            else:
                self.intake_roller.stop()
        except:
            #Notify drivers if something goes wrong
            print("INTAKE ROLLER ERROR")

#Enable robot, this code always runs first
if __name__ == '__main__':
    wpilib.run(robot)