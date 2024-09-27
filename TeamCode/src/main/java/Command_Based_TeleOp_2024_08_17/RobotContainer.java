package Command_Based_TeleOp_2024_08_17;


import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.PerpetualCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.motors.CRServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

import Command_Based_TeleOp_2024_08_17.Commands.PowerVacuumCMD;
import Command_Based_TeleOp_2024_08_17.Commands.TeleOpJoystickRobotCentricCMD;
import Command_Based_TeleOp_2024_08_17.Commands.TelemetryManagerCMD;
import Command_Based_TeleOp_2024_08_17.Subsystems.MecanumDriveBaseSubsystem;
import Command_Based_TeleOp_2024_08_17.Subsystems.TelemetryManagerSubsystem;
import Command_Based_TeleOp_2024_08_17.Subsystems.VacuumSubsystem;


@TeleOp(name = "Command Base Test")
public class RobotContainer extends CommandOpMode {


    private  MecanumDriveBaseSubsystem mecanumDriveBaseSub;
    private  TelemetryManagerSubsystem telemetryManagerSub;
    private   VacuumSubsystem vacuumSubsystem;




    double fwdPwr;
    double strafePwr;
    double rotationPwr;



    HardwareMap m_HardwareMap;
    public GamepadEx driverOP;

    @Override
    public void initialize() {
        fwdPwr = -gamepad1.left_stick_y;
        strafePwr = -gamepad1.left_stick_x;
        rotationPwr = -gamepad1.right_stick_x;
        m_HardwareMap = hardwareMap;


        mecanumDriveBaseSub = new MecanumDriveBaseSubsystem(m_HardwareMap);
        telemetryManagerSub = new TelemetryManagerSubsystem();
        vacuumSubsystem = new VacuumSubsystem(m_HardwareMap);


        driverOP = new GamepadEx(gamepad1);




        telemetryManagerSub.setDefaultCommand(new PerpetualCommand(new TelemetryManagerCMD(telemetryManagerSub)));



        mecanumDriveBaseSub.setDefaultCommand(new TeleOpJoystickRobotCentricCMD(mecanumDriveBaseSub,
                telemetryManagerSub.getTelemetryObject(), driverOP::getLeftY, driverOP::getLeftX, driverOP::getRightX));
        vacuumSubsystem.setDefaultCommand(new PowerVacuumCMD(vacuumSubsystem, 1));


    }




}

