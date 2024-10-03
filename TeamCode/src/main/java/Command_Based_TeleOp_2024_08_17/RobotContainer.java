package Command_Based_TeleOp_2024_08_17;


import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.PerpetualCommand;
import com.arcrobotics.ftclib.command.button.Button;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.motors.CRServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

import Command_Based_TeleOp_2024_08_17.Commands.MoveShoulderCMD;
import Command_Based_TeleOp_2024_08_17.Commands.PowerVacuumCMD;
import Command_Based_TeleOp_2024_08_17.Commands.TeleOpJoystickRobotCentricCMD;
import Command_Based_TeleOp_2024_08_17.Commands.TelemetryManagerCMD;
import Command_Based_TeleOp_2024_08_17.Subsystems.MecanumDriveBaseSubsystem;
import Command_Based_TeleOp_2024_08_17.Subsystems.ShoulderSubsystem;
import Command_Based_TeleOp_2024_08_17.Subsystems.TelemetryManagerSubsystem;
import Command_Based_TeleOp_2024_08_17.Subsystems.VacuumSubsystem;


@TeleOp(name = "Command Base Test")
public class RobotContainer extends CommandOpMode {


    private final MecanumDriveBaseSubsystem mecanumDriveBaseSub = new MecanumDriveBaseSubsystem();
    private final TelemetryManagerSubsystem telemetryManagerSub = new TelemetryManagerSubsystem();
    private  final VacuumSubsystem vacuumSubsystem = new VacuumSubsystem();
    private final ShoulderSubsystem shoulderSub = new  ShoulderSubsystem();
    private BNO055IMU imu;


    double fwdPwr;
    double strafePwr;
    double rotationPwr;

    Motor frontLeft;
    Motor frontRight;
    Motor backLeft;
    Motor backRight;
    Motor shoulderMotor;

    ServoEx targetVacuumServo;
    CRServo ContinousVacuumServo;

    public GamepadEx driverOP;
    public Button vacuumButton;



    @Override
    public void initialize() {
        fwdPwr = -gamepad1.left_stick_y;
        strafePwr = -gamepad1.left_stick_x;
        rotationPwr = -gamepad1.right_stick_x;

        //TODO move motor, sensor and IMU setups into their subsystem
        //TODO put constant tags into constants
        frontLeft = new Motor(hardwareMap, "front_left");
        frontRight = new Motor(hardwareMap, "front_right");
        backLeft = new Motor(hardwareMap, "back_left");
        backRight = new Motor(hardwareMap, "back_right");

        shoulderMotor = new Motor(hardwareMap,"shoulder_motor");
        shoulderMotor.setRunMode(Motor.RunMode.RawPower);

        ContinousVacuumServo = new CRServo(hardwareMap, "Vacuum_Servo");

        frontLeft.setRunMode(Motor.RunMode.RawPower);
        frontRight.setRunMode(Motor.RunMode.RawPower);
        backLeft.setRunMode(Motor.RunMode.RawPower);
        backRight.setRunMode(Motor.RunMode.RawPower);

        ContinousVacuumServo.setRunMode(Motor.RunMode.RawPower);

        backLeft.setInverted(true);
        backRight.setInverted(true);
        driverOP = new GamepadEx(gamepad1);
        vacuumButton = new GamepadButton(driverOP, GamepadKeys.Button.A);


        BNO055IMU.Parameters myIMUparameters;

        myIMUparameters = new BNO055IMU.Parameters();


        myIMUparameters.angleUnit = myIMUparameters.angleUnit.RADIANS;

        myIMUparameters.calibrationDataFile = "BNO055IMUCalibration.json";

        imu = hardwareMap.get(BNO055IMU.class, "imu");

        imu.initialize(myIMUparameters);

        telemetryManagerSub.setDefaultCommand(new PerpetualCommand(new TelemetryManagerCMD(telemetryManagerSub)));


        mecanumDriveBaseSub.setDefaultCommand(new TeleOpJoystickRobotCentricCMD(mecanumDriveBaseSub,
                telemetryManagerSub.getTelemetryObject(), driverOP::getLeftY, driverOP::getLeftX, driverOP::getRightX,
                frontLeft, frontRight, backLeft, backRight));

        shoulderSub.setDefaultCommand(new MoveShoulderCMD(shoulderSub, shoulderMotor,telemetryManagerSub.getTelemetryObject() ));

        vacuumButton.whileHeld(new PowerVacuumCMD(vacuumSubsystem, 1,ContinousVacuumServo)).whenReleased(new PowerVacuumCMD(vacuumSubsystem, 0,ContinousVacuumServo));


    }




}

