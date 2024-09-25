package Command_Based_TeleOp_2024_08_17.auto;


import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;


@Autonomous(name = "Command Auto")
public class basicAutoOpMode extends LinearOpMode {

    FtcDashboard dashboard;
    Telemetry main_dashboardTelemetry;
    private BNO055IMU imu;


    double fwdPwr;
    double strafePwr;
    double rotationPwr;

    private Motor frontLeft;
    private Motor frontRight;
    private Motor backLeft;
    private Motor backRight;

    public GamepadEx driverOP;
    public double dpp = (3 * Math.PI) / 560;
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        waitForStart();
        runtime.reset();

        // the current position of the motor

        frontLeft = new Motor(hardwareMap, "front_left");
        frontRight = new Motor(hardwareMap, "front_right");
        backLeft = new Motor(hardwareMap, "back_left");
        backRight = new Motor(hardwareMap, "back_right");

        // reset the encoder
        frontLeft.resetEncoder();
        frontRight.resetEncoder();
        backLeft.resetEncoder();
        backRight.resetEncoder();

        frontLeft.setRunMode(Motor.RunMode.PositionControl);
        frontRight.setRunMode(Motor.RunMode.PositionControl);
        backLeft.setRunMode(Motor.RunMode.PositionControl);
        backRight.setRunMode(Motor.RunMode.PositionControl);

        frontLeft.setPositionCoefficient(0.05);
        frontRight.setPositionCoefficient(0.05);
        backLeft.setPositionCoefficient(0.05);
        backRight.setPositionCoefficient(0.05);

        frontLeft.setPositionTolerance(13.6);
        frontRight.setPositionTolerance(13.6);
        backLeft.setPositionTolerance(13.6);
        backRight.setPositionTolerance(13.6);


        driverOP = new GamepadEx(gamepad1);



        frontLeft.setTargetPosition(1200);
        frontRight.setTargetPosition(1200);
        backLeft.setTargetPosition(1200);
        backRight.setTargetPosition(1200);

        frontLeft.set(0);
        frontRight.set(0);
        backRight.set(0);
        backLeft.set(0);

        frontLeft.resetEncoder();
        frontRight.resetEncoder();
        backRight.resetEncoder();
        backLeft.resetEncoder();



        dashboard = FtcDashboard.getInstance();
        main_dashboardTelemetry = dashboard.getTelemetry();



        boolean motorsAtTargetPosition = frontLeft.atTargetPosition() && frontRight.atTargetPosition() && backLeft.atTargetPosition() && backRight.atTargetPosition();


        while (opModeIsActive()) {

            //when all motors are at target position, stop runningthem
            motorsAtTargetPosition = frontLeft.atTargetPosition() && frontRight.atTargetPosition() && backLeft.atTargetPosition() && backRight.atTargetPosition();
            if(motorsAtTargetPosition)
            {
                frontLeft.stopMotor();
                frontRight.stopMotor();
                backLeft.stopMotor();
                backRight.stopMotor();

            }

            if(!frontLeft.atTargetPosition()){
                frontLeft.set(0.5);
            }
            else{
                frontLeft.stopMotor();
            }


            if(!frontRight.atTargetPosition()){
                frontRight.set(0.5);
            }
            else{
                frontRight.stopMotor();
            }

            if(!backRight.atTargetPosition()){
                backRight.set(0.5);
            }
            else{
                backRight.stopMotor();
            }


            if(!backLeft.atTargetPosition()){
                backLeft.set(0.5);
            }
            else{
                backLeft.stopMotor();
            }
            main_dashboardTelemetry.addData("frontLeft_atPosition", frontLeft.atTargetPosition());
            main_dashboardTelemetry.addData("frontRight_atPosition", frontRight.atTargetPosition());
            main_dashboardTelemetry.addData("backLeft_atPosition", backLeft.atTargetPosition());
            main_dashboardTelemetry.addData("backRight_atPosition", backRight.atTargetPosition());

            main_dashboardTelemetry.addData("frontLeft_Position", frontLeft.getCurrentPosition());
            main_dashboardTelemetry.addData("frontRight_Position", frontRight.getCurrentPosition());
            main_dashboardTelemetry.addData("backLeft_Position", backLeft.getCurrentPosition());
            main_dashboardTelemetry.addData("backRight_Position", backRight.getCurrentPosition());

            main_dashboardTelemetry.addData("motorAtPositon",motorsAtTargetPosition );

            main_dashboardTelemetry.update();

        }


    }
}
