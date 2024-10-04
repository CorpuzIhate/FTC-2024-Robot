import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.controller.wpilibcontroller.ArmFeedforward;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config

public class PidShoulderTest extends OpMode {
    public static double currentSetpoint;
    public static double kP = 1;
    public static double kI = 0;
    public static double kD = 1;
    public static double kF = 1;
    PIDFController feedforward;

    double output;

    private final FtcDashboard dashboard = FtcDashboard.getInstance();
    private final Telemetry dashboardTelemetry = dashboard.getTelemetry();

    Motor shoulderMotor;
    @Override
    public void init(){
        feedforward = new PIDFController(kP, kI, kD, kF);

        shoulderMotor = new Motor(hardwareMap,"shoulder_motor");
        shoulderMotor.setRunMode(Motor.RunMode.VelocityControl);
    }
    public void loop(){
        output = feedforward.calculate(shoulderMotor.getCurrentPosition(), currentSetpoint);

    }


}
