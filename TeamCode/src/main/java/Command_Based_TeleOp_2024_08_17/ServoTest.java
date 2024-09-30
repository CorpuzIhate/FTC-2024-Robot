package Command_Based_TeleOp_2024_08_17;

import com.acmerobotics.dashboard.config.Config;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;

@TeleOp(name = " Servo_test")

@Config
public class ServoTest extends OpMode {
    CRServo ContinousVacuumServo;
    public static double power = 1;

    @Override
    public void init(){

        ContinousVacuumServo  = hardwareMap.get(CRServo.class,"Vacuum_Servo" );



    }
    @Override
    public void loop(){
        while(gamepad1.a){
            ContinousVacuumServo.setPower(1);
        }
        while(gamepad1.b){
            ContinousVacuumServo.setPower(-1);
        }
        ContinousVacuumServo.setPower(0);

    }
}
