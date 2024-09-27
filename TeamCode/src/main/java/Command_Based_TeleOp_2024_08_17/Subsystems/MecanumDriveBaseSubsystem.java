package Command_Based_TeleOp_2024_08_17.Subsystems;


import com.arcrobotics.ftclib.command.SubsystemBase;

;


public class MecanumDriveBaseSubsystem extends SubsystemBase {




    public MecanumDriveBaseSubsystem(){


    }
    @Override
    public void  periodic(){

    }
    public double[] setMotorSpeeds(double forwardPower, double strafePower,
                                  double rotationPower){
        double[] motorSpeeds = new double[4];
        forwardPower *= -1;
        strafePower *= -1;
        rotationPower *= -1;
        

        double frontLeftSpeed = forwardPower - strafePower - rotationPower;
        double backLeftSpeed = forwardPower + strafePower - rotationPower;
        double frontRightSpeed = forwardPower + strafePower + rotationPower;
        double backRightSpeed= forwardPower -strafePower + rotationPower;

        //math.max tale 2 doubles and figure out which one is higher
        // This is used to determine the current max speed as different sides of the robot
        // may have their motors moving faster


        double max = Math.max(Math.abs(frontLeftSpeed), Math.abs(frontRightSpeed));

        //first we compare the front motors. then we compare that with the back motors to find
        // the fastest motor
        max = Math.max(max, Math.abs(backLeftSpeed));
        max = Math.max(max, Math.abs(backRightSpeed));


        // if the faster motor at the moment has a power over 1, we divide all motors by the max
        if (max > 1.0) {
            frontLeftSpeed /= max;
            frontRightSpeed /= max;
            backLeftSpeed /= max;
            backRightSpeed /= max;
        }
        motorSpeeds[0] = frontLeftSpeed;
        motorSpeeds[1] = frontRightSpeed;
        motorSpeeds[2] = backLeftSpeed;
        motorSpeeds[3] = backRightSpeed;

        return motorSpeeds;

    }





}
