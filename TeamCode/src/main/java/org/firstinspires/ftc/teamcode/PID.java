package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

public class PID extends LinearOpMode {
    DcMotorEx Motor;

    double intergalsum = 0;
    double kp = 0;
    double ki = 0;
    double kd = 0;
    double kf = 10;

    ElapsedTime timer = new ElapsedTime();
    private double lastError = 0;

    @Override
    public void runOpMode() throws InterruptedException {
    Motor = hardwareMap.get(DcMotorEx.class, "Motor");
    Motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    waitForStart();

    while(opModeIsActive()){
    double power = PIDControl(1000, Motor.getVelocity());
    Motor.setPower(power);


        }

    }
    public double PIDControl(double reference,double state){
        double error = reference - state;
        intergalsum += error * timer.seconds();
        double directive = (error - lastError) / timer.seconds();
        lastError = error;
        timer.reset();


        double output = (error * kp) + (directive * kd) + (intergalsum * ki) + (reference * kf);
        return output;
    }
}
